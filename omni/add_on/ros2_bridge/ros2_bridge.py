import time
import json
import math
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from trajectory_msgs.msg import JointTrajectoryPoint

import omni.add_on.RosBridgeSchema as ROSSchema
import omni.add_on.RosControlBridgeSchema as ROSControlSchema


# message types
GetPrims = None
GetPrimAttributes = None
GetPrimAttribute = None
SetPrimAttribute = None
FollowJointTrajectory = None


def acquire_ros2_bridge_interface(plugin_name=None, library_path=None):
    global GetPrims, GetPrimAttributes, GetPrimAttribute, SetPrimAttribute, FollowJointTrajectory
    
    from add_on_msgs.srv import GetPrims as get_prims_srv
    from add_on_msgs.srv import GetPrimAttributes as get_prim_attributes_srv
    from add_on_msgs.srv import GetPrimAttribute as get_prim_attribute_srv
    from add_on_msgs.srv import SetPrimAttribute as set_prim_attribute_srv
    from control_msgs.action import FollowJointTrajectory as follow_joint_trajectory_action

    GetPrims = get_prims_srv
    GetPrimAttributes = get_prim_attributes_srv
    GetPrimAttribute = get_prim_attribute_srv
    SetPrimAttribute = set_prim_attribute_srv
    FollowJointTrajectory = follow_joint_trajectory_action

    rclpy.init()
    bridge = Ros2Bridge()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin).start()

    return bridge

def release_ros2_bridge_interface(bridge):
    bridge.shutdown()


class Ros2Bridge(Node):
    def __init__(self):
        node_name = carb.settings.get_settings().get("/exts/omni.add_on.ros2_bridge/nodeName")
        super().__init__(node_name)

        self._components = []

        # omni objects and interfaces
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._dci = _dynamic_control.acquire_dynamic_control_interface()
        
        # events
        self._update_event = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_event)
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
        self._physx_event = self._physx_interface.subscribe_physics_step_events(self._on_physics_event)

    def shutdown(self):
        self._update_event = None
        self._timeline_event = None
        self._stage_event = None

        self._stop_components()
        rclpy.shutdown()

    def _get_ros_bridge_schemas(self):
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosAttribute":
                schemas.append(ROSSchema.RosAttribute(prim))
            elif prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
        return schemas

    def _stop_components(self):
        for component in self._components:
            component.stop()

    def _reload_components(self):
        # stop components
        self._stop_components()
        # load components
        self._components = []
        self._skip_update_step = True
        for schema in self._get_ros_bridge_schemas():
            if schema.__class__.__name__ == "RosAttribute":
                self._components.append(RosAttribute(self, self._usd_context, schema, self._dci))
            elif schema.__class__.__name__ == "RosControlFollowJointTrajectory":
                self._components.append(RosControlFollowJointTrajectory(self, self._usd_context, schema, self._dci))
                
    def _on_update_event(self, event):
        if self._timeline.is_playing():
            for component in self._components:
                if self._skip_update_step:
                    self._skip_update_step = False
                    return
                # start components
                if not component.started:
                    component.start()
                    return
                # step
                component.update_step(event.payload["dt"])
    
    def _on_timeline_event(self, event):
        # reload components
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[INFO] Ros2Bridge: components reloaded")
        # stop components
        elif event.type == int(omni.timeline.TimelineEventType.STOP) or event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._stop_components()
            print("[INFO] Ros2Bridge: components stopped")

    def _on_stage_event(self, event):
        pass

    def _on_physics_event(self, step):
        for component in self._components:
            component.physics_step(step)


class RosController():
    def __init__(self, node, usd_context, schema):
        self._node = node
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False
    
    def start(self):
        raise NotImplementedError

    def stop(self):
        print("[INFO] RosController: stopping", self._schema.__class__.__name__)
        self.started = False

    def update_step(self, dt):
        raise NotImplementedError

    def physics_step(self, step):
        raise NotImplementedError


class RosAttribute(RosController):
    def __init__(self, node, usd_context, schema, dci):
        super(RosAttribute, self).__init__(node, usd_context, schema)

        self._dci = dci

        self._srv_prims = None
        self._srv_attributes = None
        self._srv_getter = None
        self._srv_setter = None

        self._value = None
        self._attribute = None

        self._event = threading.Event()
        self._event.set()

        self.__event_timeout = carb.settings.get_settings().get("/exts/omni.add_on.ros2_bridge/eventTimeout")
        self.__set_attribute_using_asyncio = carb.settings.get_settings().get("/exts/omni.add_on.ros2_bridge/setAttributeUsingAsyncio")
        print("[INFO] RosAttribute [asyncio: {}]".format(self.__set_attribute_using_asyncio))
        print("[INFO] RosAttribute [event timeout: {}]".format(self.__event_timeout))

    async def _set_attribute(self, attribute, attribute_value):
        ret = attribute.Set(attribute_value)

    def _process_setter_request(self, request, response):
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                if request.attribute and prim.HasAttribute(request.attribute):
                    # attribute
                    attribute = prim.GetAttribute(request.attribute)
                    attribute_type = type(attribute.Get()).__name__
                    try:
                        # value
                        value = json.loads(request.value)
                        attribute_value = None

                        # parse data
                        if attribute_type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['Quatd', 'Quatf', 'Quath']:
                            attribute_value = type(attribute.Get())(*value)
                        elif attribute_type in ['Matrix4d', 'Matrix4f']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type.startswith('Vec') and attribute_type.endswith('Array'):
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type.startswith('Matrix') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute_value = type(attribute.Get())([Gf.Matrix2d(v) for v in value])
                            elif attribute_type.endswith("fArray"):
                                attribute_value = type(attribute.Get())([Gf.Matrix2f(v) for v in value])
                        elif attribute_type.startswith('Quat') and attribute_type.endswith('Array'):
                            if attribute_type.endswith("dArray"):
                                attribute_value = type(attribute.Get())([Gf.Quatd(*v) for v in value])
                            elif attribute_type.endswith("fArray"):
                                attribute_value = type(attribute.Get())([Gf.Quatf(*v) for v in value]) 
                            elif attribute_type.endswith("hArray"):
                                attribute_value = type(attribute.Get())([Gf.Quath(*v) for v in value])
                        elif attribute_type.endswith('Array'):
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['AssetPath']:
                            attribute_value = type(attribute.Get())(value)
                        elif attribute_type in ['NoneType']:
                            pass
                        else:
                            attribute_value = type(attribute.Get())(value)
                        
                        # set attribute
                        if attribute_value is not None:
                            # set attribute usign asyncio
                            if self.__set_attribute_using_asyncio:
                                try:
                                    loop = asyncio.get_event_loop()
                                except:
                                    loop = asyncio.new_event_loop()
                                asyncio.set_event_loop(loop)
                                future = asyncio.ensure_future(self._set_attribute(attribute, attribute_value))
                                loop.run_until_complete(future)
                                response.success = True
                            # set attribute in the physics event
                            else:
                                self._attribute = attribute
                                self._value = attribute_value

                                self._event.clear()
                                response.success = self._event.wait(self.__event_timeout)
                                if not response.success:
                                    response.message = "The timeout ({} s) for setting the attribute value has been reached".format(self.__event_timeout)
                                
                    except Exception as e:
                        print("[ERROR] srv {} request for {} ({}: {}): {}".format(self._srv_setter.srv_name, request.path, request.attribute, request.value, e))
                        response.success = False
                        response.message = str(e)
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
        
    def _process_getter_request(self, request, response):
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                if request.attribute and prim.HasAttribute(request.attribute):
                    attribute = prim.GetAttribute(request.attribute)
                    response.type = type(attribute.Get()).__name__
                    # parse data
                    response.success = True
                    if response.type in ['Vec2d', 'Vec2f', 'Vec2h', 'Vec2i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(2)])
                    elif response.type in ['Vec3d', 'Vec3f', 'Vec3h', 'Vec3i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(3)])
                    elif response.type in ['Vec4d', 'Vec4f', 'Vec4h', 'Vec4i']:
                        data = attribute.Get()
                        response.value = json.dumps([data[i] for i in range(4)])
                    elif response.type in ['Quatd', 'Quatf', 'Quath']:
                        data = attribute.Get()
                        response.value = json.dumps([data.real, data.imaginary[0], data.imaginary[1], data.imaginary[2]])                    
                    elif response.type in ['Matrix4d', 'Matrix4f']:
                        data = attribute.Get()
                        response.value = json.dumps([[data.GetRow(i)[j] for j in range(data.dimension[1])] for i in range(data.dimension[0])])
                    elif response.type.startswith('Vec') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d[i] for i in range(len(d))] for d in data])
                    elif response.type.startswith('Matrix') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[[d.GetRow(i)[j] for j in range(d.dimension[1])] for i in range(d.dimension[0])] for d in data])
                    elif response.type.startswith('Quat') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d.real, d.imaginary[0], d.imaginary[1], d.imaginary[2]] for d in data])
                    elif response.type.endswith('Array'):
                        try:
                            response.value = json.dumps(list(attribute.Get()))
                        except Exception as e:
                            print("[UNKNOW]", type(attribute.Get()))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                    elif response.type in ['AssetPath']:
                        response.value = json.dumps(str(attribute.Get().path))
                    else:
                        try:
                            response.value = json.dumps(attribute.Get())
                        except Exception as e:
                            print("[UNKNOW]", type(attribute.Get()), attribute.Get())
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def _process_attributes_request(self, request, response):
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prim
            if stage.GetPrimAtPath(request.path).IsValid():
                prim = stage.GetPrimAtPath(request.path)
                for attribute in prim.GetAttributes():
                    if attribute.GetNamespace():
                        response.names.append("{}:{}".format(attribute.GetNamespace(), attribute.GetBaseName()))
                    else:
                        response.names.append(attribute.GetBaseName())
                    response.displays.append(attribute.GetDisplayName())
                    response.types.append(type(attribute.Get()).__name__)
                    response.success = True
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
    
    def _process_prims_request(self, request, response):
        response.success = False
        if self._schema.GetEnabledAttr().Get():
            stage = self._usd_context.get_stage()
            # get prims
            if not request.path or stage.GetPrimAtPath(request.path).IsValid():
                path = request.path if request.path else "/"
                for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath(path)):
                    response.paths.append(str(prim.GetPath()))
                    response.types.append(prim.GetTypeName())
                    response.success = True
            else:
                response.message = "Invalid search path ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def start(self):
        self.started = True
        print("[INFO] RosAttribute: starting", self._schema.__class__.__name__)

        service_name = self._schema.GetPrimsSrvTopicAttr().Get()
        self._srv_prims = self._node.create_service(GetPrims, service_name, self._process_prims_request)
        print("[INFO] RosAttribute: register srv:", self._srv_prims.srv_name)

        service_name = self._schema.GetGetAttrSrvTopicAttr().Get()
        self._srv_getter = self._node.create_service(GetPrimAttribute, service_name, self._process_getter_request)
        print("[INFO] RosAttribute: register srv:", self._srv_getter.srv_name)

        service_name = self._schema.GetAttributesSrvTopicAttr().Get()
        self._srv_attributes = self._node.create_service(GetPrimAttributes, service_name, self._process_attributes_request)
        print("[INFO] RosAttribute: register srv:", self._srv_attributes.srv_name)

        service_name = self._schema.GetSetAttrSrvTopicAttr().Get()
        self._srv_setter = self._node.create_service(SetPrimAttribute, service_name, self._process_setter_request)
        print("[INFO] RosAttribute: register srv:", self._srv_setter.srv_name)
        
    def stop(self):
        if self._srv_prims is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_prims.srv_name)
            self._node.destroy_service(self._srv_prims)
            self._srv_prims = None
        if self._srv_getter is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_getter.srv_name)
            self._node.destroy_service(self._srv_getter)
            self._srv_getter = None
        if self._srv_attributes is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_attributes.srv_name)
            self._node.destroy_service(self._srv_attributes)
            self._srv_attributes = None
        if self._srv_setter is not None:
            print("[INFO] RosAttribute: unregister srv:", self._srv_setter.srv_name)
            self._node.destroy_service(self._srv_setter)
            self._srv_setter = None
        super(RosAttribute, self).stop()

    def update_step(self, dt):
        pass

    def physics_step(self, step):
        if self.__set_attribute_using_asyncio:
            return
        if self._dci.is_simulating():
            if not self._event.is_set():
                if self._attribute is not None:
                    ret = self._attribute.Set(self._value)
                self._event.set()


# RosControlFollowJointTrajectory (previously in trajectory_follower.py in webots_ros2)
#
# Copyright 2022 Florent AUDONNET
# Copyright 1996-2021 Cyberbotics Ltd.
# 
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# This class has been modified by Florent AUDONNET to add support for Nvidia Isaac.

class RosControlFollowJointTrajectory(RosController):
    def __init__(self, node, usd_context, schema, dci):
        super(RosControlFollowJointTrajectory, self).__init__(node, usd_context, schema)
        
        self._dci = dci

        self._dt = 0.05
        self._joints = {}
        self._action_server = None
       
        # TODO: add it to the schema
        self.__default_tolerance = 0.05

        self.__goal = None
        self.__start_time = None
        self.__tolerances = {}
        self.__current_point_index = 1
        
    def start(self):
        print("[INFO] RosControlFollowJointTrajectory: starting", self._schema.__class__.__name__)
        
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()
        relationships = self._schema.GetArticulationPrimRel().GetTargets()

        if not len(relationships):
            print("[WARNING] RosControllerFollowJointTrajectory: empty relationships")
            return

        self._action_server = ActionServer(self._node,
                                           FollowJointTrajectory,
                                           controller_name + action_namespace,
                                           execute_callback=self.__on_update,
                                           goal_callback=self.__on_goal,
                                           cancel_callback=self.__on_cancel,
                                           handle_accepted_callback=self.__on_goal_accepted)
        print("[INFO] RosControlFollowJointTrajectory: register action:", controller_name + action_namespace)

        self.started = True

    def stop(self):
        self.__goal = None
        # destroy action server
        if self._action_server is not None:
            print("[INFO] RosControlFollowJointTrajectory: destroy action server")
            # self._action_server.destroy()
            self._action_server = None
        super(RosControlFollowJointTrajectory, self).stop()

    def _duration_to_seconds(self, duration):
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self):
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        ar = self._dci.get_articulation(path)
        if ar == _dynamic_control.INVALID_HANDLE:
            print("[WARNING] RosControllerFollowJointTrajectory: prim {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(ar)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        for index in range(self._dci.get_articulation_dof_count(ar)):
            dof_ptr = self._dci.get_articulation_dof(ar, index)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                name = self._dci.get_dof_name(dof_ptr)
                if name in self._joints:
                    continue
                self._joints[name] = {"ar": ar, 
                                      "index": index, 
                                      "dof": self._dci.find_articulation_dof(ar, name), 
                                      "lower": lower_limits[index] / get_stage_units(), 
                                      "upper": upper_limits[index] / get_stage_units(), 
                                      "joint": self._dci.find_articulation_joint(ar, name),
                                      "has_limits": has_limits[index]}

        if not self._joints:
            print("[WARNING] RosControllerFollowJointTrajectory: no joints found")
            self.started = False

    def __on_goal_accepted(self, goal_handle):
        goal_handle.execute()

    def __on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # reject if joints don't match
        for name in goal_handle.trajectory.joint_names:
            if name not in self._joints.keys():
                print("[ERROR] RosControlFollowJointTrajectory: Received a goal with incorrect joint names ({} not in {})".format(name, list(self._joints.keys())))
                return GoalResponse.REJECT

        # reject if infinity or NaN
        for point in goal_handle.trajectory.points:
            for position, velocity in zip(point.positions, point.velocities):
                if math.isinf(position) or math.isnan(position) or math.isinf(velocity) or math.isnan(velocity):
                    print("[ERROR] RosControlFollowJointTrajectory: Received a goal with infinites or NaNs")
                    return GoalResponse.REJECT

        # reject if joints are already controlled
        if self.__goal is not None:
            print("[ERROR] RosControlFollowJointTrajectory: Cannot accept multiple goals")
            return GoalResponse.REJECT

        # store goal data
        self.__goal = goal_handle
        self.__current_point_index = 1
        self.__start_time = self._node.get_clock().now().nanoseconds / 1e9

        for tolerance in self.__goal.goal_tolerance:
            self.__tolerances[tolerance.name] = tolerance.position
        for name in self.__goal.trajectory.joint_names:
            if name not in self.__tolerances.keys():
                self.__tolerances[name] = self.__default_tolerance

        # if the user forget the initial position
        if self._duration_to_seconds(self.__goal.trajectory.points[0].time_from_start) != 0:
            initial_point = JointTrajectoryPoint(positions=[self.__get_joint_position(name) for name in self.__goal.trajectory.joint_names],
                                                 time_from_start=Duration().to_msg())
            self.__goal.trajectory.points.insert(0, initial_point)
        
        self._feedback_message = FollowJointTrajectory.Feedback()
        self._feedback_message.joint_names = list(self.__goal.trajectory.joint_names)

        self._result = None
        return GoalResponse.ACCEPT

    def __on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if self.__goal is not None:
            self.__goal = None
            self._feedback_message = None
            self._result = None
            goal_handle.destroy()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def update_step(self, dt):
        pass

    def physics_step(self, dt):
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        self._dt = dt
        if self.__goal is not None:
            if self.__current_point_index >= len(self.__goal.trajectory.points):
                self.__goal = None
                self._goal_handle.succeed()
                self._result = FollowJointTrajectory.Result()
                self._result.error_code = self._result.SUCCESSFUL
                return
            
            now = self._node.get_clock().now().nanoseconds / 1e9
            prev_point = self.__goal.trajectory.points[self.__current_point_index - 1]
            curr_point = self.__goal.trajectory.points[self.__current_point_index]
            time_passed = now - self.__start_time

            if time_passed <= self._duration_to_seconds(curr_point.time_from_start):
                # linear interpolation
                ratio = (time_passed - self._duration_to_seconds(prev_point.time_from_start)) \
                      / (self._duration_to_seconds(curr_point.time_from_start) - self._duration_to_seconds(prev_point.time_from_start))
                for index, name in enumerate(self.__goal.trajectory.joint_names):
                    side = -1 if curr_point.positions[index] < prev_point.positions[index] else 1
                    target_position = prev_point.positions[index] + \
                        side * ratio * abs(curr_point.positions[index] - prev_point.positions[index])
                    self.__set_joint_position(name, target_position)
            else:
                self.__current_point_index += 1
                time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self.__start_time
                self._feedback_message.actual.positions = [self.__get_joint_position(name) for name in self.__goal.trajectory.joint_names]
                self._feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
                self._goal_handle.publish_feedback(self._feedback_message)

    def __set_joint_position(self, name, target_position):
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])

        self._dci.wake_up_articulation(self._joints[name]["ar"])
        if self._dci.get_joint_type(self._joints[name]["joint"]) == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def __get_joint_position(self, name):
        dof_state = self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS)
        return dof_state.pos

    async def __on_update(self, goal_handle):
        self._goal_handle = goal_handle
        while self._result is None: 
            if self.__goal is None:
                result = FollowJointTrajectory.Result()
                result.error_code = result.PATH_TOLERANCE_VIOLATED
                return result

            time.sleep(self._dt)
        return self._result
