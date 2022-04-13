from typing import List, Any

import time
import json
import asyncio
import threading

import omni
import carb
import omni.kit
from pxr import Usd, Gf, PhysxSchema
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
GripperCommand = None


def acquire_ros2_bridge_interface(ext_id: str = "") -> 'Ros2Bridge':
    """
    Acquire the Ros2Bridge interface

    :param ext_id: The extension id
    :type ext_id: str

    :returns: The Ros2Bridge interface
    :rtype: Ros2Bridge
    """
    global GetPrims, GetPrimAttributes, GetPrimAttribute, SetPrimAttribute, FollowJointTrajectory, GripperCommand
    
    from add_on_msgs.srv import GetPrims as get_prims_srv
    from add_on_msgs.srv import GetPrimAttributes as get_prim_attributes_srv
    from add_on_msgs.srv import GetPrimAttribute as get_prim_attribute_srv
    from add_on_msgs.srv import SetPrimAttribute as set_prim_attribute_srv
    from control_msgs.action import FollowJointTrajectory as follow_joint_trajectory_action
    from control_msgs.action import GripperCommand as gripper_command_action

    GetPrims = get_prims_srv
    GetPrimAttributes = get_prim_attributes_srv
    GetPrimAttribute = get_prim_attribute_srv
    SetPrimAttribute = set_prim_attribute_srv
    FollowJointTrajectory = follow_joint_trajectory_action
    GripperCommand = gripper_command_action

    rclpy.init()
    bridge = Ros2Bridge()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin).start()

    return bridge

def release_ros2_bridge_interface(bridge: 'Ros2Bridge') -> None:
    """
    Release the Ros2Bridge interface
    
    :param bridge: The Ros2Bridge interface
    :type bridge: Ros2Bridge
    """
    bridge.shutdown()


class Ros2Bridge(Node):
    def __init__(self) -> None:
        """Initialize the Ros2Bridge interface
        """
        self._components = []
        self._node_name = carb.settings.get_settings().get("/exts/omni.add_on.ros2_bridge/nodeName")
        
        super().__init__(self._node_name)

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

    def shutdown(self) -> None:
        """Shutdown the Ros2Bridge interface
        """
        self._update_event = None
        self._timeline_event = None
        self._stage_event = None

        self._stop_components()
        self.destroy_node()
        rclpy.shutdown()

    def _get_ros_bridge_schemas(self) -> List['ROSSchema.RosBridgeComponent']:
        """Get the ROS bridge schemas in the current stage

        :returns: The ROS bridge schemas
        :rtype: list of RosBridgeComponent
        """
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosAttribute":
                schemas.append(ROSSchema.RosAttribute(prim))
            elif prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
            elif prim.GetTypeName() == "RosControlGripperCommand":
                schemas.append(ROSControlSchema.RosControlGripperCommand(prim))
        return schemas

    def _stop_components(self) -> None:
        """Stop all components
        """
        for component in self._components:
            component.stop()

    def _reload_components(self) -> None:
        """Reload all components
        """
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
            elif schema.__class__.__name__ == "RosControlGripperCommand":
                self._components.append(RosControllerGripperCommand(self, self._usd_context, schema, self._dci))
                
    def _on_update_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the kit update event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
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
    
    def _on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        # reload components
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[Info][omni.add_on.ros2_bridge] RosControlBridge: components reloaded")
        # stop components
        elif event.type == int(omni.timeline.TimelineEventType.STOP) or event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._stop_components()
            print("[Info][omni.add_on.ros2_bridge] RosControlBridge: components stopped")

    def _on_stage_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the stage event

        :param event: The stage event
        :type event: carb.events._events.IEvent
        """
        pass

    def _on_physics_event(self, step: float) -> None:
        """Handle the physics event

        :param step: The physics step
        :type step: float
        """
        for component in self._components:
            component.physics_step(step)


class RosController:
    def __init__(self, node: Node, usd_context: 'omni.usd._usd.UsdContext', schema: 'ROSSchema.RosBridgeComponent') -> None:
        """Base class for RosController

        :param node: ROS2 node
        :type node: Node
        :param usd_context: USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosBridgeComponent
        """
        self._node = node
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False
    
    def start(self) -> None:
        """Start the component
        """
        raise NotImplementedError

    def stop(self) -> None:
        """Stop the component
        """
        print("[Info][omni.add_on.ros2_bridge] RosController: stopping {}".format(self._schema.__class__.__name__))
        self.started = False

    def update_step(self, dt: float) -> None:
        """Kit update step
        
        :param dt: The delta time
        :type dt: float
        """
        raise NotImplementedError

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        raise NotImplementedError


class RosAttribute(RosController):
    def __init__(self, 
                 node: Node, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """RosAttribute interface

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param usd_context: USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosAttribute
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node, usd_context, schema)

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
        self.__set_attribute_using_asyncio = \
            carb.settings.get_settings().get("/exts/omni.add_on.ros2_bridge/setAttributeUsingAsyncio")
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: asyncio: {}".format(self.__set_attribute_using_asyncio))
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: event timeout: {}".format(self.__event_timeout))

    async def _set_attribute(self, attribute: 'pxr.Usd.Attribute', attribute_value: Any) -> None:
        """Set the attribute value using asyncio

        :param attribute: The prim's attribute to set
        :type attribute: pxr.Usd.Attribute
        :param attribute_value: The attribute value
        :type attribute_value: Any
        """
        ret = attribute.Set(attribute_value)

    def _process_setter_request(self, 
                                request: 'SetPrimAttribute.Request', 
                                response: 'SetPrimAttribute.Response') -> 'SetPrimAttribute.Response':
        """Process the setter request

        :param request: The service request
        :type request: SetPrimAttribute.Request
        :param response: The service response
        :type response: SetPrimAttribute.Response

        :return: The service response
        :rtype: SetPrimAttribute.Response
        """
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
                                    response.message = "The timeout ({} s) for setting the attribute value has been reached" \
                                        .format(self.__event_timeout)
                                
                    except Exception as e:
                        print("[Error][omni.add_on.ros2_bridge] RosAttribute: srv {} request for {} ({}: {}): {}" \
                            .format(self._srv_setter.resolved_name, request.path, request.attribute, value, e))
                        response.success = False
                        response.message = str(e)
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response
        
    def _process_getter_request(self, 
                                request: 'GetPrimAttribute.Request', 
                                response: 'GetPrimAttribute.Response') -> 'GetPrimAttribute.Response':
        """Process the getter request

        :param request: The service request
        :type request: GetPrimAttribute.Request
        :param response: The service response
        :type response: GetPrimAttribute.Response

        :return: The service response
        :rtype: GetPrimAttribute.Response
        """
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
                        response.value = json.dumps([[data.GetRow(i)[j] for j in range(data.dimension[1])] \
                            for i in range(data.dimension[0])])
                    elif response.type.startswith('Vec') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d[i] for i in range(len(d))] for d in data])
                    elif response.type.startswith('Matrix') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[[d.GetRow(i)[j] for j in range(d.dimension[1])] \
                            for i in range(d.dimension[0])] for d in data])
                    elif response.type.startswith('Quat') and response.type.endswith('Array'):
                        data = attribute.Get()
                        response.value = json.dumps([[d.real, d.imaginary[0], d.imaginary[1], d.imaginary[2]] for d in data])
                    elif response.type.endswith('Array'):
                        try:
                            response.value = json.dumps(list(attribute.Get()))
                        except Exception as e:
                            print("[Warning][omni.add_on.ros2_bridge] RosAttribute: Unknow attribute type {}" \
                                .format(type(attribute.Get())))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros2_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                    elif response.type in ['AssetPath']:
                        response.value = json.dumps(str(attribute.Get().path))
                    else:
                        try:
                            response.value = json.dumps(attribute.Get())
                        except Exception as e:
                            print("[Warning][omni.add_on.ros2_bridge] RosAttribute: Unknow {}: {}" \
                                .format(type(attribute.Get()), attribute.Get()))
                            print("  |-- Please, report a new issue (https://github.com/Toni-SM/omni.add_on.ros2_bridge/issues)")
                            response.success = False
                            response.message = "Unknow type {}".format(type(attribute.Get()))
                else:
                    response.message = "Prim has not attribute {}".format(request.attribute)
            else:
                response.message = "Invalid prim ({})".format(request.path)
        else:
            response.message = "RosAttribute prim is not enabled"
        return response

    def _process_attributes_request(self, 
                                    request: 'GetPrimAttributes.Request', 
                                    response: 'GetPrimAttributes.Response') -> 'GetPrimAttributes.Response':
        """Process the 'get all attributes' request

        :param request: The service request
        :type request: GetPrimAttributes.Request
        :param response: The service response
        :type response: GetPrimAttributes.Response

        :return: The service response
        :rtype: GetPrimAttributes.Response
        """
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
    
    def _process_prims_request(self, request: 'GetPrims.Request', response: 'GetPrims.Response') -> 'GetPrims.Response':
        """Process the 'get all prims' request

        :param request: The service request
        :type request: GetPrims.Request
        :param response: The service response
        :type response: GetPrims.Response

        :return: The service response
        :rtype: GetPrims.Response
        """
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

    def start(self) -> None:
        """Start the services
        """
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: starting {}".format(self._schema.__class__.__name__))

        service_name = self._schema.GetPrimsSrvTopicAttr().Get()
        self._srv_prims = self._node.create_service(GetPrims, service_name, self._process_prims_request)
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_prims.srv_name))

        service_name = self._schema.GetGetAttrSrvTopicAttr().Get()
        self._srv_getter = self._node.create_service(GetPrimAttribute, service_name, self._process_getter_request)
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_getter.srv_name))

        service_name = self._schema.GetAttributesSrvTopicAttr().Get()
        self._srv_attributes = self._node.create_service(GetPrimAttributes, service_name, self._process_attributes_request)
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_attributes.srv_name))

        service_name = self._schema.GetSetAttrSrvTopicAttr().Get()
        self._srv_setter = self._node.create_service(SetPrimAttribute, service_name, self._process_setter_request)
        print("[Info][omni.add_on.ros2_bridge] RosAttribute: register srv: {}".format(self._srv_setter.srv_name))
        
        self.started = True

    def stop(self) -> None:
        """Stop the services
        """
        if self._srv_prims is not None:
            print("[Info][omni.add_on.ros2_bridge] RosAttribute: unregister srv: {}".format(self._srv_prims.srv_name))
            self._node.destroy_service(self._srv_prims)
            self._srv_prims = None
        if self._srv_getter is not None:
            print("[Info][omni.add_on.ros2_bridge] RosAttribute: unregister srv: {}".format(self._srv_getter.srv_name))
            self._node.destroy_service(self._srv_getter)
            self._srv_getter = None
        if self._srv_attributes is not None:
            print("[Info][omni.add_on.ros2_bridge] RosAttribute: unregister srv: {}".format(self._srv_attributes.srv_name))
            self._node.destroy_service(self._srv_attributes)
            self._srv_attributes = None
        if self._srv_setter is not None:
            print("[Info][omni.add_on.ros2_bridge] RosAttribute: unregister srv: {}".format(self._srv_setter.srv_name))
            self._node.destroy_service(self._srv_setter)
            self._srv_setter = None
        super().stop()

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        if self.__set_attribute_using_asyncio:
            return
        if self._dci.is_simulating():
            if not self._event.is_set():
                if self._attribute is not None:
                    ret = self._attribute.Set(self._value)
                self._event.set()


class RosControlFollowJointTrajectory(RosController):
    def __init__(self, 
                 node: Node, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """FollowJointTrajectory interface
        
        :param node: The ROS node
        :type node: rclpy.node.Node
        :param usd_context: The USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The schema
        :type schema: ROSSchema.RosBridgeComponent
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node, usd_context, schema)
        
        self._dci = dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = FollowJointTrajectory.Feedback()
        
    def start(self) -> None:
        """Start the action server
        """
        print("[Info][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: starting {}" \
            .format(self._schema.__class__.__name__))
        
        # get attributes and relationships
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: empty relationships")
            return
        
        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: {} doesn't have PhysxArticulationAPI".format(path))
            return

        # start action server
        self._action_server = ActionServer(self._node,
                                           FollowJointTrajectory,
                                           controller_name + action_namespace,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: register action {}" \
            .format(controller_name + action_namespace))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: destroy action server: {}" \
                .format(self._schema.GetPrim().GetPath()))
            # self._action_server.destroy()
            self._action_server = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        self._articulation = self._dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self._dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self._dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self._dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self._dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self._dci.get_joint_type(_joint),
                                              "dof": self._dci.find_articulation_dof(self._articulation, dof_name), 
                                              "lower": lower_limits[i], 
                                              "upper": upper_limits[i], 
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: no joints found in {}".format(path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'FollowJointTrajectory.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: FollowJointTrajectory.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: joints don't match ({} not in {})" \
                    .format(name, list(self._joints.keys())))
                return GoalResponse.REJECT

        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][omni.add_on.ros2_bridge] RosControlFollowJointTrajectory: multiple goals not supported")
            return GoalResponse.REJECT

        # check initial position
        if goal.trajectory.points[0].time_from_start:
            initial_point = JointTrajectoryPoint(positions=[self._get_joint_position(name) for name in goal.trajectory.joint_names],
                                                 time_from_start=Duration().to_msg())
            goal.trajectory.points.insert(0, initial_point)
        
        # reset internal data
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None

        # store goal data
        self._action_goal = goal

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        goal_handle.destroy()
        return CancelResponse.ACCEPT
        
    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'FollowJointTrajectory.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: FollowJointTrajectory.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        # set goal
        self._action_goal_handle = goal_handle
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                result = FollowJointTrajectory.Result()
                result.error_code = result.INVALID_GOAL
                return result
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            self._action_dt = dt
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                self._action_goal = None
                self._action_result_message = FollowJointTrajectory.Result()
                self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            
            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time

            # set target using linear interpolation
            if time_passed <= self._duration_to_seconds(current_point.time_from_start):
                ratio = (time_passed - self._duration_to_seconds(previous_point.time_from_start)) \
                      / (self._duration_to_seconds(current_point.time_from_start) \
                          - self._duration_to_seconds(previous_point.time_from_start))
                self._dci.wake_up_articulation(self._articulation)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = -1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                                    + side * ratio * abs(current_point.positions[i] - previous_point.positions[i])
                    self._set_joint_position(name, target_position)
            # send feedback
            else:
                self._action_point_index += 1
                self._action_feedback_message.actual.positions = [self._get_joint_position(name) \
                    for name in self._action_goal.trajectory.joint_names]
                self._action_feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
                if self._action_goal_handle is not None:
                    self._action_goal_handle.publish_feedback(self._action_feedback_message)


class RosControllerGripperCommand(RosController):
    def __init__(self, 
                 node: Node, 
                 usd_context: 'omni.usd._usd.UsdContext', 
                 schema: 'ROSSchema.RosBridgeComponent', 
                 dci: 'omni.isaac.dynamic_control.DynamicControl') -> None:
        """GripperCommand interface

        :param node: The ROS node
        :type node: rclpy.node.Node
        :param usd_context: The USD context
        :type usd_context: omni.usd._usd.UsdContext
        :param schema: The ROS bridge schema
        :type schema: ROSSchema.RosBridgeComponent
        :param dci: The dynamic control interface
        :type dci: omni.isaac.dynamic_control.DynamicControl
        """
        super().__init__(node, usd_context, schema)
        
        self._dci = dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_server = None

        self._action_dt = 0.05
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = None
        self._action_feedback_message = GripperCommand.Feedback()
        
    def start(self) -> None:
        """Start the action server
        """
        print("[Info][omni.add_on.ros2_bridge] RosControllerGripperCommand: starting {}" \
            .format(self._schema.__class__.__name__))
        
        # get attributes and relationships
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: empty relationships")
            return
        elif len(relationships) == 1:
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: relationship is not a group")
            return
        
        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: {} doesn't have PhysxArticulationAPI".format(path))
            return

        # start action server
        self._action_server = ActionServer(self._node,
                                           GripperCommand,
                                           controller_name + action_namespace,
                                           execute_callback=self._on_execute,
                                           goal_callback=self._on_goal,
                                           cancel_callback=self._on_cancel,
                                           handle_accepted_callback=self._on_handle_accepted)
        print("[Info][omni.add_on.ros2_bridge] RosControllerGripperCommand: register action {}" \
            .format(controller_name + action_namespace))

        self.started = True

    def stop(self) -> None:
        """Stop the action server
        """
        super().stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        # destroy action server
        if self._action_server is not None:
            print("[Info][omni.add_on.ros2_bridge] RosControllerGripperCommand: destroy action server: {}" \
                .format(self._schema.GetPrim().GetPath()))
            # self._action_server.destroy()
            self._action_server = None
        self._action_goal_handle = None
        self._action_goal = None

    def _duration_to_seconds(self, duration: Duration) -> float:
        """Convert a ROS2 Duration to seconds

        :param duration: The ROS2 Duration
        :type duration: Duration

        :return: The duration in seconds
        :rtype: float
        """
        return Duration.from_msg(duration).nanoseconds / 1e9

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        self._articulation = self._dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        # TODO: move to another relationship in the schema
        paths = [relationship.GetPrimPath().pathString for relationship in relationships[1:]]

        for i in range(self._dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self._dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                # add only required joints
                if self._dci.get_dof_path(dof_ptr) in paths:
                    dof_name = self._dci.get_dof_name(dof_ptr)
                    if dof_name not in self._joints:
                        _joint = self._dci.find_articulation_joint(self._articulation, dof_name)
                        self._joints[dof_name] = {"joint": _joint,
                                                  "type": self._dci.get_joint_type(_joint),
                                                  "dof": self._dci.find_articulation_dof(self._articulation, dof_name), 
                                                  "lower": lower_limits[i], 
                                                  "upper": upper_limits[i], 
                                                  "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: no joints found in {}".format(path))
            self.started = False

    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position

    def _on_handle_accepted(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> None:
        """Callback function for handling newly accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle
        """
        goal_handle.execute()

    def _on_goal(self, goal: 'GripperCommand.Goal') -> 'rclpy.action.server.GoalResponse':
        """Callback function for handling new goal requests

        :param goal: The goal
        :type goal: GripperCommand.Goal

        :return: Whether the goal was accepted
        :rtype: rclpy.action.server.GoalResponse
        """
        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][omni.add_on.ros2_bridge] RosControllerGripperCommand: multiple goals not supported")
            return GoalResponse.REJECT
        
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")

        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'rclpy.action.server.CancelResponse':
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: Whether the goal was canceled
        :rtype: rclpy.action.server.CancelResponse
        """
        if self._action_goal is None:
            return CancelResponse.REJECT
        # reset internal data
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        goal_handle.destroy()
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle: 'rclpy.action.server.ServerGoalHandle') -> 'GripperCommand.Result':
        """Callback function for processing accepted goals

        :param goal_handle: The goal handle
        :type goal_handle: rclpy.action.server.ServerGoalHandle

        :return: The result of the goal execution
        :rtype: GripperCommand.Result
        """
        # reset internal data
        self._action_start_time = self._node.get_clock().now().nanoseconds / 1e9
        self._action_result_message = None
        self._action_previous_position_sum = float("inf")
        # set goal
        self._action_goal_handle = goal_handle
        self._action_goal = goal_handle.request
        # wait for the goal to be executed
        while self._action_result_message is None: 
            if self._action_goal is None:
                return GripperCommand.Result()
            time.sleep(self._action_dt)
        self._action_goal = None
        self._action_goal_handle = None
        return self._action_result_message

    def update_step(self, dt: float) -> None:
        """Kit update step

        :param dt: The delta time
        :type dt: float
        """
        pass

    def physics_step(self, dt: float) -> None:
        """Physics update step

        :param dt: The physics delta time
        :type dt: float
        """
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            self._action_dt = dt
            target_position = self._action_goal.command.position
            # set target
            self._dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = False
                self._action_result_message.reached_goal = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_result_message = GripperCommand.Result()
                self._action_result_message.position = position
                self._action_result_message.stalled = True
                self._action_result_message.reached_goal = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.succeed()
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = self._node.get_clock().now().nanoseconds / 1e9 - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_result_message = GripperCommand.Result()
                if self._action_goal_handle is not None:
                    self._action_goal_handle.abort()
                    self._action_goal_handle = None
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)
