try:
    import omni.kit.test
    TestCase = omni.kit.test.AsyncTestCaseFailOnLogError
except:
    class TestCase:
        pass

import json
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand

import add_on_msgs.srv


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestROS2Bridge(TestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    # Actual test, notice it is "async" function, so "await" can be used if needed
    async def test_ros2_bridge(self):
        pass


class TestROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('test_ros2_bridge')

        self.get_attribute_service_name = '/get_attribute'
        self.set_attribute_service_name = '/set_attribute'
        self.gripper_command_action_name = "/panda_hand_controller/gripper_command"
        self.follow_joint_trajectory_action_name = "/panda_arm_controller/follow_joint_trajectory"

        self.get_attribute_client = self.create_client(add_on_msgs.srv.GetPrimAttribute, self.get_attribute_service_name)
        self.set_attribute_client = self.create_client(add_on_msgs.srv.SetPrimAttribute, self.set_attribute_service_name)
        self.gripper_command_client = ActionClient(self, GripperCommand, self.gripper_command_action_name)
        self.follow_joint_trajectory_client = ActionClient(self, FollowJointTrajectory, self.follow_joint_trajectory_action_name)

        self.follow_joint_trajectory_goal_msg = FollowJointTrajectory.Goal()
        self.follow_joint_trajectory_goal_msg.path_tolerance = []
        self.follow_joint_trajectory_goal_msg.goal_tolerance = []
        self.follow_joint_trajectory_goal_msg.goal_time_tolerance = Duration().to_msg()
        self.follow_joint_trajectory_goal_msg.trajectory.header.frame_id = "panda_link0"
        self.follow_joint_trajectory_goal_msg.trajectory.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        self.follow_joint_trajectory_goal_msg.trajectory.points = [
            JointTrajectoryPoint(positions=[0.012, -0.5689, 0.0, -2.8123, 0.0, 3.0367, 0.741], time_from_start=Duration(seconds=0, nanoseconds=0).to_msg()),
            JointTrajectoryPoint(positions=[0.011073551914608105, -0.5251352171920526, 6.967729509163362e-06, -2.698296723677182, 7.613460540484924e-06, 2.93314462685839, 0.6840062390114862], time_from_start=Duration(seconds=0, nanoseconds= 524152995).to_msg()),
            JointTrajectoryPoint(positions=[0.010147103829216212, -0.48137043438410526, 1.3935459018326723e-05, -2.5842934473543644, 1.5226921080969849e-05, 2.82958925371678, 0.6270124780229722], time_from_start=Duration(seconds=1, nanoseconds=  48305989).to_msg()),
            JointTrajectoryPoint(positions=[0.009220655743824318, -0.43760565157615794, 2.0903188527490087e-05, -2.4702901710315466, 2.2840381621454772e-05, 2.72603388057517, 0.5700187170344584], time_from_start=Duration(seconds=1, nanoseconds= 572458984).to_msg()),
            JointTrajectoryPoint(positions=[0.008294207658432425, -0.39384086876821056, 2.7870918036653447e-05, -2.3562868947087283, 3.0453842161939697e-05, 2.6224785074335597, 0.5130249560459446], time_from_start=Duration(seconds=2, nanoseconds=  96611978).to_msg()),
            JointTrajectoryPoint(positions=[0.00736775957304053, -0.3500760859602632, 3.483864754581681e-05, -2.2422836183859105, 3.806730270242462e-05, 2.518923134291949, 0.45603119505743067], time_from_start=Duration(seconds=2, nanoseconds= 620764973).to_msg()),
            JointTrajectoryPoint(positions=[0.006441311487648636, -0.30631130315231586, 4.1806377054980174e-05, -2.1282803420630927, 4.5680763242909544e-05, 2.415367761150339, 0.3990374340689168], time_from_start=Duration(seconds=3, nanoseconds= 144917968).to_msg()),
            JointTrajectoryPoint(positions=[0.005514863402256743, -0.2625465203443685, 4.877410656414353e-05, -2.014277065740275, 5.3294223783394466e-05, 2.311812388008729, 0.34204367308040295], time_from_start=Duration(seconds=3, nanoseconds= 669070962).to_msg()),
            JointTrajectoryPoint(positions=[0.004588415316864848, -0.2187817375364211, 5.5741836073306894e-05, -1.900273789417457, 6.0907684323879394e-05, 2.208257014867119, 0.28504991209188907], time_from_start=Duration(seconds=4, nanoseconds= 193223957).to_msg()),
        ]

        self.gripper_command_open_goal_msg = GripperCommand.Goal()
        self.gripper_command_open_goal_msg.command.position = 0.03990753115697298
        self.gripper_command_open_goal_msg.command.max_effort = 0.0

        self.gripper_command_close_goal_msg = GripperCommand.Goal()
        self.gripper_command_close_goal_msg.command.position = 8.962388141080737e-05
        self.gripper_command_close_goal_msg.command.max_effort = 0.0


if __name__ == '__main__':

    rclpy.init()

    node = TestROS2BridgeNode()

    # ==== Gripper Command ====
    assert node.gripper_command_client.wait_for_server(timeout_sec=1.0), \
        "Action server {} not available".format(node.gripper_command_action_name)
    
    # close gripper command (with obstacle)
    future = node.gripper_command_client.send_goal_async(node.gripper_command_close_goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.stalled is True
    assert result.result.reached_goal is False
    assert abs(result.result.position - 0.0295) < 1e-3

    # open gripper command
    future = node.gripper_command_client.send_goal_async(node.gripper_command_open_goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.stalled is False
    assert result.result.reached_goal is True
    assert abs(result.result.position - 0.0389) < 1e-3

    # ==== Attribute ====
    assert node.get_attribute_client.wait_for_service(timeout_sec=5.0), \
        "Service {} not available".format(node.get_attribute_service_name)
    assert node.set_attribute_client.wait_for_service(timeout_sec=5.0), \
        "Service {} not available".format(node.set_attribute_service_name)
    
    request_get_attribute = add_on_msgs.srv.GetPrimAttribute.Request()
    request_get_attribute.path = "/Cylinder"
    request_get_attribute.attribute = "physics:collisionEnabled"
    
    request_set_attribute = add_on_msgs.srv.SetPrimAttribute.Request()
    request_set_attribute.path = request_get_attribute.path
    request_set_attribute.attribute = request_get_attribute.attribute
    request_set_attribute.value = json.dumps(False)

    # get obstacle collisionEnabled attribute
    future = node.get_attribute_client.call_async(request_get_attribute)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    
    assert result.success is True
    assert json.loads(result.value) is True

    # disable obstacle collision shape
    future = node.set_attribute_client.call_async(request_set_attribute)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()
    
    assert result.success is True

    # get obstacle collisionEnabled attribute
    future = node.get_attribute_client.call_async(request_get_attribute)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()

    assert result.success is True
    assert json.loads(result.value) is False

    # ==== Gripper Command ====
    assert node.gripper_command_client.wait_for_server(timeout_sec=1.0), \
        "Action server {} not available".format(node.gripper_command_action_name)

    # close gripper command (without obstacle)
    future = node.gripper_command_client.send_goal_async(node.gripper_command_close_goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    result = future.result()

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.stalled is False
    assert result.result.reached_goal is True
    assert abs(result.result.position - 0.0) < 1e-3

    # ==== Follow Joint Trajectory ====
    assert node.follow_joint_trajectory_client.wait_for_server(timeout_sec=1.0), \
        "Action server {} not available".format(node.follow_joint_trajectory_action_name)

    # move to goal
    future = node.follow_joint_trajectory_client.send_goal_async(node.follow_joint_trajectory_goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()

    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.error_code == result.result.SUCCESSFUL

    print("Test passed")
