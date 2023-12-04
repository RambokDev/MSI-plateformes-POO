#!/usr/bin/python3
import sys
from math import pi
import rospy
import actionlib
from rosservice import rosservice_find
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController, ListControllers
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib import SimpleActionClient
from tf2_msgs.msg import TFMessage

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class RobotUR_ROS(object):
    tool_horizontal_pose_camera = geometry_msgs.Quaternion(0.5, -0.5, -0.5, 0.5)
    tool_down_pose = geometry_msgs.Quaternion(0.9999996829318346, 0.0, 0.0,
                                              0.0007963267107332633)
    tool_horizontal_pose = geometry_msgs.Quaternion(0.5, 0.5, 0.5,
                                                    0.5)
    cartesian_controller = "pose_based_cartesian_traj_controller/follow_cartesian_trajectory"

    def __init__(self, initial_pose=geometry_msgs.Pose(geometry_msgs.Vector3(-0.731, 0.356, 0.357), tool_down_pose)):
        super().__init__()

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        self.load_srv = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)

        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        if not self._search_for_controller("pose_based_cartesian_traj_controller"):
            self._switch_controller("pose_based_cartesian_traj_controller")
        self.trajectory_client = actionlib.SimpleActionClient(self.cartesian_controller,
                                                              FollowCartesianTrajectoryAction)
        self.current_pose = None
        self.initial_pose = initial_pose  # Define an initial position
        rospy.Subscriber("tf", TFMessage, self._update_current_pose)

    def go_to_initial_position(self, duration=2):
        # self.go_to_pose(self.initial_pose, duration)
        success, message = self.go_to_pose(self.initial_pose, duration)
        return success, message

    def go_to_xyz_position(self, x, y, z, duration=2, orientation=None):
        """ Go to the x,y,z position with an orientation Quaternion (default : tool frame pointing down) """
        if orientation is None:
            orientation = self.initial_pose.orientation
        goal_pose = geometry_msgs.Pose(geometry_msgs.Vector3(x, y, z), orientation)
        self.go_to_pose(goal_pose, duration)

    def go_to_pose(self, pose, duration=10):
        """
        Send the robot to this cartesian pose
        pose : geometry_msgs.Pose (position : x,y,z and orientation : quaternion)
        duration : # of seconds for the trajectory
        """
        point = CartesianTrajectoryPoint()
        point.pose = pose
        success, message = self._go_to_this_point(point, duration)
        return success, message

    def go_to_position(self, position, duration=2):
        """
        Send the robot to this cartesian position
        pose : geometry_msgs.Vector3 (position : x,y,z)
        duration : # of seconds for the trajectory
        """
        point = CartesianTrajectoryPoint()
        point.pose.position = position
        point.pose.orientation = self.initial_pose.orientation
        self._go_to_this_point(point, duration)

    def execute_cartesian_trajectory(self, pose_list, duration_list):
        """ Creates a Cartesian trajectory and sends it using the selected action server """
        goal = FollowCartesianTrajectoryGoal()
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        self._execute_trajectory(goal)

    def relative_move(self, x, y, z, orientation=None):
        """ Perform a relative move in all x, y or z coordinates. """
        new_pose = self.get_current_pose()
        if orientation is None:
            new_pose.orientation = self.initial_pose.orientation
        new_pose.position.x += x
        new_pose.position.y += y
        new_pose.position.z += z
        success, message = self.go_to_pose(new_pose)
        return success, message

    def get_current_pose(self):
        """ Return the current pose (translation + quaternion), type = geometry_msgs.Pose """
        return self.current_pose

    ####################### Privates methods #######################

    def _go_to_this_point(self, point, duration=1):
        point.time_from_start = rospy.Duration(duration)
        goal = FollowCartesianTrajectoryGoal()
        goal.trajectory.header.frame_id = "base"
        goal.trajectory.points.append(point)
        # print(goal)
        success, message = self._execute_trajectory(goal)
        return success, message

    def _execute_trajectory(self, goal):

        self.trajectory_client.wait_for_server()
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
        if result.error_code == 0:
            success = True
            message = "Success"
            return success, message
        elif result.error_code == 1:
            success = False
            message = result.error_message
            return success, message

    def _update_current_pose(self, data):
        t = data.transforms[0].transform
        self.current_pose = geometry_msgs.Pose(
            geometry_msgs.Vector3(t.translation.x, t.translation.y, t.translation.z),
            geometry_msgs.Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def _switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
                JOINT_TRAJECTORY_CONTROLLERS
                + CARTESIAN_TRAJECTORY_CONTROLLERS
                + CONFLICTING_CONTROLLERS
        )
        other_controllers.remove(target_controller)
        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def _search_for_controller(self, controller_name):
        controller_managers = rosservice_find('controller_manager_msgs/ListControllers')
        for cm in controller_managers:
            rospy.wait_for_service(cm)
            try:
                list_controllers = rospy.ServiceProxy(cm, ListControllers)
                controller_list = list_controllers()
                for c in controller_list.controller:
                    if c.name == controller_name:
                        return True
            except rospy.ServiceException:
                rospy.loginfo("Service call failed ")
        return False

    def _execute_trajectory_joint(self, goal):

        self.trajectory_client_joint.wait_for_server()
        self.trajectory_client_joint.send_goal(goal)
        self.trajectory_client_joint.wait_for_result()
        result = self.trajectory_client_joint.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def convert_deg_to_rad(self, tab):
        res = []
        for i in tab:
            res.append(i * pi / 180)
        return res

    def switch_controler_robot(self, target):
        self._switch_controller(target)



    def send_joint_trajectory(self, position):
        action_client = SimpleActionClient("/pos_joint_traj_controller/follow_joint_trajectory",
                                           FollowJointTrajectoryAction)
        action_client.wait_for_server()
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                       "wrist_3_joint"]
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = joint_names
        positions = [
            position,  # Joint positions for the first point
        ]
        print(positions)
        time_from_start = [rospy.Duration(5.0), rospy.Duration(5.0)]  # Adjust timing as needed
        for i in range(len(positions)):
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = positions[i]
            trajectory_point.time_from_start = time_from_start[i]
            joint_trajectory.points.append(trajectory_point)
        action_goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)
        action_client.send_goal(action_goal)
        result = action_client.wait_for_result()
        if result:
            success = True
            message = "Success"
            return success, message
        else:
            success = False
            message = result.error_message
            return success, message


if __name__ == '__main__':
    import time

    rospy.init_node("test_robotUR")
    myRobot = RobotUR_ROS()
    input("============ Press `Enter` to go to initial position ...")
    myRobot.go_to_initial_position(5)

    # # this okey for center point
    # input("============ Press `Enter` to go to test charles...")
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(-0.6179846576602167, 0.000418113766730972, -0.05194822471506214), RobotUR.tool_down_pose
    # ))
    # input("============ Press `Enter` to execute a relative_move ...")
    # myRobot.relative_move(0.009, 0.006, -0.40)

    # this okey for right top point
    input("============ Press `Enter` to go to test charles...")
    myRobot.go_to_pose(geometry_msgs.Pose(
        geometry_msgs.Vector3(-0.679958428096283, 0.10701808485686919, -0.05158763753667969), RobotUR.tool_down_pose
    ))
    input("============ Press `Enter` to execute a relative_move ...")
    myRobot.relative_move(-0.01229, 0.04629, -0.40)

    # input("============ Press `Enter` to go to test charles 2...")
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(0.009850757925073026, 0.006215707372015888, -0.4000413270592923), RobotUR.tool_down_pose
    # ))
    #
    #
    # input("============ Press `Enter` to go to test charles...")
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(0.619, 0.000, -0.38), RobotUR.tool_down_pose
    # ))
    #
    # input("============ 1 Press `Enter` to go to a x,y,z  position ...")
    # myRobot.go_to_xyz_position(0.35, 0, 0.12)
    # input("============ HORIZONTAL Press `Enter` to go to a x,y,z  position ...")
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(0.442, -0.022, 0.125), RobotUR.tool_horizontal_pose
    # ), 9)
    # input("============ VERTICAL Press `Enter` to go to a x,y,z  position ...")
    # myRobot.go_to_initial_position(10)
    #
    # input("============ Press `Enter` to go to 2 different posesche ...")
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(0.275, 0, 0.3), RobotUR.tool_horizontal_pose
    # ), 1)
    # myRobot.go_to_pose(geometry_msgs.Pose(
    #     geometry_msgs.Vector3(0.3, -0.13, 0.0238), RobotUR.tool_down_pose
    # ))
    # print("Current pose : {}".format(myRobot.get_current_pose()))
    # input("============ Press `Enter` to execute a cartesian trajectory ...")
    # pose_list = [
    #     geometry_msgs.Pose(
    #         geometry_msgs.Vector3(0.3, -0.13, 0.238), RobotUR.tool_down_pose
    #     ),
    #     geometry_msgs.Pose(
    #         geometry_msgs.Vector3(0.4, -0.1, 0.2), RobotUR.tool_down_pose
    #     ),
    #     geometry_msgs.Pose(
    #         geometry_msgs.Vector3(0.4, 0.3, 0.2), RobotUR.tool_down_pose
    #     ),
    #     geometry_msgs.Pose(
    #         geometry_msgs.Vector3(0.3, 0.3, 0.2), RobotUR.tool_down_pose
    #     ),
    #     geometry_msgs.Pose(
    #         geometry_msgs.Vector3(0.3, -0.13, 0.238), RobotUR.tool_down_pose
    #     ),
    # ]
    # duration_list = [1.0, 3.0, 4.0, 5.0, 6.0]
    # myRobot.execute_cartesian_trajectory(pose_list, duration_list)
    # input("============ Press `Enter` to execute a relative_move ...")
    # myRobot.relative_move(0.02, 0.04, 0.06)
    # print("The end")
