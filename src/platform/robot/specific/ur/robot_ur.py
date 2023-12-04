#!/usr/bin/python3
import threading
import time
import geometry_msgs.msg as geometry_msgs
import rospy
from ur_dashboard_msgs.srv import GetRobotMode
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction, SetSpeedSliderFractionRequest, SetSpeedSliderFractionResponse
from src.platform.robot.specific.ur.commands.compute_trajectory import compute_trajectory, formatting_commands
from src.platform.robot.specific.ur.commands.robot_ur_ros import RobotUR_ROS
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState


class RobotUR(RobotUR_ROS):

    def __init__(self):
        super().__init__()

    def connexion_state(self, state: bool):
        """
        This function allow you to connect or disconnect the robot
        Example : True the robot is connected
        @param: state       a bool
        """
        if state:
            service = "play"
        else:
            service = "stop"

        rospy.wait_for_service('/ur_hardware_interface/dashboard/{}'.format(service))
        robot_connexion_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/{}'.format(service), Trigger)

        try:
            resp = robot_connexion_state()
            if resp.success:
                time.sleep(5)
                rospy.init_node("robotUR", disable_signals=True)
            return resp.success, resp.message
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False, "Error"

    def go_to(self, trajectories):

        if trajectories.kind == "cartesian":
            success, message = self.cartesian_trajectory(trajectories.coord, trajectories.move,
                                                         trajectories.tool_position)
            return success, message
        elif trajectories.kind == "articular":
            success, message = self.articular_trajectory(trajectories.coord)
            return success, message
        else:
            raise ValueError("Unknown Trajectory.")

    def execute_recipe(self, params, args):

        if params.name == "pickup":
            trajectories = params.trajectories
            devices = params.devices
            trajectory_camera = next((x for x in trajectories if x.name == "camera_angle"), None)
            device_venturi = next((x for x in devices if x.name == "venturi"), None)
            device_contact_sensor = next((x for x in devices if x.name == "contact_sensor"), None)

            contact_sensor = self.contact_sensor(trajectory_camera, device_contact_sensor)

            Xreal, Yreal = args[0]
            robot_command, vector = self.compute_traj(Xreal, Yreal)
            print(robot_command, vector)
            for trajectory in trajectories:
                print(trajectory.name)
                if trajectory.name == "top_bin":
                    trajectory.coord = robot_command
                elif trajectory.name == "down_bin":
                    device_venturi.toggle_pin_state()
                    trajectory.coord = vector
                print(trajectory.coord)
                success, message = self.go_to(trajectory)
                if contact_sensor == 1:
                    return True, "Go to Camera"
                if not success:
                    return success, message

            return True, "ok"

    def compute_traj(self, Xreal, Yreal):
        start_pt, vect = compute_trajectory(Xreal, Yreal, 40)
        robot_command, vector = formatting_commands(start_pt, vect)
        return robot_command, vector

    def thread_function_contact_sensor(self, trajectory, device, sensor_state):
        print("Thread contact_sensor is running")
        sensor_state = 0
        while True:
            print(device.name)
            msg = device.get_state_information()
            if msg is True:
                self.go_to(trajectory)
                print(msg)
                sensor_state = 1
                break

        return sensor_state

    def contact_sensor(self, trajectory, device):
        """
        This function is main test function
        """
        sensor_state = 0
        contact_sensor_thread = threading.Thread(target=self.thread_function_contact_sensor,
                                                 args=(trajectory, device, sensor_state))
        contact_sensor_thread.start()
        return sensor_state

    def cartesian_trajectory(self, command, move, tool_position):
        self.switch_controler_robot("pose_based_cartesian_traj_controller")
        if command == 'initial_position':
            duration = 5
            success, message = self.go_to_initial_position(duration)
            return success, message
        elif type(command) == list:
            if move == 'relative':
                success, message = self.relative_move(command[0], command[1], command[2])
                return success, message
            else:
                if tool_position == 'horizontal':
                    success, message = self.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        self.tool_horizontal_pose_camera
                    ))
                    return success, message
                elif tool_position == 'down':
                    success, message = self.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        self.tool_down_pose
                    ))
                    return success, message
                else:
                    success, message = self.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        tool_position
                    ))
                    return success, message

    def articular_trajectory(self, command):
        print(type(command))
        if type(command) == list:
            self.switch_controler_robot("pos_joint_traj_controller")
            success, message = self.send_joint_trajectory(self.convert_deg_to_rad(command))
            return success, message

    def robot_get_info(self, info_type, robot):
        if info_type == "current_pose":
            data = robot.get_current_pose()
            return data

    def robot_create_quaternions(self, pose):
        print(pose)
        data = geometry_msgs.Quaternion(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w)
        print(data)
        return data

    def get_robot_mode(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode')
        robot_get_mode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode',
                                            GetRobotMode)
        try:
            resp = robot_get_mode()
            if resp.success:
                return resp.robot_mode.mode
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    def set_robot_state(self, state: bool):
        """
        This function allowed you to control the robot state ( ON , OFF )
        Example : True      in order to power_on the robot controller and brakes
        Example : False     in order to power_off the robot controller
        @param: state       a Boolean
        """
        print(state)
        if state:
            robot_state_str = 'brake_release'
        elif not state:
            robot_state_str = 'power_off'

        rospy.wait_for_service('/ur_hardware_interface/dashboard/{}'.format(robot_state_str))
        robot_change_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/{}'.format(robot_state_str), Trigger)

        try:
            resp = robot_change_state()
            if resp.success:
                time.sleep(5)
            return resp.success, resp.message
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False, "Error"

    def set_speed(self, speed: int):
        """
        Change de speed slider of the ur Dashboard
        Example : 0.1
        @param: speed       An int that represent the speed in porcent
        """
        rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
        robot_speed_state = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)

        try:
            resp = robot_speed_state(speed)
            return resp.success
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False

    def get_joint_positions(self):
        try:
            # Wait for a message on the specified topic
            msg = rospy.wait_for_message("/joint_states", JointState,
                                         timeout=5.0)  # Adjust timeout as needed

            # Access joint positions from the received message
            joint_positions = msg.position
            # base = 2
            # Epaule = 1
            # coude = 0
            # P1 = 3
            # P2 = 4
            # P3 = 5
            return joint_positions

        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get joint positions from topic /ur_hardware_interface/get_joint_states: {e}")
            return None


if __name__ == '__main__':
    print("Storing DATA ")
