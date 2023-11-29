#!/usr/bin/python3
from platform.robot.specific.ur.commands.RobotUR_ROS import RobotUR_ROS
import geometry_msgs.msg as geometry_msgs


class RobotUR(object):

    def __init__(self):
        super().__init__()
        self.Robot = RobotUR_ROS()

    def cartesian_trajectory(self, robot, command, move, tool_position):
        robot.switch_controler_robot("pose_based_cartesian_traj_controller")

        if command == 'initial_position':
            duration = 5
            success, message = robot.go_to_initial_position(duration)
            return success, message
        elif type(command) == list:
            if move == 'relative':
                success, message = robot.relative_move(command[0], command[1], command[2])
                return success, message
            else:
                if tool_position == 'horizontal':
                    success, message = robot.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        robot.tool_horizontal_pose_camera
                    ))
                    return success, message
                elif tool_position == 'down':
                    success, message = robot.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        robot.tool_down_pose
                    ))
                    return success, message
                else:
                    success, message = robot.go_to_pose(geometry_msgs.Pose(
                        geometry_msgs.Vector3(command[0], command[1], command[2]),
                        tool_position
                    ))
                    return success, message

    def articular_trajectory(robot, command):
        if type(command) == list:
            robot.switch_controler_robot("pos_joint_traj_controller")
            success, message = robot.send_joint_trajectory(robot.convert_deg_to_rad(command))
            return success, message

    def robot_get_info(info_type, robot):
        if info_type == "current_pose":
            data = robot.get_current_pose()
            return data

    def robot_create_quaternions(pose):
        print(pose)
        data = geometry_msgs.Quaternion(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w)
        print(data)
        return data


if __name__ == '__main__':
    print("Storing DATA ")
