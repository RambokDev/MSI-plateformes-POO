#!/usr/bin/python3
import time
import roslaunch
from src.platform.robot.specific.ur.RobotUR import RobotUR


class Robot(RobotUR):

    def __init__(self):
        super().__init__()
        self.robot_ur = RobotUR
        self.robot_interface = None

    def trajectory(self):
        print("traj")

    def robot_connexion(self, state: bool):
        """This function allowed you to established the robot connexion

        :param state: boolean state, no default value
        :type state: bool

        :return: success, message
        :rtype: bool, str


        """
        if state:
            print("======Start Robot Connexion======")
            success, message = self.robot_ur.set_robot_state(True)
            if success:
                while True:
                    robot_state = self.robot_ur.get_robot_mode()
                    if robot_state == 7:
                        print("=========Robot mode is {} ======== ".format(robot_state))
                        print("=========Start reverse connexion with the command interface===========")

                        success, message, robot = self.robot_ur.connexion_state(True)
                        if success:
                            print("=========You are actually connected===========")
                            self.robot_interface = robot
                            return success, message, robot
                        else:
                            return success, message, robot
            else:
                return success, message, None
        else:
            print("========Start Stoping brakes please wait ======")
            success, message = self.robot_ur.set_robot_state(False)
            if success:
                while True:
                    robot_state = self.robot_ur.get_robot_mode()
                    if robot_state == 3:
                        print("=========Robot mode is {} ======== ".format(robot_state))
                        print("=========Stop reverse connexion with the command interface===========")
                        success, message, robot = self.robot_ur.connexion_state(False)
                        if success:
                            print("=========You are actually disconnected===========")
                            self.robot_interface = None
                            return success, message
                        else:
                            return success, message

            else:
                return success, message

    def start_ros_config(self, data_config):
        """
        This function is called after loading the config file
        This start the ROS config
        """
        print(data_config)
        robot_ip = data_config['robot_ip']
        launch_file_path = data_config['launch_file_path']
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file_path, 'robot_ip:={}'.format(robot_ip)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(2)
        return True


if __name__ == '__main__':
    print("Generic Class Robot ")
