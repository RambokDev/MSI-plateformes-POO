#!/usr/bin/python3
import time
import roslaunch

from src.platform.robot.specific.fanuc.robot_fanuc import RobotFanuc
from src.platform.robot.specific.ur.robot_ur import RobotUR


class Robot(RobotUR, RobotFanuc):

    def __init__(self, config: dict):

        self.name = config.get("name", None)
        self.type = config.get("type", None)
        self.ip = config.get("ip", None)
        self.roslaunch_file = config.get("roslaunch_file", None)
        self.connected = False

        if self.type not in ["ur", "fanuc"]:
            raise ValueError("Robot type Unknown.")

        # Provisoire
        if self.type == "fanuc":
            raise NotImplemented

        self.start_ros_config()

        super().__init__()

    def start_ros_config(self):
        """
        This function is called after loading the config file
        This start the ROS config
        """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [self.roslaunch_file, 'robot_ip:={}'.format(self.ip)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(2)
        return True

    def articular_trajectory(self, command):
        if self.type == "ur":
            success, message = RobotUR.articular_trajectory(self, command)
            return success, message

    def connexion(self):
        """This function allowed you to established the robot connexion

        :param state: boolean state, no default value
        :type state: bool

        :return: success, message
        :rtype: bool, str


        """

        if self.type == "ur":

            if not self.connected:
                print("======Start Robot Connexion======")

                success, message = self.set_robot_state(True)
                if success:
                    while True:
                        robot_state = self.get_robot_mode()
                        if robot_state == 7:
                            print("=========Robot mode is {} ======== ".format(robot_state))
                            print("=========Start reverse connexion with the command interface===========")

                            success, message = self.connexion_state(True)
                            if success:
                                print("=========You are actually connected===========")
                                self.connected = not self.connected
                                return success, message
                            else:
                                return success, message
                else:
                    return success, message

            else:
                print("========Start Stoping brakes please wait ======")

                success, message = self.set_robot_state(False)
                if success:
                    while True:
                        robot_state = self.get_robot_mode()
                        if robot_state == 3:
                            print("=========Robot mode is {} ======== ".format(robot_state))
                            print("=========Stop reverse connexion with the command interface===========")
                            success, message = self.connexion_state(False)
                            if success:
                                print("=========You are actually disconnected===========")
                                self.connected = not self.connected
                                return success, message
                            else:
                                return success, message

                else:
                    return success, message


if __name__ == '__main__':
    print("Generic Class Robot ")
