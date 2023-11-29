#!/usr/bin/python3
from platform.robot.specific.ur.RobotUR import RobotUR


class Robot(RobotUR):

    def __init__(self):
        super().__init__()
        self.Robot_interface = RobotUR()



    def robot_connexion(state: bool):
        """This function allowed you to established the robot connexion

        :param state: boolean state, no default value
        :type state: bool

        :return: success, message
        :rtype: bool, str


        """
        if state:
            print("======Start Robot Connexion======")
            success, message = set_robot_state(True)
            if success:
                while True:
                    robot_state = get_robot_mode()
                    if robot_state == 7:
                        print("=========Robot mode is {} ======== ".format(robot_state))
                        print("=========Start reverse connexion with the command interface===========")

                        success, message, robot = connexion_state(True)
                        if success:
                            print("=========You are actually connected===========")

                            Platform().storeInterface(robot)
                            return success, message, robot
                        else:
                            return success, message, robot
            else:
                return success, message, None
        else:
            print("========Start Stoping brakes please wait ======")
            success, message = set_robot_state(False)
            if success:
                while True:
                    robot_state = get_robot_mode()
                    if robot_state == 3:
                        print("=========Robot mode is {} ======== ".format(robot_state))
                        print("=========Stop reverse connexion with the command interface===========")
                        success, message, robot = connexion_state(False)
                        if success:
                            print("=========You are actually disconnected===========")

                            return success, message
                        else:
                            return success, message

            else:
                return success, message


    def _start_ros_config(self, data_config):
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
