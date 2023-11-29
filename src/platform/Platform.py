#!/usr/bin/python3
import json
import time

from src.platform.robot.generic.Robot import Robot


class Platform(Robot):

    def __init__(self):
        super(Platform, self).__init__()
        self.robot = Robot()
        self.Robot_params = None
        self.config_filename = None

    def initialisation(self, config_file):
        """
        This function is called after loading the config file
        This start the ROS config and store different data
        """
        self.config_filename = config_file
        print("initialisation", config_file)

        config_file = open(config_file, 'r')

        try:
            data_config = json.load(config_file)
            success = self.robot.start_ros_config(data_config)
            if success:
                robot_params = data_config['project']
                self.Robot_params = robot_params
                print(self.Robot_params)
            config_file.close()
        except ValueError as e:
            return False, "Error loading config file: {}".format(e)

    def connexion(self):
        print("connexion")


if __name__ == '__main__':
    print("Storing DATA ")
