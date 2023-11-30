#!/usr/bin/python3
import json
import time

from src.platform.devices.generic.device import Device
from src.platform.robot.generic.robot import Robot
from src.platform.utils import deep_get


class Platform:

    def __init__(self, config_file):
        super().__init__()

        self.config_file = config_file

        self.robot = None
        self.devices = []

        # self.points = []

        self.initialisation()

    def initialisation(self):
        """
        This function is called after loading the config file
        This start the ROS config and store different data
        """

        try:
            with open(self.config_file, 'r') as f:
                data_config = json.load(f)

            config = deep_get(data_config, ['platform', 'robot'], None)
            points = deep_get(data_config, ["project", "points"], [])
            devices = deep_get(data_config, ['platform', 'devices'], [])
            robot_type = config['type']

            for device in devices:
                name = device.get("name", None)
                pin = device.get("pin", None)


                if isinstance(name, str) and isinstance(pin, int):
                    device = Device(name, pin,robot_type)
                    self.devices.append(device)

            self.robot = Robot(config, points)


        except ValueError as e:
            return False, "Error loading config file: {}".format(e)


if __name__ == '__main__':
    print("Storing DATA ")
