#!/usr/bin/python3
import json
import time

from src.platform.devices.generic.device import Device
from src.platform.robot.generic.robot import Robot
from src.platform.robot.specific.ur.commands.trajectory import Trajectory
from src.platform.utils import deep_get


class Platform:

    def __init__(self, config_file):
        super().__init__()
        self.config_file = config_file
        self.robot = None
        self.devices = []
        self.trajectories = []
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
            trajectories = deep_get(data_config, ["project", "trajectories"], [])
            devices = deep_get(data_config, ['platform', 'devices'], [])

            robot_type = config['type']

            for device in devices:
                name = device.get("name", None)
                pin = device.get("pin", None)
                if isinstance(name, str) and isinstance(pin, int):
                    device = Device(name, pin, robot_type)
                    self.devices.append(device)

            self.robot = Robot(config)

            for trajectory in trajectories:
                name = trajectory.get("name", None)
                data = trajectory.get("data", None)
                trajectory_type = data.get("trajectory_type", None)
                coord = data.get("coord", None)
                tool_position = data.get("tool_position", "down")
                move = data.get('move', None)
                if isinstance(name, str) and isinstance(trajectory_type, str) and isinstance(coord, list):
                    trajectory = Trajectory(name, trajectory_type, coord, move, tool_position)
                    self.trajectories.append(trajectory)




        except ValueError as e:
            return False, "Error loading config file: {}".format(e)


if __name__ == '__main__':
    print("Storing DATA ")
