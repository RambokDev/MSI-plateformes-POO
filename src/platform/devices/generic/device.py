#!/usr/bin/python3
from src.platform.devices.specific.ur.device_ur import DeviceUR


class Device(DeviceUR):
    def __init__(self, name, pin, robot_type):
        super().__init__()
        self.name = name
        self.pin = pin
        self.robot_type = robot_type
        self.state = False

    def toggle_pin_state(self):
        if self.robot_type == 'ur':
            self.state = not self.state
            self.set_pin_state_ur(self.pin, self.state)
        elif self.robot_type == 'fanuc':
            raise NotImplemented

    def inpulse_pin_state(self, duration):
        if self.robot_type == 'ur':
            self.impulse_pin_state_ur(self.pin, duration)
        elif self.robot_type == 'fanuc':
            raise NotImplemented

    def get_state_information(self):
        """
        This function is create in order to get the contact
        """
        if self.robot_type == "ur":
            self.get_state_information_ur(self.pin)
        elif self.robot_type == 'fanuc':
            raise NotImplemented

