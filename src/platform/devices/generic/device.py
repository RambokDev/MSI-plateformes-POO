#!/usr/bin/python3
from src.platform.devices.specific.ur.device_ur import DeviceUR


class Device(DeviceUR):
    def __init__(self, name, pin,robot_type):
        super().__init__()
        self.name = name
        self.pin = pin
        self.robot_type = robot_type

    def set_pin_state(self, pin, state):
        if self.robot_type == 'ur':
            self.set_pin_state_ur(pin, state)
        elif self.robot_type == 'fanuc':
            raise NotImplemented

    def inpulse_pin_state(self, duration):
        if self.robot_type == 'ur':
            self.impulse_pin_state_ur(self.pin, duration)
        elif self.robot_type == 'fanuc':
            raise NotImplemented
