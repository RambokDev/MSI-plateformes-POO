#!/usr/bin/python3
import rospy
from ur_msgs.srv import SetIO
import time
from ur_msgs.msg import IOStates


class DeviceUR:

    def __init__(self):
        super().__init__()

    def set_pin_state_ur(self, pin: int, state: bool):
        set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_interface(1, pin, state)

    def impulse_pin_state_ur(self, pin: int, time: float):
        print("hard")
        set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_interface(1, pin, True)
        rospy.sleep(time)
        set_io_interface(1, pin, False)

    def get_state_information_ur(self, pin: int):
        """
        This function is create in order to get the contact
        """
        msg = rospy.wait_for_message('ur_hardware_interface/io_states', IOStates)
        return msg.digital_in_states[pin].state
