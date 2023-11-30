#!/usr/bin/python3
import rospy
from ur_msgs.srv import SetIO



class DeviceUR:

    def __init__(self):
        super().__init__()


    def set_pin_state_ur(self,pin: int, state: bool):
        set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_interface(1, pin, state)


    def impulse_pin_state_ur(self,pin: int, time: float):
        print("hard")
        set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        set_io_interface(1, pin, 1)
        rospy.sleep(time)
        set_io_interface(1, pin, 0)


