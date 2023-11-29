



def go_pick_and_place(self, robot_command, vector):
    """
    This function is used for the pick and place action
    :param robot_command: the robot command
    :param vector: the vector to move the robot to pick and place pump
    """
    self.sensor_contact = sensor_loop()
    success, message = robot_trajectory("cartesian", self.myRobot, "initial_position", None, "down")
    if success:
        robot_trajectory("cartesian", self.myRobot, robot_command, None, "down")
        if self.sensor_contact != 1:
            set_pin_state("ur", 0, True)
            robot_trajectory("cartesian", self.myRobot, vector, "relative", "down")
            success, message = self.go_to_camera()
            return success, message
    else:
        return success, message

def go_to_camera(self):
    """
    This function allowed you to go to the position of the angle camera
    """
    camera_command = [-0.883, 0.775, 0.594]
    success, message = robot_trajectory("cartesian", self.myRobot, camera_command, None, "horizontal")
    return success, message





def go_to_position(robot_command):
    print(robot_command)
    myRobot = RobotUR()
    # myRobot.go_to_initial_position(5)

    myRobot.go_to_pose(geometry_msgs.Pose(
        geometry_msgs.Vector3(robot_command[0], robot_command[1], robot_command[2]),
        RobotUR.tool_down_pose
    ))


def thread_function1(command1, sensor_state):
    print("Thread 1 is running")
    print("This thread will wait for the robot to detect any changement in trajectory")
    sensor_state = 0
    while True:
        msg = contact_sensor()
        # print(msg)
        if msg is True:
            go_to_position(command1)
            print(msg)
            sensor_state = 1
            break

    return sensor_state


def thread_function2(command2):
    print("Thread 2 is running")
    go_to_position(command2)


def sensor_loop():
    """
    This function is main test function
    """
    # rospy.init_node("test_robot")

    command1 = [-0.736, 0.356, 0.222]
    # command2 = [-0.736, 0.100, 0.222]
    sensor_state = 0
    t1 = threading.Thread(target=thread_function1, args=(command1, sensor_state))
    # t2 = threading.Thread(target=thread_function2, args=(command2,))

    t1.start()
    # t2.start()

    # t1.join()
    return sensor_state
    # t2.join()