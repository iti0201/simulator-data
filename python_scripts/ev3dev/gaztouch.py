from __future__ import division

import numpy


import rospy
import time
import math
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState


class GazTouch:

    STICK_LENGTH = 0.08
    HOLDER_LENGTH = 0.04
    ACTIVATION_LIMIT = 0.01

    def __init__(self):
        self.touch_data = None
        self.is_pressed = 0
        self.times_listened = 0
        self.sub = rospy.Subscriber('lego_robot/touch_joint_states', JointState, self.callback)

    def callback(self, data):
        if self.times_listened >= 1:
            self.touch_data = data
            self.is_pressed = self.get_button_value()
        self.times_listened += 1  # for some reason the first set of data always gives old data

    def get_button_value(self):
        if self.touch_data is not None:
            robot_pose = self.touch_data.position # this gets the robot's orientation
            stick_x = robot_pose[0]
            stick_y = robot_pose[1]
            holder_x = robot_pose[2]
            holder_y = robot_pose[3]
            distance = math.sqrt(math.pow(stick_x - holder_x, 2) + math.pow(stick_y - holder_y, 2))
            # print("distance is: ", distance)

            if distance < self.STICK_LENGTH / 2 + self.HOLDER_LENGTH / 2 - self.ACTIVATION_LIMIT:
                return 1
            else:
                return 0

        else:
            return 0 # the problem here is that if you ask the yaw immediately after initialising the sensor, it won't have enough time to fetch the data

    def button_value(self):
        return self.is_pressed

