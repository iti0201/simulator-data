import rospy
import time
import math
from std_msgs.msg import String
from sensor_msgs.msg import Range


class GazSonar:
    def __init__(self):
        self.sonar_data = None
        self.times_listened = 0
        self.sub = rospy.Subscriber('lego_robot/sonar', Range, self.callback)

    # gazebo_msgs/Image
    def callback(self, data):
        if self.times_listened >= 1:
            self.sonar_data = data
        self.times_listened += 1  # for some reason the first set of data always gives old data

    def get_distance(self):
        if self.sonar_data is not None:
            return self.sonar_data.range
        else:
            return 2.55 # the problem here is that if you ask the distance immediately after initialising the sensor, it won't have enough time to fetch the data
