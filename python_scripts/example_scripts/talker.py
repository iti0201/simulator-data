#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import math

def talker():
    pub = rospy.Publisher('lego_robot/cmd_vel_left', Twist, queue_size=10)
    pub2 = rospy.Publisher('lego_robot/cmd_vel_right', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    counter = 0
    while not rospy.is_shutdown():
        twist = Twist()
        angular = geometry_msgs.msg.Vector3()
        linear = geometry_msgs.msg.Vector3()
        if counter > 4:
            linear.x = 0  # m/s
            angular.x = 0
        else:
            linear.x = 0 # m/s
            angular.x = 500 * math.pi / 180
        twist.angular = angular
        twist.linear = linear

        counter += 1
        rospy.loginfo(twist)
        pub.publish(twist)
        pub2.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
