#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates

#gazebo_msgs/LinkStates
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.twist[2])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/gazebo/link_states', LinkStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
