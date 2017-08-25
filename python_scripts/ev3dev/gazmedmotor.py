import rospy
import geometry_msgs.msg
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math


class GazMedMotor:

    def __init__(self):
        self.terminated = False
        self.stopped = False
        self.duty_cycle_sp = None
        self.clear = True
        self.motor_counter = 0

    def talker_abs_pos(self, position_sp, speed):
        self.motor_counter += 1
        current_counter = self.motor_counter

        pub = rospy.Publisher('lego_robot/sonar_wheel_pose', Pose, queue_size=10)

        publishFrequency = 25
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0

        while not rospy.is_shutdown() and not self.stopped:
            if self.motor_counter != current_counter:
                return
            # print(counter, self.motor_counter)
            pose = Pose()
            position = geometry_msgs.msg.Vector3()
            # rotation = geometry_msgs.msg.Quaternion()
            position.x = math.radians(speed)
            # tf.transformations.quaternion_from_euler(0, 0, kwargs['position_sp'])

            counter += 1
            pose.position = position
            orientation = geometry_msgs.msg.Quaternion()
            orientation.z = -math.radians(position_sp)
            pose.orientation = orientation
            # rospy.loginfo(pose)
            pub.publish(pose)
            rate.sleep()

    def activate_thread(self):

        self.stopped = False

    def stop_thread(self):
        # print('STOPPING')
        self.stopped = True
        self.clear = True

    def set_duty_cycle_sp(self, duty_cycle_sp):
        self.duty_cycle_sp = duty_cycle_sp
