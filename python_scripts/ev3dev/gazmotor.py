import rospy
import geometry_msgs.msg
import time
from geometry_msgs.msg import Twist
import math


class GazMotor:

    def __init__(self, wheel):
        self.wheel = wheel
        self.terminated = False
        self.stopped = False
        self.duty_cycle_sp = None
        self.clear = True
        self.motor_counter = 0

    def talker_timed(self, kwargs):
        self.motor_counter += 1
        current_counter = self.motor_counter
        pub = rospy.Publisher('lego_robot/cmd_vel_' + self.wheel, Twist, queue_size=10)

        publishFrequency = 60
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0

        t_end = time.time() + kwargs['time_sp'] / 1000

        while not rospy.is_shutdown() and not self.stopped and time.time() < t_end:
            if self.motor_counter != current_counter:
                return
            # print(counter)
            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()

            angular.x = math.radians(kwargs['speed_sp'])
            # angular.x = kwargs['speed_sp'] * math.pi / 180

            counter += 1

            twist.angular = angular
            twist.linear = linear

            # rospy.loginfo(twist)
            pub.publish(twist)

            rate.sleep()

        self.coast(pub, math.radians(kwargs['speed_sp']))

    def talker_direct(self):
        self.motor_counter += 1
        current_counter = self.motor_counter
        pub = rospy.Publisher('lego_robot/cmd_vel_' + self.wheel, Twist, queue_size=10)

        max_speed = 800
        # running the robot at duty_cycle_sp of 100 gave me a speed of 880 something in the air
        # running on the ground it got like 800

        publishFrequency = 60
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0

        while not rospy.is_shutdown() and not self.stopped:
            if self.motor_counter != current_counter:
                return
            # print(counter)
            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()

            angular.x = math.radians(max_speed * self.duty_cycle_sp / 100)
            # angular.x = kwargs['speed_sp'] * math.pi / 180

            counter += 1

            twist.angular = angular
            twist.linear = linear

            # rospy.loginfo(twist)
            pub.publish(twist)

            rate.sleep()

        self.coast(pub, math.radians(max_speed * self.duty_cycle_sp / 100))

    def talker_forever(self, kwargs):
        self.motor_counter += 1
        current_counter = self.motor_counter

        pub = rospy.Publisher('lego_robot/cmd_vel_' + self.wheel, Twist, queue_size=10)

        publishFrequency = 60
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0

        while not rospy.is_shutdown() and not self.stopped:
            if self.motor_counter != current_counter:
                return
            # print(counter, self.wheel, self.motor_counter)
            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()
            angular.x = math.radians(kwargs['speed_sp'])
            counter += 1
            twist.angular = angular
            twist.linear = linear
            # rospy.loginfo(twist)
            pub.publish(twist)
            rate.sleep()

        self.coast(pub, math.radians(kwargs['speed_sp']))

    def coast(self, pub, currentSpeed):
        '''
        The idea of this function is to simulate coasting.
        I've set it so that on coasting, the wheel slows down 0.5 radians ever 0.1 seconds.
        So that's 5 radians per second.
        '''
        coastStep = 0.5
        totalSteps = int(math.floor(math.fabs(currentSpeed) / coastStep))
        current_counter = self.motor_counter
        for i in range(0, totalSteps):
            if self.motor_counter != current_counter:
                return
            if i == totalSteps - 1:
                currentSpeed = 0
            else:
                if currentSpeed > 0:
                    currentSpeed -= coastStep
                else:
                    currentSpeed += coastStep

            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()

            angular.x = currentSpeed

            twist.angular = angular
            twist.linear = linear

            # rospy.loginfo(twist)
            pub.publish(twist)
            time.sleep(0.1)

    def activate_thread(self):

        self.stopped = False

    def stop_thread(self):
        # print('STOPPING')
        self.stopped = True
        self.clear = True

    def set_duty_cycle_sp(self, duty_cycle_sp):
        self.duty_cycle_sp = duty_cycle_sp
