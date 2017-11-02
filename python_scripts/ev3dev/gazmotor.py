import rospy
import geometry_msgs.msg
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
import math
import numpy


class GazMotor:

    def __init__(self, wheel):
        self.wheel = wheel
        self.terminated = False
        self.stopped = False
        self.duty_cycle_sp = None
        self.clear = True
        self.motor_counter = 0
        self.current_speed = 0
        self.current_yaw = 0
        self.current_angle = 0
        self.previous_wheel_yaw = None
        sub = rospy.Subscriber('gazebo/link_states', LinkStates, self.linkcallback)
        sub2 = rospy.Subscriber('gazebo/model_states', ModelStates, self.modelcallback)

    def linkcallback(self, data):
        index = data.name.index('lego_robot::' + str(self.wheel) + '_wheel')
        self.current_speed = data.twist[index].angular.x
        wheel_orientation = data.pose[index].orientation
        quaternion = (
                wheel_orientation.x,
                wheel_orientation.y,
                wheel_orientation.z,
                wheel_orientation.w)
        euler = euler_from_quaternion(quaternion)
        wheel_yaw = euler[1]
        if self.previous_wheel_yaw is not None:
            delta = math.fabs(self.previous_wheel_yaw - wheel_yaw)
            if self.get_speed() > 0:
                #print("Current angle: " + str(self.current_angle))
                self.current_angle += delta
            elif self.get_speed() < 0:
                self.current_angle -= delta
        self.previous_wheel_yaw = wheel_yaw

        #print(self.current_angle)
        #if str(self.wheel) == 'left':
         #   print(str(self.current_speed))

    def modelcallback(self, data):
        index = data.name.index('lego_robot')
        robot_orientation = data.pose[index].orientation
        quaternion = (
                robot_orientation.x,
                robot_orientation.y,
                robot_orientation.z,
                robot_orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.current_yaw = yaw

    def talker_timed(self, kwargs):
        self.motor_counter += 1
        current_counter = self.motor_counter
        pub = rospy.Publisher('lego_robot/cmd_vel_' + self.wheel, Twist, queue_size=10)

        publishFrequency = 60
        rate = rospy.Rate(publishFrequency)  # 60hz
        counter = 0

        max_speed = 1050
        speed_sp = kwargs['speed_sp']
        if (speed_sp > max_speed):
            speed_sp = max_speed
        elif (speed_sp < -max_speed):
            speed_sp = -max_speed

        current_time = rospy.get_time()
        while current_time == 0: # this is necessary because for some reason sometimes rospy.get_time() outputs 0
            current_time = rospy.get_time()

        t_end = current_time + kwargs['time_sp'] / 1000

        while not rospy.is_shutdown() and not self.stopped and rospy.get_time() < t_end:
            #rint("current time: " + str(rospy.get_time()))
            #print("end time: " + str(t_end))
            if self.motor_counter != current_counter:
                return
            # print(counter)
            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()

            angular.x = math.radians(speed_sp)
            # angular.x = kwargs['speed_sp'] * math.pi / 180

            counter += 1

            twist.angular = angular
            twist.linear = linear

            # rospy.loginfo(twist)
            pub.publish(twist)

            rate.sleep()

        self.coast(pub, math.radians(speed_sp))

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

        max_speed = 1050
        speed_sp = kwargs['speed_sp']
        if (speed_sp > max_speed):
            speed_sp = max_speed
        elif (speed_sp < -max_speed):
            speed_sp = -max_speed

        while not rospy.is_shutdown() and not self.stopped:
            if self.motor_counter != current_counter:
                return
            # print(counter, self.wheel, self.motor_counter)
            twist = Twist()
            angular = geometry_msgs.msg.Vector3()
            linear = geometry_msgs.msg.Vector3()
            angular.x = math.radians(speed_sp)
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
            rospy.sleep(0.1)

    def activate_thread(self):

        self.stopped = False

    def stop_thread(self):
        # print('STOPPING')
        self.stopped = True
        self.clear = True

    def set_duty_cycle_sp(self, duty_cycle_sp):
        self.duty_cycle_sp = duty_cycle_sp

    def set_position(self, position):
        self.current_angle = math.radians(position)

    def get_speed(self):
        if math.cos(self.current_yaw) == 0:
            return 0
        return -math.degrees(self.current_speed / math.cos(self.current_yaw))

    def get_ticks(self):
        return math.degrees(self.current_angle)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)


def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
