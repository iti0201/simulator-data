from __future__ import division

import numpy

from random import uniform
import rospy
import time
import math
from std_msgs.msg import String
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState


class GazGyro:
    def __init__(self):
        self.gyro_data = None
        self.times_listened = 0
        self.initial_yaw = None
        self.current_yaw = None
        self.current_time = None
        self.previous_yaw = None
        self.total_yaw = 0
        self.sub = rospy.Subscriber('lego_robot/gyro_joint_states', JointState, self.callback)

    def callback(self, data):
        if self.times_listened >= 1:
            self.gyro_data = data
            if self.initial_yaw is None and self.gyro_data is not None:
                self.initial_yaw = self.get_yaw()
                self.current_yaw = self.initial_yaw
                self.previous_yaw = self.initial_yaw
                self.current_time = time.time()
                # print("just set the initial yaw to: ", self.initial_yaw)
            else:
                self.get_yaw()
        self.times_listened += 1  # for some reason the first set of data always gives old data

    def get_yaw(self):
        if self.gyro_data is not None:
            robot_orientation = self.gyro_data.position # this gets the robot's orientation
            quaternion = (
                robot_orientation[0],
                robot_orientation[1],
                robot_orientation[2],
                robot_orientation[3])
            euler = euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = math.degrees(euler[2])
            if yaw < 0:
                yaw += 360
            if self.initial_yaw is None:
                return yaw
            else:
                noise_param = rospy.get_param('gyro_noise', 0.05)

                old_total = self.total_yaw
                if yaw - self.current_yaw < -270:
                    self.total_yaw += 360 + yaw - self.current_yaw
                elif yaw - self.current_yaw > 270:
                    self.total_yaw += yaw - self.current_yaw - 360
                else:
                    self.total_yaw += yaw - self.current_yaw
                self.current_yaw = yaw
                if math.fabs(old_total - self.total_yaw) > 0.1:
                    self.total_yaw += uniform(-3, 3) * noise_param
                return self.total_yaw
        else:
            return 0 # the problem here is that if you ask the yaw immediately after initialising the sensor, it won't have enough time to fetch the data

    def return_yaw(self):
        return -self.total_yaw

    def get_rate(self):
        if self.previous_yaw is not None:
            if self.initial_yaw == self.previous_yaw:
                self.previous_yaw = 0
            yaw = self.get_yaw()
            previous_yaw = self.previous_yaw
            previous_time = self.current_time
            self.previous_yaw = yaw
            self.current_time = time.time()
            yaw_difference = self.previous_yaw - previous_yaw
            if yaw_difference < -270:
                yaw_difference += 360
            elif yaw_difference > 270:
                yaw_difference -= 360
            return -yaw_difference / (self.current_time - previous_time)
        else:
            return 0 # the problem here is that if you ask the yaw immediately after initialising the sensor, it won't have enough time to fetch the data

    def reset_gyro(self):
        self.gyro_data = None
        self.total_yaw = 0
        self.initial_yaw = None
        self.current_yaw = None
        self.previous_yaw = None


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
