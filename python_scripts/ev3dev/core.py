# -----------------------------------------------------------------------------
# Copyright (c) 2015 Ralph Hempel <rhempel@hempeldesigngroup.com>
# Copyright (c) 2015 Anton Vanhoucke <antonvh@gmail.com>
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
# Copyright (c) 2015 Eric Pascual <eric@pobot.org>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

# ~autogen autogen-header
# Sections of the following code were auto-generated based on spec v1.2.0

# ~autogen

# -----------------------------------------------------------------------------

# This file has been modified by Rasmus Iila <rasmusiila@gmail.com>

import sys

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')

# -----------------------------------------------------------------------------

import os
import io
import fnmatch
import numbers
import array
import mmap
import ctypes
import re
import stat
import select
import time
from os.path import abspath
from struct import pack, unpack
from subprocess import Popen, check_output

# ADDED
from ev3dev.gazmotor import GazMotor
from ev3dev.gazmedmotor import GazMedMotor
from ev3dev.gazcolor import GazColor
from ev3dev.gazsonar import GazSonar
from ev3dev.gazgyro import GazGyro
from ev3dev.gaztouch import GazTouch
import threading
import rospy

rospy.init_node('talker', anonymous=True)

try:
    # This is a linux-specific module.
    # It is required by the Button() class, but failure to import it may be
    # safely ignored if one just needs to run API tests on Windows.
    import fcntl
except ImportError:
    print("WARNING: Failed to import fcntl. Button class will be unuseable!")

INPUT_AUTO = ''
OUTPUT_AUTO = ''


# -----------------------------------------------------------------------------
def list_device_names(class_path, name_pattern, **kwargs):
    """
    This is a generator function that lists names of all devices matching the
    provided parameters.

    Parameters:
	class_path: class path of the device, a subdirectory of /sys/class.
	    For example, '/sys/class/tacho-motor'.
	name_pattern: pattern that device name should match.
	    For example, 'sensor*' or 'motor*'. Default value: '*'.
	keyword arguments: used for matching the corresponding device
	    attributes. For example, address='outA', or
	    driver_name=['lego-ev3-us', 'lego-nxt-us']. When argument value
	    is a list, then a match against any entry of the list is
	    enough.
    """

    if not os.path.isdir(class_path):
        return

    def matches(attribute, pattern):
        try:
            with io.FileIO(attribute) as f:
                value = f.read().strip().decode()
        except:
            return False

        if isinstance(pattern, list):
            return any([value.find(p) >= 0 for p in pattern])
        else:
            return value.find(pattern) >= 0

    for f in os.listdir(class_path):
        if fnmatch.fnmatch(f, name_pattern):
            path = class_path + '/' + f
            if all([matches(path + '/' + k, kwargs[k]) for k in kwargs]):
                yield f


# -----------------------------------------------------------------------------
# Define the base class from which all other ev3dev classes are defined.

class Device(object):
    """The ev3dev device base class"""

    __slots__ = ['_path', 'connected', '_device_index', 'kwargs']

    DEVICE_ROOT_PATH = '/sys/class'

    _DEVICE_INDEX = re.compile(r'^.*(?P<idx>\d+)$')

    def __init__(self, class_name, name_pattern='*', name_exact=False, **kwargs):
        """Spin through the Linux sysfs class for the device type and find
        a device that matches the provided name pattern and attributes (if any).

        Parameters:
            class_name: class name of the device, a subdirectory of /sys/class.
                For example, 'tacho-motor'.
            name_pattern: pattern that device name should match.
                For example, 'sensor*' or 'motor*'. Default value: '*'.
            name_exact: when True, assume that the name_pattern provided is the
                exact device name and use it directly.
            keyword arguments: used for matching the corresponding device
                attributes. For example, address='outA', or
                driver_name=['lego-ev3-us', 'lego-nxt-us']. When argument value
                is a list, then a match against any entry of the list is
                enough.

        Example::

            d = ev3dev.Device('tacho-motor', address='outA')
            s = ev3dev.Device('lego-sensor', driver_name=['lego-ev3-us', 'lego-nxt-us'])

        When connected succesfully, the `connected` attribute is set to True.
        """

        classpath = abspath(Device.DEVICE_ROOT_PATH + '/' + class_name)
        self.kwargs = kwargs

        def get_index(file):
            match = Device._DEVICE_INDEX.match(file)
            if match:
                return int(match.group('idx'))
            else:
                return None

        if name_exact:
            self._path = classpath + '/' + name_pattern
            self._device_index = get_index(name_pattern)
            self.connected = True
        else:
            try:
                name = next(list_device_names(classpath, name_pattern, **kwargs))
                self._path = classpath + '/' + name
                self._device_index = get_index(name)
                self.connected = True
            except StopIteration:
                self._path = None
                self._device_index = None
                self.connected = False

    def __str__(self):
        if 'address' in self.kwargs:
            return "%s(%s)" % (self.__class__.__name__, self.kwargs.get('address'))
        else:
            return self.__class__.__name__

    def _attribute_file_open(self, name):
        path = self._path + '/' + name
        mode = stat.S_IMODE(os.stat(path)[stat.ST_MODE])
        r_ok = mode & stat.S_IRGRP
        w_ok = mode & stat.S_IWGRP

        if r_ok and w_ok:
            mode = 'r+'
        elif w_ok:
            mode = 'w'
        else:
            mode = 'r'

        return io.FileIO(path, mode)

    def _get_attribute(self, attribute, name):
        """Device attribute getter"""
        if self.connected:
            if None == attribute:
                attribute = self._attribute_file_open(name)
            else:
                attribute.seek(0)
            return attribute, attribute.read().strip().decode()
        else:
            raise Exception('Device is not connected')

    def _set_attribute(self, attribute, name, value):
        """Device attribute setter"""
        if self.connected:
            if None == attribute:
                attribute = self._attribute_file_open(name)
            else:
                attribute.seek(0)
            attribute.write(value.encode())
            attribute.flush()
            return attribute
        else:
            raise Exception('Device is not connected')

    def get_attr_int(self, attribute, name):
        attribute, value = self._get_attribute(attribute, name)
        return attribute, int(value)

    def set_attr_int(self, attribute, name, value):
        return self._set_attribute(attribute, name, str(int(value)))

    def get_attr_string(self, attribute, name):
        return self._get_attribute(attribute, name)

    def set_attr_string(self, attribute, name, value):
        return self._set_attribute(attribute, name, value)

    def get_attr_line(self, attribute, name):
        return self._get_attribute(attribute, name)

    def get_attr_set(self, attribute, name):
        attribute, value = self.get_attr_line(attribute, name)
        return attribute, [v.strip('[]') for v in value.split()]

    def get_attr_from_set(self, attribute, name):
        attribute, value = self.get_attr_line(attribute, name)
        for a in value.split():
            v = a.strip('[]')
            if v != a:
                return v
        return ""

    @property
    def device_index(self):
        return self._device_index


def list_devices(class_name, name_pattern, **kwargs):
    """
    This is a generator function that takes same arguments as `Device` class
    and enumerates all devices present in the system that match the provided
    arguments.

    Parameters:
	class_name: class name of the device, a subdirectory of /sys/class.
	    For example, 'tacho-motor'.
	name_pattern: pattern that device name should match.
	    For example, 'sensor*' or 'motor*'. Default value: '*'.
	keyword arguments: used for matching the corresponding device
	    attributes. For example, address='outA', or
	    driver_name=['lego-ev3-us', 'lego-nxt-us']. When argument value
	    is a list, then a match against any entry of the list is
	    enough.
    """
    classpath = abspath(Device.DEVICE_ROOT_PATH + '/' + class_name)

    return (Device(class_name, name, name_exact=True)
            for name in list_device_names(classpath, name_pattern, **kwargs))


# ~autogen generic-class classes.motor>currentClass

class Motor(Device):
    """
    The motor class provides a uniform interface for using motors with
    positional and directional feedback such as the EV3 and NXT motors.
    This feedback allows for precise control of the motors. This is the
    most common type of motor, so we just call it `motor`.

    The way to configure a motor is to set the '_sp' attributes when
    calling a command or before. Only in 'run_direct' mode attribute
    changes are processed immediately, in the other modes they only
    take place when a new command is issued.
    """

    SYSTEM_CLASS_NAME = 'tacho-motor'
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if address is not None:
            kwargs['address'] = address
        super(Motor, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._command = None
        self._commands = None
        self._count_per_rot = None
        self._count_per_m = None
        self._driver_name = None
        self._duty_cycle = None
        self._duty_cycle_sp = None
        self._full_travel_count = None
        self._polarity = None
        self._position = None
        self._position_p = None
        self._position_i = None
        self._position_d = None
        self._position_sp = None
        self._max_speed = None
        self._speed = None
        self._speed_sp = None
        self._ramp_up_sp = None
        self._ramp_down_sp = None
        self._speed_p = None
        self._speed_i = None
        self._speed_d = None
        self._state = None
        self._stop_action = None
        self._stop_actions = None
        self._time_sp = None
        self._threads = None
        self._motor = None

        # ~autogen

        self._poll = None

    __slots__ = [
        # ~autogen generic-class-slots classes.motor>currentClass

        '_address',
        '_command',
        '_commands',
        '_count_per_rot',
        '_count_per_m',
        '_driver_name',
        '_duty_cycle',
        '_duty_cycle_sp',
        '_full_travel_count',
        '_polarity',
        '_position',
        '_position_p',
        '_position_i',
        '_position_d',
        '_position_sp',
        '_max_speed',
        '_speed',
        '_speed_sp',
        '_ramp_up_sp',
        '_ramp_down_sp',
        '_speed_p',
        '_speed_i',
        '_speed_d',
        '_state',
        '_stop_action',
        '_stop_actions',
        '_time_sp',
        '_threads',
        '_motor',

        # ~autogen
        '_poll',
    ]

    # ~autogen generic-get-set classes.motor>currentClass

    @property
    def address(self):
        """
        Returns the name of the port that this motor is connected to.
        """
        return self._address

    @property
    def command(self):
        """
        Sends a command to the motor controller. See `commands` for a list of
        possible values.
        """
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self._command = value

    @property
    def commands(self):
        """
        Returns a list of commands that are supported by the motor
        controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
        `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.

        - `run-forever` will cause the motor to run until another command is sent.
        - `run-to-abs-pos` will run to an absolute position specified by `position_sp` NOP
          and then stop using the action specified in `stop_action`.
        - `run-to-rel-pos` will run to a position relative to the current `position` value. NO
          The new position will be current `position` + `position_sp`. When the new
          position is reached, the motor will stop using the action specified by `stop_action`.
        - `run-timed` will run the motor for the amount of time specified in `time_sp`
          and then stop the motor using the action specified by `stop_action`.
        - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
          Unlike other run commands, changing `duty_cycle_sp` while running *will*
          take effect immediately.
        - `stop` will stop any of the run commands before they are complete using the
          action specified by `stop_action`.
        - `reset` will reset all of the motor parameter attributes to their default value.
          This will also have the effect of stopping the motor.
        """
        return self._commands

    @property
    def count_per_rot(self):
        """
        Returns the number of tacho counts in one rotation of the motor. Tacho counts
        are used by the position and speed attributes, so you can use this value
        to convert rotations or degrees to tacho counts. (rotation motors only)
        """
        return self._count_per_rot

    @property
    def count_per_m(self):
        """
        Returns the number of tacho counts in one meter of travel of the motor. Tacho
        counts are used by the position and speed attributes, so you can use this
        value to convert from distance to tacho counts. (linear motors only)
        """
        return self._count_per_m

    @property
    def driver_name(self):
        """
        Returns the name of the driver that provides this tacho motor device.
        """
        return self._driver_name

    @property
    def duty_cycle(self):
        """
        Returns the current duty cycle of the motor. Units are percent. Values
        are -100 to 100.
        """
        return self._duty_cycle

    @property
    def duty_cycle_sp(self):
        """
        Writing sets the duty cycle setpoint. Reading returns the current value.
        Units are in percent. Valid values are -100 to 100. A negative value causes
        the motor to rotate in reverse.
        """
        return self._duty_cycle_sp

    @duty_cycle_sp.setter
    def duty_cycle_sp(self, value):
        self._duty_cycle_sp = value
        self._motor.set_duty_cycle_sp(self._duty_cycle_sp)

    @property
    def full_travel_count(self):
        """
        Returns the number of tacho counts in the full travel of the motor. When
        combined with the `count_per_m` atribute, you can use this value to
        calculate the maximum travel distance of the motor. (linear motors only)
        """
        return self._full_travel_count

    @property
    def polarity(self):
        """
        Sets the polarity of the motor. With `normal` polarity, a positive duty
        cycle will cause the motor to rotate clockwise. With `inversed` polarity,
        a positive duty cycle will cause the motor to rotate counter-clockwise.
        Valid values are `normal` and `inversed`.
        """
        return self._polarity

    @polarity.setter
    def polarity(self, value):
        self._polarity = value

    @property
    def position(self):
        """
        Returns the current position of the motor in pulses of the rotary
        encoder. When the motor rotates clockwise, the position will increase.
        Likewise, rotating counter-clockwise causes the position to decrease.
        Writing will set the position to that value.
        """
        return int('{0:g}'.format(round(self._motor.get_ticks(), 0)))

    @position.setter
    def position(self, value):
        self._position = value
        self._motor.set_position(self._position)

    @property
    def position_p(self):
        """
        The proportional constant for the position PID.
        """
        return self._position_p

    @position_p.setter
    def position_p(self, value):
        self._position_p = value

    @property
    def position_i(self):
        """
        The integral constant for the position PID.
        """
        return self._position_i

    @position_i.setter
    def position_i(self, value):
        self._position_i = value

    @property
    def position_d(self):
        """
        The derivative constant for the position PID.
        """
        return self._position_d

    @position_d.setter
    def position_d(self, value):
        self._position_d = value

    @property
    def position_sp(self):
        """
        Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
        commands. Reading returns the current value. Units are in tacho counts. You
        can use the value returned by `counts_per_rot` to convert tacho counts to/from
        rotations or degrees.
        """
        return self._position_sp

    @position_sp.setter
    def position_sp(self, value):
        self._position_sp = value

    @property
    def max_speed(self):
        """
        Returns the maximum value that is accepted by the `speed_sp` attribute. This
        may be slightly different than the maximum speed that a particular motor can
        reach - it's the maximum theoretical speed.
        """
        return self._max_speed

    @property
    def speed(self):
        """
        Returns the current motor speed in tacho counts per second. Note, this is
        not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
        attribute to convert this value to RPM or deg/sec.
        """
        return int('{0:g}'.format(round(self._motor.get_speed(), 0)))

    @property
    def speed_sp(self):
        """
        Writing sets the target speed in tacho counts per second used for all `run-*`
        commands except `run-direct`. Reading returns the current value. A negative
        value causes the motor to rotate in reverse with the exception of `run-to-*-pos`
        commands where the sign is ignored. Use the `count_per_rot` attribute to convert
        RPM or deg/sec to tacho counts per second. Use the `count_per_m` attribute to
        convert m/s to tacho counts per second.
        """
        return self._speed_sp

    @speed_sp.setter
    def speed_sp(self, value):
        self._speed_sp = value

    @property
    def ramp_up_sp(self):
        """
        Writing sets the ramp up setpoint. Reading returns the current value. Units
        are in milliseconds and must be positive. When set to a non-zero value, the
        motor speed will increase from 0 to 100% of `max_speed` over the span of this
        setpoint. The actual ramp time is the ratio of the difference between the
        `speed_sp` and the current `speed` and max_speed multiplied by `ramp_up_sp`.
        """
        return self._ramp_up_sp

    @ramp_up_sp.setter
    def ramp_up_sp(self, value):
        self._ramp_up_sp = value

    @property
    def ramp_down_sp(self):
        """
        Writing sets the ramp down setpoint. Reading returns the current value. Units
        are in milliseconds and must be positive. When set to a non-zero value, the
        motor speed will decrease from 0 to 100% of `max_speed` over the span of this
        setpoint. The actual ramp time is the ratio of the difference between the
        `speed_sp` and the current `speed` and max_speed multiplied by `ramp_down_sp`.
        """
        return self._ramp_down_sp

    @ramp_down_sp.setter
    def ramp_down_sp(self, value):
        self._ramp_down_sp = value

    @property
    def speed_p(self):
        """
        The proportional constant for the speed regulation PID.
        """
        return self._speed_p

    @speed_p.setter
    def speed_p(self, value):
        self._speed_p = value

    @property
    def speed_i(self):
        """
        The integral constant for the speed regulation PID.
        """
        return self._speed_i

    @speed_i.setter
    def speed_i(self, value):
        self._speed_i = value

    @property
    def speed_d(self):
        """
        The derivative constant for the speed regulation PID.
        """
        return self._speed_d

    @speed_d.setter
    def speed_d(self, value):
        self._speed_d = value

    @property
    def state(self):
        """
        Reading returns a list of state flags. Possible flags are
        `running`, `ramping`, `holding`, `overloaded` and `stalled`.
        """
        return self._state

    @property
    def stop_action(self):
        """
        Reading returns the current stop action. Writing sets the stop action.
        The value determines the motors behavior when `command` is set to `stop`.
        Also, it determines the motors behavior when a run command completes. See
        `stop_actions` for a list of possible values.
        """
        return self._stop_action

    @stop_action.setter
    def stop_action(self, value):
        self._stop_action = value

    @property
    def stop_actions(self):
        """
        Returns a list of stop actions supported by the motor controller.
        Possible values are `coast`, `brake` and `hold`. `coast` means that power will
        be removed from the motor and it will freely coast to a stop. `brake` means
        that power will be removed from the motor and a passive electrical load will
        be placed on the motor. This is usually done by shorting the motor terminals
        together. This load will absorb the energy from the rotation of the motors and
        cause the motor to stop more quickly than coasting. `hold` does not remove
        power from the motor. Instead it actively tries to hold the motor at the current
        position. If an external force tries to turn the motor, the motor will 'push
        back' to maintain its position.
        """
        return self._stop_actions

    @property
    def time_sp(self):
        """
        Writing specifies the amount of time the motor will run when using the
        `run-timed` command. Reading returns the current value. Units are in
        milliseconds.
        """
        return self._time_sp

    @time_sp.setter
    def time_sp(self, value):
        self._time_sp = value

    # ~autogen
    # ~autogen generic-property-value classes.motor>currentClass

    # Run the motor until another command is sent.
    COMMAND_RUN_FOREVER = 'run-forever'

    # Run to an absolute position specified by `position_sp` and then
    # stop using the action specified in `stop_action`.
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'

    # Run to a position relative to the current `position` value.
    # The new position will be current `position` + `position_sp`.
    # When the new position is reached, the motor will stop using
    # the action specified by `stop_action`.
    COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'

    # Run the motor for the amount of time specified in `time_sp`
    # and then stop the motor using the action specified by `stop_action`.
    COMMAND_RUN_TIMED = 'run-timed'

    # Run the motor at the duty cycle specified by `duty_cycle_sp`.
    # Unlike other run commands, changing `duty_cycle_sp` while running *will*
    # take effect immediately.
    COMMAND_RUN_DIRECT = 'run-direct'

    # Stop any of the run commands before they are complete using the
    # action specified by `stop_action`.
    COMMAND_STOP = 'stop'

    # Reset all of the motor parameter attributes to their default value.
    # This will also have the effect of stopping the motor.
    COMMAND_RESET = 'reset'

    # Sets the normal polarity of the rotary encoder.
    ENCODER_POLARITY_NORMAL = 'normal'

    # Sets the inversed polarity of the rotary encoder.
    ENCODER_POLARITY_INVERSED = 'inversed'

    # With `normal` polarity, a positive duty cycle will
    # cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    # With `inversed` polarity, a positive duty cycle will
    # cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    # Power is being sent to the motor.
    STATE_RUNNING = 'running'

    # The motor is ramping up or down and has not yet reached a constant output level.
    STATE_RAMPING = 'ramping'

    # The motor is not turning, but rather attempting to hold a fixed position.
    STATE_HOLDING = 'holding'

    # The motor is turning, but cannot reach its `speed_sp`.
    STATE_OVERLOADED = 'overloaded'

    # The motor is not turning when it should be.
    STATE_STALLED = 'stalled'

    # Power will be removed from the motor and it will freely coast to a stop.
    STOP_ACTION_COAST = 'coast'

    # Power will be removed from the motor and a passive electrical load will
    # be placed on the motor. This is usually done by shorting the motor terminals
    # together. This load will absorb the energy from the rotation of the motors and
    # cause the motor to stop more quickly than coasting.
    STOP_ACTION_BRAKE = 'brake'

    # Does not remove power from the motor. Instead it actively try to hold the motor
    # at the current position. If an external force tries to turn the motor, the motor
    # will `push back` to maintain its position.
    STOP_ACTION_HOLD = 'hold'

    # ~autogen
    # ~autogen motor_commands classes.motor>currentClass

    def run_forever(self, **kwargs):
        """Run the motor until another command is sent.
        """
        if 'speed_sp' not in kwargs:
            kwargs['speed_sp'] = self._speed_sp
        else:
            self.speed_sp = kwargs['speed_sp']
        t = threading.Thread(target=self._motor.talker_forever, args=(kwargs,))
        if len(self._threads) > 0:
            self._threads = []
        self._threads.append(t)
        self._motor.activate_thread()
        t.start() # TODO threadi saad initis teha

    def run_to_abs_pos(self, **kwargs):
        """Run to an absolute position specified by `position_sp` and then
        stop using the action specified in `stop_action`.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TO_ABS_POS

    def run_to_rel_pos(self, **kwargs):
        """Run to a position relative to the current `position` value.
        The new position will be current `position` + `position_sp`.
        When the new position is reached, the motor will stop using
        the action specified by `stop_action`.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TO_REL_POS

    def run_timed(self, **kwargs):
        """Run the motor for the amount of time specified in `time_sp`
        and then stop the motor using the action specified by `stop_action`.
        This mock only implements the 'coast' stop_action.
        """
        if 'speed_sp' not in kwargs:
            kwargs['speed_sp'] = self._speed_sp
        else:
            self.speed_sp = kwargs['speed_sp']
        if 'time_sp' not in kwargs:
            kwargs['time_sp'] = self._time_sp
        else:
            self.time_sp = kwargs['time_sp']
        t = threading.Thread(target=self._motor.talker_timed, args=(kwargs,))
        if len(self._threads) > 0:
            self._threads = []
        self._threads.append(t)
        self._motor.activate_thread()
        t.start()

    def run_direct(self, **kwargs):
        """Run the motor at the duty cycle specified by `duty_cycle_sp`.
        Unlike other run commands, changing `duty_cycle_sp` while running *will*
        take effect immediately.
        """
        if 'duty_cycle_sp' in kwargs:
            self.duty_cycle_sp = kwargs['duty_cycle_sp']
        self._motor.set_duty_cycle_sp(self._duty_cycle_sp)
        t = threading.Thread(target=self._motor.talker_direct)
        if len(self._threads) > 0:
            self._threads = []
        self._threads.append(t)
        self._motor.activate_thread()
        t.start()

    def stop(self, **kwargs):
        """Stop any of the run commands before they are complete using the
        action specified by `stop_action`.
        This mock only implements the 'coast' stop_action.
        """
        self._motor.stop_thread()
        self._threads = []

    def reset(self, **kwargs):
        """Reset all of the motor parameter attributes to their default value.
        This will also have the effect of stopping the motor.
        """
        self._commands = ['run-forever', 'run-to-abs-pos', 'run-to-rel-pos', 'run_timed', 'run-direct', 'stop', 'reset']
        self._count_per_rot = 360
        self._driver_name = 'lego-ev3-l-motor'
        self._duty_cycle = 0
        self.duty_cycle_sp = 0
        self._polarity = 'normal'
        self.position = 0
        self._position_p = 80000
        self._position_i = 0
        self._position_d = 0
        self._position_sp = 0
        self._max_speed = 1050
        self._speed = 0
        self._speed_sp = 0
        self._ramp_up_sp = 0
        self._ramp_down_sp = 0
        self._speed_p = 1000
        self._speed_i = 60
        self._speed_d = 0
        self._state = []
        self._stop_action = 'coast'
        self._stop_actions = ['coast', 'brake', 'hold']
        self._time_sp = 0

        self._motor.stop_thread()
        self._threads = []

    # ~autogen

    def wait(self, cond, timeout=None):
        """
        Blocks until ``cond(self.state)`` is ``True``.  The condition is
        checked when there is an I/O event related to the ``state`` attribute.
        Exits early when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.
        """

        tic = time.time()

        if self._poll is None:
            if self._state is None:
                self._state = self._attribute_file_open('state')
            self._poll = select.poll()
            self._poll.register(self._state, select.POLLPRI)

        while True:
            self._poll.poll(None if timeout is None else timeout)

            if timeout is not None and time.time() >= tic + timeout / 1000:
                return False

            if cond(self.state):
                return True

    def wait_until(self, s, timeout=None):
        """
        Blocks until ``s`` is in ``self.state``.  The condition is checked when
        there is an I/O event related to the ``state`` attribute.  Exits early
        when ``timeout`` (in milliseconds) is reached.

        .. warning::
            In ev3dev kernel release cycles 16 and earlier, there is a bug
            which causes the ``state`` attribute to include ``stalled``
            immediately after starting the motor even if it is not actually
            being prevented from rotating. As a workaround, we recommend
            sleeping your code for around 100ms after starting a motor if you
            are going to use this method to wait for it to be ``stalled``. A
            fix for this has not yet been released.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_until('stalled')
        """
        return self.wait(lambda state: s in state, timeout)

    def wait_while(self, s, timeout=None):
        """
        Blocks until ``s`` is not in ``self.state``.  The condition is checked
        when there is an I/O event related to the ``state`` attribute.  Exits
        early when ``timeout`` (in milliseconds) is reached.

        .. warning::
            In ev3dev kernel release cycles 16 and earlier, there is a bug
            which causes the ``state`` attribute to include ``stalled``
            immediately after starting the motor even if it is not actually
            being prevented from rotating. As a workaround, we recommend
            sleeping your code for around 100ms after starting a motor if you
            are going to use this method to wait while it is ``stalled``. A
            fix for this has not yet been released.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_while('running')
        """
        return self.wait(lambda state: s not in state, timeout)


def list_motors(name_pattern=Motor.SYSTEM_DEVICE_NAME_CONVENTION, **kwargs):
    """
    This is a generator function that enumerates all tacho motors that match
    the provided arguments.

    Parameters:
	name_pattern: pattern that device name should match.
	    For example, 'motor*'. Default value: '*'.
	keyword arguments: used for matching the corresponding device
	    attributes. For example, driver_name='lego-ev3-l-motor', or
	    address=['outB', 'outC']. When argument value
	    is a list, then a match against any entry of the list is
	    enough.
    """
    class_path = abspath(Device.DEVICE_ROOT_PATH + '/' + Motor.SYSTEM_CLASS_NAME)

    return (Motor(name_pattern=name, name_exact=True)
            for name in list_device_names(class_path, name_pattern, **kwargs))


# ~autogen generic-class classes.largeMotor>currentClass

class LargeMotor(Motor):
    """
    EV3/NXT large servo motor
    """

    SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is 'outA':
            wheel = 'left'
        elif address is 'outD':
            wheel = 'right'
        else:
            print('Sorry, this simulation assumes that the left motor is plugged into port A and '
                  'right motor is plugged into port D.')
            os._exit(1)
        m = GazMotor(wheel)

        super(LargeMotor, self).__init__(address, name_pattern, name_exact,
                                         driver_name=['lego-ev3-l-motor', 'lego-nxt-motor'], **kwargs)

        self._address = address
        #self._command = None  # writeonly
        self._commands = ['run-forever', 'run-to-abs-pos', 'run-to-rel-pos', 'run_timed', 'run-direct', 'stop', 'reset']
        self._count_per_rot = 360
        #self._count_per_m = None  #
        self._driver_name = 'lego-ev3-l-motor'
        self._duty_cycle = 0
        self._duty_cycle_sp = 0
        #self._full_travel_count = None  #
        self._polarity = 'normal'
        self._position = 0
        self._position_p = 80000
        self._position_i = 0
        self._position_d = 0
        self._position_sp = 0
        self._max_speed = 1050
        self._speed = 0
        self._speed_sp = 0
        self._ramp_up_sp = 0
        self._ramp_down_sp = 0
        self._speed_p = 1000
        self._speed_i = 60
        self._speed_d = 0
        self._state = []
        self._stop_action = 'coast' # lego robot says 'coast'
        self._stop_actions = ['coast', 'brake', 'hold']
        self._time_sp = 0

        self._motor = m
        self._threads = []

    # ~autogen
    __slots__ = [
        # ~autogen generic-class-slots classes.largeMotor>currentClass

        # ~autogen
    ]


# ~autogen generic-class classes.mediumMotor>currentClass

class MediumMotor(Motor):
    """
    EV3 medium servo motor
    """

    SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not 'outB':
            print('Sorry, this simulation assumes that the medium motor is plugged in to port outB')
            os._exit(1)

        med = GazMedMotor()

        super(MediumMotor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-ev3-m-motor'], **kwargs)

        self._address = address
        # self._command = None  # writeonly
        self._commands = ['run-forever', 'run-to-abs-pos', 'run-to-rel-pos', 'run_timed', 'run-direct', 'stop', 'reset']
        self._count_per_rot = 360
        # self._count_per_m = None  #
        self._driver_name = 'lego-ev3-m-motor'
        self._duty_cycle = 0
        self._duty_cycle_sp = 0
        # self._full_travel_count = None  #
        self._polarity = 'normal'
        self._position = 0
        self._position_p = 160000
        self._position_i = 0
        self._position_d = 0
        self._position_sp = 0
        self._max_speed = 1560
        self._speed = 0
        self._speed_sp = 0
        self._ramp_up_sp = 0
        self._ramp_down_sp = 0
        self._speed_p = 1000
        self._speed_i = 60
        self._speed_d = 0
        self._state = []
        self._stop_action = 'coast'  # lego robot says 'coast'
        self._stop_actions = ['coast', 'brake', 'hold']
        self._time_sp = 0

        self._motor = med
        self._threads = []

    # ~autogen
    __slots__ = [
        # ~autogen generic-class-slots classes.mediumMotor>currentClass

        # ~autogen
    ]

    def run_to_abs_pos(self, **kwargs):
        """Run to an absolute position specified by `position_sp` and then
        stop using the action specified in `stop_action`.
        """
        if 'position_sp' in kwargs:
            self.position_sp = kwargs['position_sp']
        if 'speed_sp' in kwargs:
            self.speed_sp = kwargs['speed_sp']
        self._motor.set_duty_cycle_sp(self._position_sp)
        t = threading.Thread(target=self._motor.talker_abs_pos, args=(self.position_sp, self.speed_sp))
        if len(self._threads) > 0:
            self._threads = []
        self._threads.append(t)
        self._motor.activate_thread()
        t.start()

    def reset(self, **kwargs):
        """Reset all of the motor parameter attributes to their default value.
        This will also have the effect of stopping the motor.
        """
        self._commands = ['run-forever', 'run-to-abs-pos', 'run-to-rel-pos', 'run_timed', 'run-direct', 'stop', 'reset']
        self._count_per_rot = 360
        self._driver_name = 'lego-ev3-m-motor'
        self._duty_cycle = 0
        self._duty_cycle_sp = 0
        self._polarity = 'normal'
        self._position = 0
        self._position_p = 160000
        self._position_i = 0
        self._position_d = 0
        self._position_sp = 0
        self._max_speed = 1560
        self._speed = 0
        self._speed_sp = 0
        self._ramp_up_sp = 0
        self._ramp_down_sp = 0
        self._speed_p = 1000
        self._speed_i = 60
        self._speed_d = 0
        self._state = []
        self._stop_action = 'coast'
        self._stop_actions = ['coast', 'brake', 'hold']
        self._time_sp = 0

        self._motor.stop_thread()
        self._threads = []


# ~autogen generic-class classes.actuonix50Motor>currentClass

class ActuonixL1250Motor(Motor):
    """
    Actuonix L12 50 linear servo motor
    """

    SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = 'linear*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(ActuonixL1250Motor, self).__init__(address, name_pattern, name_exact, driver_name=['act-l12-ev3-50'],
                                                 **kwargs)

    # ~autogen
    __slots__ = [
        # ~autogen generic-class-slots classes.actuonix50Motor>currentClass

        # ~autogen
    ]


# ~autogen generic-class classes.actuonix100Motor>currentClass

class ActuonixL12100Motor(Motor):
    """
    Actuonix L12 100 linear servo motor
    """

    SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = 'linear*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(ActuonixL12100Motor, self).__init__(address, name_pattern, name_exact, driver_name=['act-l12-ev3-100'],
                                                  **kwargs)

    # ~autogen
    __slots__ = [
        # ~autogen generic-class-slots classes.actuonix100Motor>currentClass

        # ~autogen
    ]


# ~autogen generic-class classes.dcMotor>currentClass

class DcMotor(Device):
    """
    The DC motor class provides a uniform interface for using regular DC motors
    with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors
    and LEGO Power Functions motors.
    """

    SYSTEM_CLASS_NAME = 'dc-motor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'motor*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if address is not None:
            kwargs['address'] = address
        super(DcMotor, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._command = None
        self._commands = None
        self._driver_name = None
        self._duty_cycle = None
        self._duty_cycle_sp = None
        self._polarity = None
        self._ramp_down_sp = None
        self._ramp_up_sp = None
        self._state = None
        self._stop_action = None
        self._stop_actions = None
        self._time_sp = None

    # ~autogen

    __slots__ = [
        # ~autogen generic-class-slots classes.dcMotor>currentClass

        '_address',
        '_command',
        '_commands',
        '_driver_name',
        '_duty_cycle',
        '_duty_cycle_sp',
        '_polarity',
        '_ramp_down_sp',
        '_ramp_up_sp',
        '_state',
        '_stop_action',
        '_stop_actions',
        '_time_sp',

        # ~autogen
    ]

    # ~autogen generic-get-set classes.dcMotor>currentClass

    @property
    def address(self):
        """
        Returns the name of the port that this motor is connected to.
        """
        self._address, value = self.get_attr_string(self._address, 'address')
        return value

    @property
    def command(self):
        """
        Sets the command for the motor. Possible values are `run-forever`, `run-timed` and
        `stop`. Not all commands may be supported, so be sure to check the contents
        of the `commands` attribute.
        """
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self._command = self.set_attr_string(self._command, 'command', value)

    @property
    def commands(self):
        """
        Returns a list of commands supported by the motor
        controller.
        """
        self._commands, value = self.get_attr_set(self._commands, 'commands')
        return value

    @property
    def driver_name(self):
        """
        Returns the name of the motor driver that loaded this device. See the list
        of [supported devices] for a list of drivers.
        """
        self._driver_name, value = self.get_attr_string(self._driver_name, 'driver_name')
        return value

    @property
    def duty_cycle(self):
        """
        Shows the current duty cycle of the PWM signal sent to the motor. Values
        are -100 to 100 (-100% to 100%).
        """
        self._duty_cycle, value = self.get_attr_int(self._duty_cycle, 'duty_cycle')
        return value

    @property
    def duty_cycle_sp(self):
        """
        Writing sets the duty cycle setpoint of the PWM signal sent to the motor.
        Valid values are -100 to 100 (-100% to 100%). Reading returns the current
        setpoint.
        """
        self._duty_cycle_sp, value = self.get_attr_int(self._duty_cycle_sp, 'duty_cycle_sp')
        return value

    @duty_cycle_sp.setter
    def duty_cycle_sp(self, value):
        self._duty_cycle_sp = self.set_attr_int(self._duty_cycle_sp, 'duty_cycle_sp', value)

    @property
    def polarity(self):
        """
        Sets the polarity of the motor. Valid values are `normal` and `inversed`.
        """
        self._polarity, value = self.get_attr_string(self._polarity, 'polarity')
        return value

    @polarity.setter
    def polarity(self, value):
        self._polarity = self.set_attr_string(self._polarity, 'polarity', value)

    @property
    def ramp_down_sp(self):
        """
        Sets the time in milliseconds that it take the motor to ramp down from 100%
        to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0.
        """
        self._ramp_down_sp, value = self.get_attr_int(self._ramp_down_sp, 'ramp_down_sp')
        return value

    @ramp_down_sp.setter
    def ramp_down_sp(self, value):
        self._ramp_down_sp = self.set_attr_int(self._ramp_down_sp, 'ramp_down_sp', value)

    @property
    def ramp_up_sp(self):
        """
        Sets the time in milliseconds that it take the motor to up ramp from 0% to
        100%. Valid values are 0 to 10000 (10 seconds). Default is 0.
        """
        self._ramp_up_sp, value = self.get_attr_int(self._ramp_up_sp, 'ramp_up_sp')
        return value

    @ramp_up_sp.setter
    def ramp_up_sp(self, value):
        self._ramp_up_sp = self.set_attr_int(self._ramp_up_sp, 'ramp_up_sp', value)

    @property
    def state(self):
        """
        Gets a list of flags indicating the motor status. Possible
        flags are `running` and `ramping`. `running` indicates that the motor is
        powered. `ramping` indicates that the motor has not yet reached the
        `duty_cycle_sp`.
        """
        self._state, value = self.get_attr_set(self._state, 'state')
        return value

    @property
    def stop_action(self):
        """
        Sets the stop action that will be used when the motor stops. Read
        `stop_actions` to get the list of valid values.
        """
        raise Exception("stop_action is a write-only property!")

    @stop_action.setter
    def stop_action(self, value):
        self._stop_action = self.set_attr_string(self._stop_action, 'stop_action', value)

    @property
    def stop_actions(self):
        """
        Gets a list of stop actions. Valid values are `coast`
        and `brake`.
        """
        self._stop_actions, value = self.get_attr_set(self._stop_actions, 'stop_actions')
        return value

    @property
    def time_sp(self):
        """
        Writing specifies the amount of time the motor will run when using the
        `run-timed` command. Reading returns the current value. Units are in
        milliseconds.
        """
        self._time_sp, value = self.get_attr_int(self._time_sp, 'time_sp')
        return value

    @time_sp.setter
    def time_sp(self, value):
        self._time_sp = self.set_attr_int(self._time_sp, 'time_sp', value)

    # ~autogen
    # ~autogen generic-property-value classes.dcMotor>currentClass

    # Run the motor until another command is sent.
    COMMAND_RUN_FOREVER = 'run-forever'

    # Run the motor for the amount of time specified in `time_sp`
    # and then stop the motor using the action specified by `stop_action`.
    COMMAND_RUN_TIMED = 'run-timed'

    # Run the motor at the duty cycle specified by `duty_cycle_sp`.
    # Unlike other run commands, changing `duty_cycle_sp` while running *will*
    # take effect immediately.
    COMMAND_RUN_DIRECT = 'run-direct'

    # Stop any of the run commands before they are complete using the
    # action specified by `stop_action`.
    COMMAND_STOP = 'stop'

    # With `normal` polarity, a positive duty cycle will
    # cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    # With `inversed` polarity, a positive duty cycle will
    # cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    # Power will be removed from the motor and it will freely coast to a stop.
    STOP_ACTION_COAST = 'coast'

    # Power will be removed from the motor and a passive electrical load will
    # be placed on the motor. This is usually done by shorting the motor terminals
    # together. This load will absorb the energy from the rotation of the motors and
    # cause the motor to stop more quickly than coasting.
    STOP_ACTION_BRAKE = 'brake'

    # ~autogen
    # ~autogen motor_commands classes.dcMotor>currentClass

    def run_forever(self, **kwargs):
        """Run the motor until another command is sent.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_FOREVER

    def run_timed(self, **kwargs):
        """Run the motor for the amount of time specified in `time_sp`
        and then stop the motor using the action specified by `stop_action`.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TIMED

    def run_direct(self, **kwargs):
        """Run the motor at the duty cycle specified by `duty_cycle_sp`.
        Unlike other run commands, changing `duty_cycle_sp` while running *will*
        take effect immediately.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_DIRECT

    def stop(self, **kwargs):
        """Stop any of the run commands before they are complete using the
        action specified by `stop_action`.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_STOP


# ~autogen
# ~autogen generic-class classes.servoMotor>currentClass

class ServoMotor(Device):
    """
    The servo motor class provides a uniform interface for using hobby type
    servo motors.
    """

    SYSTEM_CLASS_NAME = 'servo-motor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'motor*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if address is not None:
            kwargs['address'] = address
        super(ServoMotor, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._command = None
        self._driver_name = None
        self._max_pulse_sp = None
        self._mid_pulse_sp = None
        self._min_pulse_sp = None
        self._polarity = None
        self._position_sp = None
        self._rate_sp = None
        self._state = None

    # ~autogen

    __slots__ = [
        # ~autogen generic-class-slots classes.servoMotor>currentClass

        '_address',
        '_command',
        '_driver_name',
        '_max_pulse_sp',
        '_mid_pulse_sp',
        '_min_pulse_sp',
        '_polarity',
        '_position_sp',
        '_rate_sp',
        '_state',

        # ~autogen
    ]

    # ~autogen generic-get-set classes.servoMotor>currentClass

    @property
    def address(self):
        """
        Returns the name of the port that this motor is connected to.
        """
        self._address, value = self.get_attr_string(self._address, 'address')
        return value

    @property
    def command(self):
        """
        Sets the command for the servo. Valid values are `run` and `float`. Setting
        to `run` will cause the servo to be driven to the position_sp set in the
        `position_sp` attribute. Setting to `float` will remove power from the motor.
        """
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self._command = self.set_attr_string(self._command, 'command', value)

    @property
    def driver_name(self):
        """
        Returns the name of the motor driver that loaded this device. See the list
        of [supported devices] for a list of drivers.
        """
        self._driver_name, value = self.get_attr_string(self._driver_name, 'driver_name')
        return value

    @property
    def max_pulse_sp(self):
        """
        Used to set the pulse size in milliseconds for the signal that tells the
        servo to drive to the maximum (clockwise) position_sp. Default value is 2400.
        Valid values are 2300 to 2700. You must write to the position_sp attribute for
        changes to this attribute to take effect.
        """
        self._max_pulse_sp, value = self.get_attr_int(self._max_pulse_sp, 'max_pulse_sp')
        return value

    @max_pulse_sp.setter
    def max_pulse_sp(self, value):
        self._max_pulse_sp = self.set_attr_int(self._max_pulse_sp, 'max_pulse_sp', value)

    @property
    def mid_pulse_sp(self):
        """
        Used to set the pulse size in milliseconds for the signal that tells the
        servo to drive to the mid position_sp. Default value is 1500. Valid
        values are 1300 to 1700. For example, on a 180 degree servo, this would be
        90 degrees. On continuous rotation servo, this is the 'neutral' position_sp
        where the motor does not turn. You must write to the position_sp attribute for
        changes to this attribute to take effect.
        """
        self._mid_pulse_sp, value = self.get_attr_int(self._mid_pulse_sp, 'mid_pulse_sp')
        return value

    @mid_pulse_sp.setter
    def mid_pulse_sp(self, value):
        self._mid_pulse_sp = self.set_attr_int(self._mid_pulse_sp, 'mid_pulse_sp', value)

    @property
    def min_pulse_sp(self):
        """
        Used to set the pulse size in milliseconds for the signal that tells the
        servo to drive to the miniumum (counter-clockwise) position_sp. Default value
        is 600. Valid values are 300 to 700. You must write to the position_sp
        attribute for changes to this attribute to take effect.
        """
        self._min_pulse_sp, value = self.get_attr_int(self._min_pulse_sp, 'min_pulse_sp')
        return value

    @min_pulse_sp.setter
    def min_pulse_sp(self, value):
        self._min_pulse_sp = self.set_attr_int(self._min_pulse_sp, 'min_pulse_sp', value)

    @property
    def polarity(self):
        """
        Sets the polarity of the servo. Valid values are `normal` and `inversed`.
        Setting the value to `inversed` will cause the position_sp value to be
        inversed. i.e `-100` will correspond to `max_pulse_sp`, and `100` will
        correspond to `min_pulse_sp`.
        """
        self._polarity, value = self.get_attr_string(self._polarity, 'polarity')
        return value

    @polarity.setter
    def polarity(self, value):
        self._polarity = self.set_attr_string(self._polarity, 'polarity', value)

    @property
    def position_sp(self):
        """
        Reading returns the current position_sp of the servo. Writing instructs the
        servo to move to the specified position_sp. Units are percent. Valid values
        are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,
        `0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.
        """
        self._position_sp, value = self.get_attr_int(self._position_sp, 'position_sp')
        return value

    @position_sp.setter
    def position_sp(self, value):
        self._position_sp = self.set_attr_int(self._position_sp, 'position_sp', value)

    @property
    def rate_sp(self):
        """
        Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full
        range of the servo). Units are in milliseconds. Example: Setting the rate_sp
        to 1000 means that it will take a 180 degree servo 2 second to move from 0
        to 180 degrees. Note: Some servo controllers may not support this in which
        case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation
        servos, this value will affect the rate_sp at which the speed ramps up or down.
        """
        self._rate_sp, value = self.get_attr_int(self._rate_sp, 'rate_sp')
        return value

    @rate_sp.setter
    def rate_sp(self, value):
        self._rate_sp = self.set_attr_int(self._rate_sp, 'rate_sp', value)

    @property
    def state(self):
        """
        Returns a list of flags indicating the state of the servo.
        Possible values are:
        * `running`: Indicates that the motor is powered.
        """
        self._state, value = self.get_attr_set(self._state, 'state')
        return value

    # ~autogen
    # ~autogen generic-property-value classes.servoMotor>currentClass

    # Drive servo to the position set in the `position_sp` attribute.
    COMMAND_RUN = 'run'

    # Remove power from the motor.
    COMMAND_FLOAT = 'float'

    # With `normal` polarity, a positive duty cycle will
    # cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    # With `inversed` polarity, a positive duty cycle will
    # cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    # ~autogen
    # ~autogen motor_commands classes.servoMotor>currentClass

    def run(self, **kwargs):
        """Drive servo to the position set in the `position_sp` attribute.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN

    def float(self, **kwargs):
        """Remove power from the motor.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_FLOAT


# ~autogen
# ~autogen generic-class classes.sensor>currentClass

class Sensor(Device):
    """
    The sensor class provides a uniform interface for using most of the
    sensors available for the EV3. The various underlying device drivers will
    create a `lego-sensor` device for interacting with the sensors.

    Sensors are primarily controlled by setting the `mode` and monitored by
    reading the `value<N>` attributes. Values can be converted to floating point
    if needed by `value<N>` / 10.0 ^ `decimals`.

    Since the name of the `sensor<N>` device node does not correspond to the port
    that a sensor is plugged in to, you must look at the `address` attribute if
    you need to know which port a sensor is plugged in to. However, if you don't
    have more than one sensor of each type, you can just look for a matching
    `driver_name`. Then it will not matter which port a sensor is plugged in to - your
    program will still work.
    """

    SYSTEM_CLASS_NAME = 'lego-sensor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if address is not None:
            kwargs['address'] = address
        super(Sensor, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._command = None
        self._commands = None
        self._decimals = None
        self._driver_name = None
        self._mode = None
        self._modes = None
        self._num_values = None
        self._units = None

        self._sensor = None
        self._threads = None

        # ~autogen

        self._value = [None, None, None, None, None, None, None, None]

        self._bin_data_format = None
        self._bin_data_size = None
        self._bin_data = None

    __slots__ = [
        # ~autogen generic-class-slots classes.sensor>currentClass

        '_address',
        '_command',
        '_commands',
        '_decimals',
        '_driver_name',
        '_mode',
        '_modes',
        '_num_values',
        '_units',
        '_sensor',
        '_threads',

        # ~autogen
        '_value',
        '_bin_data_format',
        '_bin_data_size',
        '_bin_data',
    ]

    # ~autogen generic-get-set classes.sensor>currentClass

    @property
    def address(self):
        """
        Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
        I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
        """
        return self._address

    @property
    def command(self):
        """
        Sends a command to the sensor.
        """
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self._command = value

    @property
    def commands(self):
        """
        Returns a list of the valid commands for the sensor.
        Returns -EOPNOTSUPP if no commands are supported.
        """
        return self._commands

    @property
    def decimals(self):
        """
        Returns the number of decimal places for the values in the `value<N>`
        attributes of the current mode.
        """
        return self._decimals

    @property
    def driver_name(self):
        """
        Returns the name of the sensor device/driver. See the list of [supported
        sensors] for a complete list of drivers.
        """
        return self._driver_name

    @property
    def mode(self):
        """
        Returns the current mode. Writing one of the values returned by `modes`
        sets the sensor to that mode.
        """
        return self._mode

    @mode.setter
    def mode(self, value):
        self._mode = value

    @property
    def modes(self):
        """
        Returns a list of the valid modes for the sensor.
        """
        return self._modes

    @property
    def num_values(self):
        """
        Returns the number of `value<N>` attributes that will return a valid value
        for the current mode.
        """
        return self._num_values

    @property
    def units(self):
        """
        Returns the units of the measured value for the current mode. May return
        empty string
        """
        return self._units

    # ~autogen

    def value(self, n=0):
        """
        Returns the value or values measured by the sensor. Check num_values to
        see how many values there are. Values with N >= num_values will return
        an error. The values are fixed point numbers, so check decimals to see
        if you need to divide to get the actual value.
        """
        #        if isinstance(n, numbers.Integral):
        #           n = '{0:d}'.format(n)
        #       elif isinstance(n, numbers.Real):
        if isinstance(n, numbers.Real):
            #           n = '{0:.0f}'.format(n)
            n = int(n)
        elif isinstance(n, str):
            n = int(n)

        return self._value[n]

    @property
    def bin_data_format(self):
        """
        Returns the format of the values in `bin_data` for the current mode.
        Possible values are:

        - `u8`: Unsigned 8-bit integer (byte)
        - `s8`: Signed 8-bit integer (sbyte)
        - `u16`: Unsigned 16-bit integer (ushort)
        - `s16`: Signed 16-bit integer (short)
        - `s16_be`: Signed 16-bit integer, big endian
        - `s32`: Signed 32-bit integer (int)
        - `float`: IEEE 754 32-bit floating point (float)
        """
        return self._bin_data_format

    def bin_data(self, fmt=None):
        """
        Returns the unscaled raw values in the `value<N>` attributes as raw byte
        array. Use `bin_data_format`, `num_values` and the individual sensor
        documentation to determine how to interpret the data.

        Use `fmt` to unpack the raw bytes into a struct.

        Example::

            >>> from ev3dev import *
            >>> ir = InfraredSensor()
            >>> ir.value()
            28
            >>> ir.bin_data('<b')
            (28,)
        """

        if self._bin_data_size == None:
            self._bin_data_size = {
                                      "u8": 1,
                                      "s8": 1,
                                      "u16": 2,
                                      "s16": 2,
                                      "s16_be": 2,
                                      "s32": 4,
                                      "float": 4
                                  }.get(self.bin_data_format, 1) * self.num_values

        if None == self._bin_data:
            self._bin_data = self._attribute_file_open('bin_data')

        self._bin_data.seek(0)
        raw = bytearray(self._bin_data.read(self._bin_data_size))

        if fmt is None: return raw

        return unpack(fmt, raw)


def list_sensors(name_pattern=Sensor.SYSTEM_DEVICE_NAME_CONVENTION, **kwargs):
    """
    This is a generator function that enumerates all sensors that match the
    provided arguments.

    Parameters:
	name_pattern: pattern that device name should match.
	    For example, 'sensor*'. Default value: '*'.
	keyword arguments: used for matching the corresponding device
	    attributes. For example, driver_name='lego-ev3-touch', or
	    address=['in1', 'in3']. When argument value is a list, 
        then a match against any entry of the list is enough.
    """
    class_path = abspath(Device.DEVICE_ROOT_PATH + '/' + Sensor.SYSTEM_CLASS_NAME)
    return (Sensor(name_pattern=name, name_exact=True)
            for name in list_device_names(class_path, name_pattern, **kwargs))


# ~autogen generic-class classes.i2cSensor>currentClass

class I2cSensor(Sensor):
    """
    A generic interface to control I2C-type EV3 sensors.
    """

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(I2cSensor, self).__init__(address, name_pattern, name_exact, driver_name=['nxt-i2c-sensor'], **kwargs)

        self._fw_version = None
        self._poll_ms = None

    # ~autogen
    # ~autogen generic-get-set classes.i2cSensor>currentClass

    @property
    def fw_version(self):
        """
        Returns the firmware version of the sensor if available. Currently only
        I2C/NXT sensors support this.
        """
        self._fw_version, value = self.get_attr_string(self._fw_version, 'fw_version')
        return value

    @property
    def poll_ms(self):
        """
        Returns the polling period of the sensor in milliseconds. Writing sets the
        polling period. Setting to 0 disables polling. Minimum value is hard
        coded as 50 msec. Returns -EOPNOTSUPP if changing polling is not supported.
        Currently only I2C/NXT sensors support changing the polling period.
        """
        self._poll_ms, value = self.get_attr_int(self._poll_ms, 'poll_ms')
        return value

    @poll_ms.setter
    def poll_ms(self, value):
        self._poll_ms = self.set_attr_int(self._poll_ms, 'poll_ms', value)


# ~autogen
# ~autogen special-sensors

class TouchSensor(Sensor):
    """
    Touch Sensor
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(TouchSensor, self).__init__(address, name_pattern, name_exact,
                                          driver_name=['lego-ev3-touch', 'lego-nxt-touch'], **kwargs)
        self.auto_mode = True
        self._sensor = GazTouch()
        self._threads = []

    # Button state
    MODE_TOUCH = 'TOUCH'

    def value(self, n=0):
        if self.mode == self.MODE_TOUCH:
            return self.is_pressed
        else:
            return None

    @property
    def is_pressed(self):
        """
        A boolean indicating whether the current touch sensor is being
        pressed.
        """

        if self.auto_mode:
            self.mode = self.MODE_TOUCH

        return self._sensor.button_value()


class ColorSensor(Sensor):
    """
    LEGO EV3 color sensor.
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not 'In4' and address is not 'in4':
            print('Sorry, this simulation assumes that the color sensor is plugged in to port In4')
            os._exit(1)
        super(ColorSensor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-ev3-color'], **kwargs)
        self.auto_mode = True
        self._sensor = GazColor()
        self._threads = []

    # Reflected light. Red LED on.
    MODE_COL_REFLECT = 'COL-REFLECT'

    # Ambient light. Red LEDs off.
    MODE_COL_AMBIENT = 'COL-AMBIENT'

    # Color. All LEDs rapidly cycling, appears white.
    MODE_COL_COLOR = 'COL-COLOR'

    # Raw reflected. Red LED on
    MODE_REF_RAW = 'REF-RAW'

    # Raw Color Components. All LEDs rapidly cycling, appears white.
    MODE_RGB_RAW = 'RGB-RAW'

    def value(self, n=0):
        if self.mode == self.MODE_COL_REFLECT:
            return self.reflected_light_intensity
        elif self.mode == self.MODE_COL_COLOR:
            return self.color
        elif self.mode == self.MODE_RGB_RAW:
            return self.raw
        else:
            return None

    @property
    def reflected_light_intensity(self):
        """
        Reflected light intensity as a percentage. Light on sensor is red.
        """
        if self.auto_mode:
            self.mode = self.MODE_COL_REFLECT

        # reflection = None
        # t = threading.Thread(target=self._sensor.listener_reflect)
        # # self._threads.append(t)
        # t.start()
        # return reflection
        return int('{0:g}'.format(round(self._sensor.listener_reflect() * 100, 0)))

    @property
    def ambient_light_intensity(self):
        """
        Ambient light intensity. Light on sensor is dimly lit blue.
        """

        if self.auto_mode:
            self.mode = self.MODE_COL_AMBIENT

        return self.value(0)

    @property
    def color(self):
        """
        Color detected by the sensor, categorized by overall value.
          - 0: No color - THIS
          - 1: Black - THIS
          - 2: Blue - THIS
          - 3: Green - THIS
          - 4: Yellow
          - 5: Red - THIS
          - 6: White - THIS
          - 7: Brown
        """
        if self.auto_mode:
            self.mode = self.MODE_COL_COLOR
        return self._sensor.listener_color()


    @property
    def raw(self):
        """
        Red, green, and blue components of the detected color, in the range 0-1020.
        ADDITION: I don't know why the doc says range 0-1020 as it clearly is not when you test it on the real robot.
        Actual range is 0-255.
        """
        if self.auto_mode:
            self.mode = self.MODE_RGB_RAW
        return self._sensor.listener_raw_color()

    @property
    def red(self):
        """
        Red component of the detected color, in the range 0-1020.
        ADDITION: I don't know why the doc says range 0-1020 as it clearly is not when you test it on the real robot.
        Actual range is 0-255.
        """
        if self.auto_mode:
            self.mode = self.MODE_RGB_RAW
        return self._sensor.listener_raw_color()[0]

    @property
    def green(self):
        """
        Green component of the detected color, in the range 0-1020.
        ADDITION: I don't know why the doc says range 0-1020 as it clearly is not when you test it on the real robot.
        Actual range is 0-255.
        """
        if self.auto_mode:
            self.mode = self.MODE_RGB_RAW
        return self._sensor.listener_raw_color()[1]

    @property
    def blue(self):
        """
        Blue component of the detected color, in the range 0-1020.
        ADDITION: I don't know why the doc says range 0-1020 as it clearly is not when you test it on the real robot.
        Actual range is 0-255.
        """
        if self.auto_mode:
            self.mode = self.MODE_RGB_RAW
        return self._sensor.listener_raw_color()[2]


class UltrasonicSensor(Sensor):
    """
    LEGO EV3 ultrasonic sensor.
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not 'In1' and address is not 'in1':
            print('Sorry, this simulation assumes that the ultrasonic sensor is plugged in to port In1')
            os._exit(1)
        super(UltrasonicSensor, self).__init__(address, name_pattern, name_exact,
                                               driver_name=['lego-ev3-us', 'lego-nxt-us'], **kwargs)
        self.auto_mode = True

        self._sensor = GazSonar()
        self._threads = []

    # Continuous measurement in centimeters.
    MODE_US_DIST_CM = 'US-DIST-CM'

    # Continuous measurement in inches.
    MODE_US_DIST_IN = 'US-DIST-IN'

    # Listen.
    MODE_US_LISTEN = 'US-LISTEN'

    # Single measurement in centimeters.
    MODE_US_SI_CM = 'US-SI-CM'

    # Single measurement in inches.
    MODE_US_SI_IN = 'US-SI-IN'

    def value(self, n=0):
        if self.mode == self.MODE_US_DIST_CM:
            return self.distance_centimeters
        elif self.mode == self.MODE_US_DIST_IN:
            return self.distance_inches
        else:
            return None

    @property
    def distance_centimeters(self):
        """
        Measurement of the distance detected by the sensor,
        in centimeters.
        """

        if self.auto_mode:
            self.mode = self.MODE_US_DIST_CM

        return int('{0:g}'.format(round(self._sensor.get_distance() * 1000, 0)))

    @property
    def distance_inches(self):
        """
        Measurement of the distance detected by the sensor,
        in inches.
        """

        if self.auto_mode:
            self.mode = self.MODE_US_DIST_IN

        return int('{0:g}'.format(round(self._sensor.get_distance() * 393.7, 0)))

    @property
    def other_sensor_present(self):
        """
        Value indicating whether another ultrasonic sensor could
        be heard nearby.
        """

        if self.auto_mode:
            self.mode = self.MODE_US_LISTEN

        return self.value(0)


class GyroSensor(Sensor):
    """
    LEGO EV3 gyro sensor.
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not 'In2' and address is not 'in2':
            print('Sorry, this simulation assumes that the gyro sensor is plugged in to port In2')
            os._exit(1)
        super(GyroSensor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-ev3-gyro'], **kwargs)
        self.auto_mode = True

        self._sensor = GazGyro()
        self._threads = []
        self.mode = self.MODE_GYRO_ANG

    # Angle
    MODE_GYRO_ANG = 'GYRO-ANG'

    # Rotational speed
    MODE_GYRO_RATE = 'GYRO-RATE'

    # Raw sensor value
    MODE_GYRO_FAS = 'GYRO-FAS'

    # Angle and rotational speed
    MODE_GYRO_G_A = 'GYRO-G&A'

    # Calibration ???
    MODE_GYRO_CAL = 'GYRO-CAL'

    def value(self, n=0):
        if self.mode == self.MODE_GYRO_ANG:
            return self.angle
        elif self.mode == self.MODE_GYRO_RATE:
            return self.rate
        elif self.mode == self.MODE_GYRO_G_A:
            return self.rate_and_angle
        else:
            return None

    @property
    def angle(self):
        """
        The number of degrees that the sensor has been rotated
        since it was put into this mode.
        """

        if self.mode != self.MODE_GYRO_ANG:
            self._sensor.reset_gyro()

        if self.auto_mode:
            self.mode = self.MODE_GYRO_ANG

        return int('{0:g}'.format(round(self._sensor.return_yaw(), 0)))

    @property
    def rate(self):
        """
        The rate at which the sensor is rotating, in degrees/second.
        """
        if self.mode != self.MODE_GYRO_RATE:
            self._sensor.reset_gyro()
        if self.auto_mode:
            self.mode = self.MODE_GYRO_RATE

        return int('{0:g}'.format(round(self._sensor.get_rate(), 0)))

    @property
    def rate_and_angle(self):
        """
        Angle (degrees) and Rotational Speed (degrees/second).
        """
        if self.mode != self.MODE_GYRO_G_A:
            self._sensor.reset_gyro()
        if self.auto_mode:
            self.mode = self.MODE_GYRO_G_A

        return int('{0:g}'.format(round(self._sensor.return_yaw(), 0))), int('{0:g}'.format(round(self._sensor.get_rate(), 0)))


class InfraredSensor(Sensor):
    """
    LEGO EV3 infrared sensor.
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(InfraredSensor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-ev3-ir'], **kwargs)
        self.auto_mode = True

    # Proximity
    MODE_IR_PROX = 'IR-PROX'

    # IR Seeker
    MODE_IR_SEEK = 'IR-SEEK'

    # IR Remote Control
    MODE_IR_REMOTE = 'IR-REMOTE'

    # IR Remote Control. State of the buttons is coded in binary
    MODE_IR_REM_A = 'IR-REM-A'

    # Calibration ???
    MODE_IR_CAL = 'IR-CAL'

    @property
    def proximity(self):
        """
        A measurement of the distance between the sensor and the remote,
        as a percentage. 100% is approximately 70cm/27in.
        """

        if self.auto_mode:
            self.mode = self.MODE_IR_PROX

        return self.value(0)


class SoundSensor(Sensor):
    """
    LEGO NXT Sound Sensor
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(SoundSensor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-nxt-sound'], **kwargs)
        self.auto_mode = True

    # Sound pressure level. Flat weighting
    MODE_DB = 'DB'

    # Sound pressure level. A weighting
    MODE_DBA = 'DBA'

    @property
    def sound_pressure(self):
        """
        A measurement of the measured sound pressure level, as a
        percent. Uses a flat weighting.
        """

        if self.auto_mode:
            self.mode = self.MODE_DB

        return self.value(0)

    @property
    def sound_pressure_low(self):
        """
        A measurement of the measured sound pressure level, as a
        percent. Uses A-weighting, which focuses on levels up to 55 dB.
        """

        if self.auto_mode:
            self.mode = self.MODE_DBA

        return self.value(0)


class LightSensor(Sensor):
    """
    LEGO NXT Light Sensor
    """

    __slots__ = ['auto_mode']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(LightSensor, self).__init__(address, name_pattern, name_exact, driver_name=['lego-nxt-light'], **kwargs)
        self.auto_mode = True

    # Reflected light. LED on
    MODE_REFLECT = 'REFLECT'

    # Ambient light. LED off
    MODE_AMBIENT = 'AMBIENT'

    @property
    def reflected_light_intensity(self):
        """
        A measurement of the reflected light intensity, as a percentage.
        """

        if self.auto_mode:
            self.mode = self.MODE_REFLECT

        return self.value(0)

    @property
    def ambient_light_intensity(self):
        """
        A measurement of the ambient light intensity, as a percentage.
        """

        if self.auto_mode:
            self.mode = self.MODE_AMBIENT

        return self.value(0)


# ~autogen

# ~autogen generic-class classes.led>currentClass

class Led(Device):
    """
    Any device controlled by the generic LED driver.
    See https://www.kernel.org/doc/Documentation/leds/leds-class.txt
    for more details.
    """

    SYSTEM_CLASS_NAME = 'leds'
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if address is not None:
            kwargs['address'] = address
        super(Led, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._max_brightness = None
        self._brightness = None
        self._triggers = None
        self._trigger = None
        self._delay_on = None
        self._delay_off = None

    # ~autogen

    __slots__ = [
        # ~autogen generic-class-slots classes.led>currentClass

        '_max_brightness',
        '_brightness',
        '_triggers',
        '_trigger',
        '_delay_on',
        '_delay_off',

        # ~autogen
    ]

    # ~autogen generic-get-set classes.led>currentClass

    @property
    def max_brightness(self):
        """
        Returns the maximum allowable brightness value.
        """
        self._max_brightness, value = self.get_attr_int(self._max_brightness, 'max_brightness')
        return value

    @property
    def brightness(self):
        """
        Sets the brightness level. Possible values are from 0 to `max_brightness`.
        """
        self._brightness, value = self.get_attr_int(self._brightness, 'brightness')
        return value

    @brightness.setter
    def brightness(self, value):
        self._brightness = self.set_attr_int(self._brightness, 'brightness', value)

    @property
    def triggers(self):
        """
        Returns a list of available triggers.
        """
        self._triggers, value = self.get_attr_set(self._triggers, 'trigger')
        return value

    # ~autogen

    @property
    def trigger(self):
        """
        Sets the led trigger. A trigger
        is a kernel based source of led events. Triggers can either be simple or
        complex. A simple trigger isn't configurable and is designed to slot into
        existing subsystems with minimal additional code. Examples are the `ide-disk` and
        `nand-disk` triggers.

        Complex triggers whilst available to all LEDs have LED specific
        parameters and work on a per LED basis. The `timer` trigger is an example.
        The `timer` trigger will periodically change the LED brightness between
        0 and the current brightness setting. The `on` and `off` time can
        be specified via `delay_{on,off}` attributes in milliseconds.
        You can change the brightness value of a LED independently of the timer
        trigger. However, if you set the brightness value to 0 it will
        also disable the `timer` trigger.
        """
        self._trigger, value = self.get_attr_from_set(self._trigger, 'trigger')
        return value

    @trigger.setter
    def trigger(self, value):
        self._trigger = self.set_attr_string(self._trigger, 'trigger', value)

        # Workaround for ev3dev/ev3dev#225.
        # When trigger is set to 'timer', we need to wait for 'delay_on' and
        # 'delay_off' attributes to appear with correct permissions.
        if value == 'timer':
            for attr in ('delay_on', 'delay_off'):
                path = self._path + '/' + attr

                # Make sure the file has been created:
                for _ in range(5):
                    if os.path.exists(path):
                        break
                    time.sleep(0.2)
                else:
                    raise Exception('"{}" attribute has not been created'.format(attr))

                # Make sure the file has correct permissions:
                for _ in range(5):
                    mode = stat.S_IMODE(os.stat(path)[stat.ST_MODE])
                    if mode & stat.S_IRGRP and mode & stat.S_IWGRP:
                        break
                    time.sleep(0.2)
                else:
                    raise Exception('"{}" attribute has wrong permissions'.format(attr))

    @property
    def delay_on(self):
        """
        The `timer` trigger will periodically change the LED brightness between
        0 and the current brightness setting. The `on` time can
        be specified via `delay_on` attribute in milliseconds.
        """

        # Workaround for ev3dev/ev3dev#225.
        # 'delay_on' and 'delay_off' attributes are created when trigger is set
        # to 'timer', and destroyed when it is set to anything else.
        # This means the file cache may become outdated, and we may have to
        # reopen the file.
        for retry in (True, False):
            try:
                self._delay_on, value = self.get_attr_int(self._delay_on, 'delay_on')
                return value
            except OSError:
                if retry:
                    self._delay_on = None
                else:
                    raise

    @delay_on.setter
    def delay_on(self, value):
        # Workaround for ev3dev/ev3dev#225.
        # 'delay_on' and 'delay_off' attributes are created when trigger is set
        # to 'timer', and destroyed when it is set to anything else.
        # This means the file cache may become outdated, and we may have to
        # reopen the file.
        for retry in (True, False):
            try:
                self._delay_on = self.set_attr_int(self._delay_on, 'delay_on', value)
                return
            except OSError:
                if retry:
                    self._delay_on = None
                else:
                    raise

    @property
    def delay_off(self):
        """
        The `timer` trigger will periodically change the LED brightness between
        0 and the current brightness setting. The `off` time can
        be specified via `delay_off` attribute in milliseconds.
        """

        # Workaround for ev3dev/ev3dev#225.
        # 'delay_on' and 'delay_off' attributes are created when trigger is set
        # to 'timer', and destroyed when it is set to anything else.
        # This means the file cache may become outdated, and we may have to
        # reopen the file.
        for retry in (True, False):
            try:
                self._delay_off, value = self.get_attr_int(self._delay_off, 'delay_off')
                return value
            except OSError:
                if retry:
                    self._delay_off = None
                else:
                    raise

    @delay_off.setter
    def delay_off(self, value):
        # Workaround for ev3dev/ev3dev#225.
        # 'delay_on' and 'delay_off' attributes are created when trigger is set
        # to 'timer', and destroyed when it is set to anything else.
        # This means the file cache may become outdated, and we may have to
        # reopen the file.
        for retry in (True, False):
            try:
                self._delay_off = self.set_attr_int(self._delay_off, 'delay_off', value)
                return
            except OSError:
                if retry:
                    self._delay_off = None
                else:
                    raise

    @property
    def brightness_pct(self):
        """
        Returns led brightness as a fraction of max_brightness
        """
        return float(self.brightness) / self.max_brightness

    @brightness_pct.setter
    def brightness_pct(self, value):
        self.brightness = value * self.max_brightness


class ButtonBase(object):
    """
    Abstract button interface.
    """

    def __str__(self):
        return self.__class__.__name__

    @staticmethod
    def on_change(changed_buttons):
        """
        This handler is called by `process()` whenever state of any button has
        changed since last `process()` call. `changed_buttons` is a list of
        tuples of changed button names and their states.
        """
        pass

    _state = set([])

    def any(self):
        """
        Checks if any button is pressed.
        """
        return False

    def check_buttons(self, buttons=[]):
        """
        Check if currently pressed buttons exactly match the given list.
        """
        return set(self.buttons_pressed) == set(buttons)

    def process(self):
        """
        Check for currenly pressed buttons. If the new state differs from the
        old state, call the appropriate button event handlers.
        """
        new_state = set(self.buttons_pressed)
        old_state = self._state
        self._state = new_state

        state_diff = new_state.symmetric_difference(old_state)
        for button in state_diff:
            handler = getattr(self, 'on_' + button)
            if handler is not None: handler(button in new_state)

        if self.on_change is not None and state_diff:
            self.on_change([(button, button in new_state) for button in state_diff])

    @property
    def buttons_pressed(self):
        raise NotImplementedError()


class ButtonEVIO(ButtonBase):
    """
    Provides a generic button reading mechanism that works with event interface
    and may be adapted to platform specific implementations.

    This implementation depends on the availability of the EVIOCGKEY ioctl
    to be able to read the button state buffer. See Linux kernel source
    in /include/uapi/linux/input.h for details.
    """

    KEY_MAX = 0x2FF
    KEY_BUF_LEN = int((KEY_MAX + 7) / 8)
    EVIOCGKEY = (2 << (14 + 8 + 8) | KEY_BUF_LEN << (8 + 8) | ord('E') << 8 | 0x18)

    _buttons = {}

    def __init__(self):
        print("okkk")
        self._file_cache = {}
        self._buffer_cache = {}
        for b in self._buttons:
            name = self._buttons[b]['name']
            # if name not in self._file_cache:
            #     self._file_cache[name] = open(name, 'rb', 0)
            #     self._buffer_cache[name] = array.array('B', [0] * self.KEY_BUF_LEN)

    def _button_file(self, name):
        return self._file_cache[name]

    def _button_buffer(self, name):
        return self._buffer_cache[name]

    @property
    def buttons_pressed(self):
        """
        Returns list of names of pressed buttons.
        """
        for b in self._buffer_cache:
            fcntl.ioctl(self._button_file(b), self.EVIOCGKEY, self._buffer_cache[b])

        pressed = []
        for k, v in self._buttons.items():
            buf = self._buffer_cache[v['name']]
            bit = v['value']
            if bool(buf[int(bit / 8)] & 1 << bit % 8):
                pressed += [k]
        return pressed


# ~autogen remote-control specialSensorTypes.infraredSensor.remoteControl>currentClass
class RemoteControl(ButtonBase):
    """
    EV3 Remote Controller
    """

    _BUTTON_VALUES = {
        0: [],
        1: ['red_up'],
        2: ['red_down'],
        3: ['blue_up'],
        4: ['blue_down'],
        5: ['red_up', 'blue_up'],
        6: ['red_up', 'blue_down'],
        7: ['red_down', 'blue_up'],
        8: ['red_down', 'blue_down'],
        9: ['beacon'],
        10: ['red_up', 'red_down'],
        11: ['blue_up', 'blue_down']
    }

    on_red_up = None
    on_red_down = None
    on_blue_up = None
    on_blue_down = None
    on_beacon = None

    @property
    def red_up(self):
        """
        Checks if `red_up` button is pressed.
        """
        return 'red_up' in self.buttons_pressed

    @property
    def red_down(self):
        """
        Checks if `red_down` button is pressed.
        """
        return 'red_down' in self.buttons_pressed

    @property
    def blue_up(self):
        """
        Checks if `blue_up` button is pressed.
        """
        return 'blue_up' in self.buttons_pressed

    @property
    def blue_down(self):
        """
        Checks if `blue_down` button is pressed.
        """
        return 'blue_down' in self.buttons_pressed

    @property
    def beacon(self):
        """
        Checks if `beacon` button is pressed.
        """
        return 'beacon' in self.buttons_pressed

    # ~autogen

    def __init__(self, sensor=None, channel=1):
        if sensor is None:
            self._sensor = InfraredSensor()
        else:
            self._sensor = sensor

        self._channel = max(1, min(4, channel)) - 1
        self._state = set([])

        if self._sensor.connected:
            self._sensor.mode = 'IR-REMOTE'

    @property
    def connected(self):
        return self._sensor.connected

    @property
    def buttons_pressed(self):
        """
        Returns list of currently pressed buttons.
        """
        return RemoteControl._BUTTON_VALUES.get(self._sensor.value(self._channel), [])


# ~autogen generic-class classes.powerSupply>currentClass

class PowerSupply(Device):
    """
    A generic interface to read data from the system's power_supply class.
    Uses the built-in legoev3-battery if none is specified.
    """

    SYSTEM_CLASS_NAME = 'power_supply'
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not None:
            kwargs['address'] = address
        super(PowerSupply, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._measured_current = None
        self._measured_voltage = None
        self._max_voltage = None
        self._min_voltage = None
        self._technology = None
        self._type = None

    # ~autogen

    __slots__ = [
        # ~autogen generic-class-slots classes.powerSupply>currentClass

        '_measured_current',
        '_measured_voltage',
        '_max_voltage',
        '_min_voltage',
        '_technology',
        '_type',

        # ~autogen
    ]

    # ~autogen generic-get-set classes.powerSupply>currentClass

    @property
    def measured_current(self):
        """
        The measured current that the battery is supplying (in microamps)
        """
        self._measured_current, value = self.get_attr_int(self._measured_current, 'current_now')
        return value

    @property
    def measured_voltage(self):
        """
        The measured voltage that the battery is supplying (in microvolts)
        """
        self._measured_voltage, value = self.get_attr_int(self._measured_voltage, 'voltage_now')
        return value

    @property
    def max_voltage(self):
        """
        """
        self._max_voltage, value = self.get_attr_int(self._max_voltage, 'voltage_max_design')
        return value

    @property
    def min_voltage(self):
        """
        """
        self._min_voltage, value = self.get_attr_int(self._min_voltage, 'voltage_min_design')
        return value

    @property
    def technology(self):
        """
        """
        self._technology, value = self.get_attr_string(self._technology, 'technology')
        return value

    @property
    def type(self):
        """
        """
        self._type, value = self.get_attr_string(self._type, 'type')
        return value

    # ~autogen

    @property
    def measured_amps(self):
        """
        The measured current that the battery is supplying (in amps)
        """
        return self.measured_current / 1e6

    @property
    def measured_volts(self):
        """
        The measured voltage that the battery is supplying (in volts)
        """
        return self.measured_voltage / 1e6


# ~autogen generic-class classes.legoPort>currentClass

class LegoPort(Device):
    """
    The `lego-port` class provides an interface for working with input and
    output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
    WeDo and LEGO Power Functions sensors and motors. Supported devices include
    the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
    various sensor multiplexers from 3rd party manufacturers.

    Some types of ports may have multiple modes of operation. For example, the
    input ports on the EV3 brick can communicate with sensors using UART, I2C
    or analog validate signals - but not all at the same time. Therefore there
    are multiple modes available to connect to the different types of sensors.

    In most cases, ports are able to automatically detect what type of sensor
    or motor is connected. In some cases though, this must be manually specified
    using the `mode` and `set_device` attributes. The `mode` attribute affects
    how the port communicates with the connected device. For example the input
    ports on the EV3 brick can communicate using UART, I2C or analog voltages,
    but not all at the same time, so the mode must be set to the one that is
    appropriate for the connected sensor. The `set_device` attribute is used to
    specify the exact type of sensor that is connected. Note: the mode must be
    correctly set before setting the sensor type.

    Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
    incremented each time a new port is registered. Note: The number is not
    related to the actual port at all - use the `address` attribute to find
    a specific port.
    """

    SYSTEM_CLASS_NAME = 'lego-port'
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        if address is not None:
            kwargs['address'] = address
        super(LegoPort, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._driver_name = None
        self._modes = None
        self._mode = None
        self._set_device = None
        self._status = None

    # ~autogen

    __slots__ = [
        # ~autogen generic-class-slots classes.legoPort>currentClass

        '_address',
        '_driver_name',
        '_modes',
        '_mode',
        '_set_device',
        '_status',

        # ~autogen
    ]

    # ~autogen generic-get-set classes.legoPort>currentClass

    @property
    def address(self):
        """
        Returns the name of the port. See individual driver documentation for
        the name that will be returned.
        """
        self._address, value = self.get_attr_string(self._address, 'address')
        return value

    @property
    def driver_name(self):
        """
        Returns the name of the driver that loaded this device. You can find the
        complete list of drivers in the [list of port drivers].
        """
        self._driver_name, value = self.get_attr_string(self._driver_name, 'driver_name')
        return value

    @property
    def modes(self):
        """
        Returns a list of the available modes of the port.
        """
        self._modes, value = self.get_attr_set(self._modes, 'modes')
        return value

    @property
    def mode(self):
        """
        Reading returns the currently selected mode. Writing sets the mode.
        Generally speaking when the mode changes any sensor or motor devices
        associated with the port will be removed new ones loaded, however this
        this will depend on the individual driver implementing this class.
        """
        self._mode, value = self.get_attr_string(self._mode, 'mode')
        return value

    @mode.setter
    def mode(self, value):
        self._mode = self.set_attr_string(self._mode, 'mode', value)

    @property
    def set_device(self):
        """
        For modes that support it, writing the name of a driver will cause a new
        device to be registered for that driver and attached to this port. For
        example, since NXT/Analog sensors cannot be auto-detected, you must use
        this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
        device is not supported.
        """
        raise Exception("set_device is a write-only property!")

    @set_device.setter
    def set_device(self, value):
        self._set_device = self.set_attr_string(self._set_device, 'set_device', value)

    @property
    def status(self):
        """
        In most cases, reading status will return the same value as `mode`. In
        cases where there is an `auto` mode additional values may be returned,
        such as `no-device` or `error`. See individual port driver documentation
        for the full list of possible values.
        """
        self._status, value = self.get_attr_string(self._status, 'status')
        return value


# ~autogen

class FbMem(object):
    """The framebuffer memory object.

    Made of:
        - the framebuffer file descriptor
        - the fix screen info struct
        - the var screen info struct
        - the mapped memory
    """

    # ------------------------------------------------------------------
    # The code is adapted from
    # https://github.com/LinkCareServices/cairotft/blob/master/cairotft/linuxfb.py
    #
    # The original code came with the following license:
    # ------------------------------------------------------------------
    # Copyright (c) 2012 Kurichan
    #
    # This program is free software. It comes without any warranty, to
    # the extent permitted by applicable law. You can redistribute it
    # and/or modify it under the terms of the Do What The Fuck You Want
    # To Public License, Version 2, as published by Sam Hocevar. See
    # http://sam.zoy.org/wtfpl/COPYING for more details.
    # ------------------------------------------------------------------

    __slots__ = ('fid', 'fix_info', 'var_info', 'mmap')

    FBIOGET_VSCREENINFO = 0x4600
    FBIOGET_FSCREENINFO = 0x4602

    FB_VISUAL_MONO01 = 0
    FB_VISUAL_MONO10 = 1

    class FixScreenInfo(ctypes.Structure):
        """The fb_fix_screeninfo from fb.h."""

        _fields_ = [
            ('id_name', ctypes.c_char * 16),
            ('smem_start', ctypes.c_ulong),
            ('smem_len', ctypes.c_uint32),
            ('type', ctypes.c_uint32),
            ('type_aux', ctypes.c_uint32),
            ('visual', ctypes.c_uint32),
            ('xpanstep', ctypes.c_uint16),
            ('ypanstep', ctypes.c_uint16),
            ('ywrapstep', ctypes.c_uint16),
            ('line_length', ctypes.c_uint32),
            ('mmio_start', ctypes.c_ulong),
            ('mmio_len', ctypes.c_uint32),
            ('accel', ctypes.c_uint32),
            ('reserved', ctypes.c_uint16 * 3),
        ]

    class VarScreenInfo(ctypes.Structure):
        class FbBitField(ctypes.Structure):
            """The fb_bitfield struct from fb.h."""

            _fields_ = [
                ('offset', ctypes.c_uint32),
                ('length', ctypes.c_uint32),
                ('msb_right', ctypes.c_uint32),
            ]

        """The fb_var_screeninfo struct from fb.h."""

        _fields_ = [
            ('xres', ctypes.c_uint32),
            ('yres', ctypes.c_uint32),
            ('xres_virtual', ctypes.c_uint32),
            ('yres_virtual', ctypes.c_uint32),
            ('xoffset', ctypes.c_uint32),
            ('yoffset', ctypes.c_uint32),

            ('bits_per_pixel', ctypes.c_uint32),
            ('grayscale', ctypes.c_uint32),

            ('red', FbBitField),
            ('green', FbBitField),
            ('blue', FbBitField),
            ('transp', FbBitField),
        ]

    def __init__(self, fbdev=None):
        """Create the FbMem framebuffer memory object."""
        fid = FbMem._open_fbdev(fbdev)
        fix_info = FbMem._get_fix_info(fid)
        fbmmap = FbMem._map_fb_memory(fid, fix_info)
        self.fid = fid
        self.fix_info = fix_info
        self.var_info = FbMem._get_var_info(fid)
        self.mmap = fbmmap

    def __del__(self):
        """Close the FbMem framebuffer memory object."""
        self.mmap.close()
        FbMem._close_fbdev(self.fid)

    @staticmethod
    def _open_fbdev(fbdev=None):
        """Return the framebuffer file descriptor.

        Try to use the FRAMEBUFFER
        environment variable if fbdev is not given. Use '/dev/fb0' by
        default.
        """
        dev = fbdev or os.getenv('FRAMEBUFFER', '/dev/fb0')
        fbfid = os.open(dev, os.O_RDWR)
        return fbfid

    @staticmethod
    def _close_fbdev(fbfid):
        """Close the framebuffer file descriptor."""
        os.close(fbfid)

    @staticmethod
    def _get_fix_info(fbfid):
        """Return the fix screen info from the framebuffer file descriptor."""
        fix_info = FbMem.FixScreenInfo()
        fcntl.ioctl(fbfid, FbMem.FBIOGET_FSCREENINFO, fix_info)
        return fix_info

    @staticmethod
    def _get_var_info(fbfid):
        """Return the var screen info from the framebuffer file descriptor."""
        var_info = FbMem.VarScreenInfo()
        fcntl.ioctl(fbfid, FbMem.FBIOGET_VSCREENINFO, var_info)
        return var_info

    @staticmethod
    def _map_fb_memory(fbfid, fix_info):
        """Map the framebuffer memory."""
        return mmap.mmap(
            fbfid,
            fix_info.smem_len,
            mmap.MAP_SHARED,
            mmap.PROT_READ | mmap.PROT_WRITE,
            offset=0
        )


class Screen(FbMem):
    """
    A convenience wrapper for the FbMem class.
    Provides drawing functions from the python imaging library (PIL).
    """

    def __init__(self):
        from PIL import Image, ImageDraw
        FbMem.__init__(self)

        self._img = Image.new(
            self.var_info.bits_per_pixel == 1 and "1" or "RGB",
            (self.fix_info.line_length * 8 // self.var_info.bits_per_pixel, self.yres),
            "white")

        self._draw = ImageDraw.Draw(self._img)

    @property
    def xres(self):
        """
        Horizontal screen resolution
        """
        return self.var_info.xres

    @property
    def yres(self):
        """
        Vertical screen resolution
        """
        return self.var_info.yres

    @property
    def shape(self):
        """
        Dimensions of the screen.
        """
        return (self.xres, self.yres)

    @property
    def draw(self):
        """
        Returns a handle to PIL.ImageDraw.Draw class associated with the screen.

        Example::

            screen.draw.rectangle((10,10,60,20), fill='black')
        """
        return self._draw

    @property
    def image(self):
        """
        Returns a handle to PIL.Image class that is backing the screen. This can
        be accessed for blitting images to the screen.

        Example::

            screen.image.paste(picture, (0, 0))
        """
        return self._img

    def clear(self):
        """
        Clears the screen
        """
        self._draw.rectangle(((0, 0), self.shape), fill="white")

    def _color565(self, r, g, b):
        """Convert red, green, blue components to a 16-bit 565 RGB value. Components
        should be values 0 to 255.
        """
        return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

    def _img_to_rgb565_bytes(self):
        pixels = [self._color565(r, g, b) for (r, g, b) in self._img.getdata()]
        return pack('H' * len(pixels), *pixels)

    def update(self):
        """
        Applies pending changes to the screen.
        Nothing will be drawn on the screen until this function is called.
        """
        if self.var_info.bits_per_pixel == 1:
            self.mmap[:] = self._img.tobytes("raw", "1;IR")
        elif self.var_info.bits_per_pixel == 16:
            self.mmap[:] = self._img_to_rgb565_bytes()
        else:
            raise Exception("Not supported")


class Sound:
    """
    Sound-related functions. The class has only static methods and is not
    intended for instantiation. It can beep, play wav files, or convert text to
    speech.

    Note that all methods of the class spawn system processes and return
    subprocess.Popen objects. The methods are asynchronous (they return
    immediately after child process was spawned, without waiting for its
    completion), but you can call wait() on the returned result.

    Examples::

        # Play 'bark.wav', return immediately:
        Sound.play('bark.wav')

        # Introduce yourself, wait for completion:
        Sound.speak('Hello, I am Robot').wait()

    """

    channel = None

    @staticmethod
    def beep(args=''):
        """
        Call beep command with the provided arguments (if any).
        See `beep man page`_ and google `linux beep music`_ for inspiration.

        .. _`beep man page`: https://linux.die.net/man/1/beep
        .. _`linux beep music`: https://www.google.com/search?q=linux+beep+music
        """
        with open(os.devnull, 'w') as n:
            return Popen('/usr/bin/beep %s' % args, stdout=n, shell=True)

    @staticmethod
    def tone(*args):
        """
        .. rubric:: tone(tone_sequence)

        Play tone sequence. The tone_sequence parameter is a list of tuples,
        where each tuple contains up to three numbers. The first number is
        frequency in Hz, the second is duration in milliseconds, and the third
        is delay in milliseconds between this and the next tone in the
        sequence.

        Here is a cheerful example::

            Sound.tone([
                (392, 350, 100), (392, 350, 100), (392, 350, 100), (311.1, 250, 100),
                (466.2, 25, 100), (392, 350, 100), (311.1, 250, 100), (466.2, 25, 100),
                (392, 700, 100), (587.32, 350, 100), (587.32, 350, 100),
                (587.32, 350, 100), (622.26, 250, 100), (466.2, 25, 100),
                (369.99, 350, 100), (311.1, 250, 100), (466.2, 25, 100), (392, 700, 100),
                (784, 350, 100), (392, 250, 100), (392, 25, 100), (784, 350, 100),
                (739.98, 250, 100), (698.46, 25, 100), (659.26, 25, 100),
                (622.26, 25, 100), (659.26, 50, 400), (415.3, 25, 200), (554.36, 350, 100),
                (523.25, 250, 100), (493.88, 25, 100), (466.16, 25, 100), (440, 25, 100),
                (466.16, 50, 400), (311.13, 25, 200), (369.99, 350, 100),
                (311.13, 250, 100), (392, 25, 100), (466.16, 350, 100), (392, 250, 100),
                (466.16, 25, 100), (587.32, 700, 100), (784, 350, 100), (392, 250, 100),
                (392, 25, 100), (784, 350, 100), (739.98, 250, 100), (698.46, 25, 100),
                (659.26, 25, 100), (622.26, 25, 100), (659.26, 50, 400), (415.3, 25, 200),
                (554.36, 350, 100), (523.25, 250, 100), (493.88, 25, 100),
                (466.16, 25, 100), (440, 25, 100), (466.16, 50, 400), (311.13, 25, 200),
                (392, 350, 100), (311.13, 250, 100), (466.16, 25, 100),
                (392.00, 300, 150), (311.13, 250, 100), (466.16, 25, 100), (392, 700)
                ]).wait()

        .. rubric:: tone(frequency, duration)

        Play single tone of given frequency (Hz) and duration (milliseconds).
        """

        def play_tone_sequence(tone_sequence):
            def beep_args(frequency=None, duration=None, delay=None):
                args = ''
                if frequency is not None: args += '-f %s ' % frequency
                if duration is not None: args += '-l %s ' % duration
                if delay is not None: args += '-D %s ' % delay

                return args

            return Sound.beep(' -n '.join([beep_args(*t) for t in tone_sequence]))

        if len(args) == 1:
            return play_tone_sequence(args[0])
        elif len(args) == 2:
            return play_tone_sequence([(args[0], args[1])])
        else:
            raise Exception("Unsupported number of parameters in Sound.tone()")

    @staticmethod
    def play(wav_file):
        """
        Play wav file.
        """
        with open(os.devnull, 'w') as n:
            return Popen('/usr/bin/aplay -q "%s"' % wav_file, stdout=n, shell=True)

    @staticmethod
    def speak(text, espeak_opts='-a 200 -s 130'):
        """
        Speak the given text aloud.
        """
        with open(os.devnull, 'w') as n:
            cmd_line = '/usr/bin/espeak --stdout {0} "{1}" | /usr/bin/aplay -q'.format(espeak_opts, text)
            return Popen(cmd_line, stdout=n, shell=True)

    @staticmethod
    def _get_channel():
        if Sound.channel is None:
            # Get default channel as the first one that pops up in
            # 'amixer scontrols' output, which contains strings in the
            # following format:
            #
            #     Simple mixer control 'Master',0
            #     Simple mixer control 'Capture',0
            out = check_output(['amixer', 'scontrols']).decode()
            m = re.search("'(?P<channel>[^']+)'", out)
            if m:
                Sound.channel = m.group('channel')
            else:
                Sound.channel = 'Playback'

        return Sound.channel

    @staticmethod
    def set_volume(pct, channel=None):
        """
        Sets the sound volume to the given percentage [0-100] by calling
        ``amixer -q set <channel> <pct>%``.
        If the channel is not specified, it tries to determine the default one
        by running ``amixer scontrols``. If that fails as well, it uses the
        ``Playback`` channel, as that is the only channel on the EV3.
        """

        if channel is None:
            channel = Sound._get_channel()

        cmd_line = '/usr/bin/amixer -q set {0} {1:d}%'.format(channel, pct)
        Popen(cmd_line, shell=True).wait()

    @staticmethod
    def get_volume(channel=None):
        """
        Gets the current sound volume by parsing the output of
        ``amixer get <channel>``.
        If the channel is not specified, it tries to determine the default one
        by running ``amixer scontrols``. If that fails as well, it uses the
        ``Playback`` channel, as that is the only channel on the EV3.
        """

        if channel is None:
            channel = Sound._get_channel()

        out = check_output(['amixer', 'get', channel]).decode()
        m = re.search('\[(?P<volume>\d+)%\]', out)
        if m:
            return int(m.group('volume'))
        else:
            raise Exception('Failed to parse output of `amixer get {}`'.format(channel))
