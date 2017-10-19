import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time


c = ev3.UltrasonicSensor('in1')
# TODO FLOOR

print(c.value)



