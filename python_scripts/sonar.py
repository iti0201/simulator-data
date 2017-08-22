import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time


c = ev3.UltrasonicSensor('In2')
# TODO FLOOR

while True:
    print(c.distance_centimeters)
    print(c.distance_inches)
    time.sleep(0.1)


