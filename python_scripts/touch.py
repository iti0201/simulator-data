import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time


c = ev3.TouchSensor('In4')

while True:
    a = c.is_pressed
    print(a)
    time.sleep(1)