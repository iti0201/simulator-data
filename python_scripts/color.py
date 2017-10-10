import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time


c = ev3.ColorSensor('in3')

while True:
    print(c.reflected_light_intensity)


