import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.MediumMotor('outB')

m.speed_sp = 200
m.run_to_abs_pos(position_sp=0)

print('done it!')


