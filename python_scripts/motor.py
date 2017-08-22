import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.LargeMotor('outA')
m1 = ev3.LargeMotor('outD')

m.speed_sp = 360
m.time_sp = 10000
m1.speed_sp = 360
m1.time_sp = 10000
m.run_timed()
m1.run_timed()
