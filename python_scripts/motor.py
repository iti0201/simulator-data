import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.LargeMotor('outA')
m1 = ev3.LargeMotor('outD')

#m.run_direct(duty_cycle_sp = -80)
speed = 400
m.run_timed(speed_sp = speed, time_sp = 10000)

#m.run_timed(speed_sp = -360, time_sp = 1000)
#m1.run_timed(speed_sp = -360, time_sp = 1000)

