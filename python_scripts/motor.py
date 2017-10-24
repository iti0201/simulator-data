import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.LargeMotor('outA')
m1 = ev3.LargeMotor('outD')

#m.run_direct(duty_cycle_sp = -80)
speed = 250
m.run_forever(speed_sp = 0)
m1.run_forever(speed_sp = speed)


#m.run_timed(speed_sp = 360, time_sp = 3000)
#m1.run_timed(speed_sp = -360, time_sp = 3000)

while True:
    print(str(m1.speed))
    rospy.sleep(0.5)
