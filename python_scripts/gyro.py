import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

m = ev3.LargeMotor('outA')
c = ev3.GyroSensor('In1')

m.speed_sp = 360

m.run_forever()

while True:
    print(c.rate)
    time.sleep(1)
    

m.stop()
