import sys
import os

import ev3dev.ev3 as ev3
import rospy
import time

ev3.LargeMotor('outA').run_timed(speed_sp=400, time_sp=4000)

gyro = ev3.GyroSensor('In3')

for i in range(10):
    stuff = gyro.rate_and_angle
    print('Gyro angle = {}, Gyro rate = {}'.format(stuff[0], stuff[1]))

    time.sleep(1)
