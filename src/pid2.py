#!/usr/bin/env python3

import rospy
from pid_controller import *

rospy.init_node("setpoint2")
pid = PID(4,5,6,20,20,20)
# pid2 = PID(4,5,6,20,20,20)


while not rospy.is_shutdown():
    print("1: ", pid.kp, " ", pid.ki, " ", pid.kd)
    # print("2: ", pid2.kp, " ", pid2.ki, " ", pid2.kd)
    time.sleep(.25)