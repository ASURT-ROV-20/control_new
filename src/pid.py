#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import pid_controller
import Adafruit_PCA9685
import ms5837
import threading
import time

sensor = ms5837.MS5837_30BA()
hat = Adafruit_PCA9685.PCA9685()
hat.set_pwm_freq(50)
sensor.init()

offset = 340

hat.set_pwm(12,0,340)
hat.set_pwm(13,0,340)

time.sleep(1)


rospy.init_node("depth_control")
control_effort_pub = rospy.Publisher("control_effort",Float64, queue_size=5)
setpoint_pub = rospy.Publisher("setpoint",Float64, queue_size=5)
state_pub = rospy.Publisher("state",Float64, queue_size=5)

pid_depth = pid_controller.PID(kp = 50, ki = 0, kd = 0,
                             wind_up = 20, upper_limit = 130, lower_limit = -100)

setpoint = 0
state = 0
pid_depth.enable = True

def run():
    while not rospy.is_shutdown():
        sensor.read()
        state = sensor.depth()
        pwm = pid_depth.update(setpoint = setpoint, state = state)
        pwm += offset 
        hat.set_pwm(12, 0, pwm)
        hat.set_pwm(13, 0, pwm)
        print(pwm)

        state_pub.publish(state)
        setpoint_pub.publish(setpoint)
        control_effort_pub.publish(pwm)
        time.sleep(0.09)

main_fn = threading.Thread(target=run)
main_fn.start()

while not rospy.is_shutdown():
    setpoint = float(input("Setpoint: "))
    # state = float(input("Enter the state: "))
    # state = sensor.depth()
    # sensor.read()
    
    # pwm = pid_depth.update(setpoint = setpoint, state = state)
    # pwm += offset 
    
    # print(pwm)
    # state_pub.publish(state)
    # setpoint_pub.publish(setpoint)
    # control_effort_pub.publish(pwm)
    # # hat.set_pwm(12, 0, pwm)
    # # hat.set_pwm(13, 0, pwm)
    
    # time.sleep(0.05)
