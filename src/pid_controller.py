#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server 
from control.cfg import pid_paramConfig

class PID:

    def __init__(self, kp, ki, kd, wind_up, upper_limit, lower_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.wind_up = wind_up
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

        self.sample_time = 0.09
        self.current_time = time.time()
        self.last_time = self.current_time
        self.first = True
        self.clear()

        self.srv = Server(pid_paramConfig, self.reconfigurecallback)

    def reconfigurecallback(self, config, level):
        if self.first :
            self.first = False
            print("kp = ", self.kp, "ki = ", self.ki, "kd = ", self.kd)
            return config
        self.kp = config["Kp"] * config["Kp_scale"]
        self.ki = config["Ki"] * config["Ki_scale"]
        self.kd = config["Kd"] * config["Kd_scale"]
        print("pid paramerters changed")
        print("kp =", self.kp, " ki =", self.ki, " kd =", self.kd)
        return config

    def clear(self):
        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0
        self.last_error = 0.0

        self.enable = False
        # self.wind_up = 0.0
        self.output = 0.0
        # self.int_error = 0.0

    def update(self, setpoint, state):
        if ~self.enable :
            return 0
        self.error = setpoint - state
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = self.error - self.last_error

        if delta_time >= self.sample_time :
            self.pterm = self.kp * self.error
            self.iterm += delta_error * delta_time

            if self.iterm > self.wind_up:
                self.iterm = self.wind_up
            elif self.iterm < -self.wind_up:
                self.iterm = -self.wind_up

            self.dterm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = self.error

            self.output = self.pterm + self.ki * self.iterm + self.dterm * self.kd

            if self.output > self.upper_limit:
                self.output = self.upper_limit
            elif self.output < self.lower_limit:
                self.output = self.lower_limit

        return self.output
