#!/usr/bin/env python

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import numpy as np

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
	#  Calculate steer angle and acceleration by the Yaw and PID controller
    def __init__(self, vehicle_mass, fuel_capacity, min_speed, brake_deadband, deceleration_limit, acceleration_limit,
                 wheel_base, wheel_radius, steer_ratio, max_lat_accel, max_steer_angle):

        self.deceleration_limit = deceleration_limit
        self.velocity_controller = PID(.31, .101, 0., 0., .2)
        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.vel_lpf = LowPassFilter(0.5, 0.02)
        self.brake_deadband = brake_deadband
        self.min_speed = min_speed

    #  Get velocity and time elapsed from last call
	#  Calculate acceleration/deceleration by PID
    def control(self, twist_cmd, current_velocity, time_elapsed):
        
        # add lowpass filter to remove noise
        current_vel = self.vel_lpf.filt(current_velocity.twist.linear.x)
        vel_error = twist_cmd.twist.linear.x - current_vel
        throttle = self.velocity_controller.step(vel_error, time_elapsed)
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x, twist_cmd.twist.angular.z, current_vel)
        brake = 0
        if twist_cmd.twist.linear.x == 0 and current_vel < 0.:
            throttle = 0
            brake = 500 # inceease to 500 N/m
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.deceleration_limit)
            brake = abs(decel)*self.total_mass*self.wheel_radius
        
        return throttle, brake, steer

    def reset(self):
        self.velocity_controller.reset()
