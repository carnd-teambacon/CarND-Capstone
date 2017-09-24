import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, cp):

        self.pid = PID(kp=5.0, ki=0.5, kd=0.5) # self.pid = PID(kp=1.2, ki=0.005, kd=0.5, mn=cp.decel_limit, mx=cp.accel_limit)
        self.yaw_controller = YawController(
            wheel_base=cp.wheel_base,
            steer_ratio=cp.steer_ratio,
            min_speed=cp.min_speed,
            max_lat_accel=cp.max_lat_accel,
            max_steer_angle=cp.max_steer_angle)

        self.s_lpf = LowPassFilter(tau=3, ts=1)
        self.t_lpf = LowPassFilter(tau=3, ts=1)

    def reset(self):
        self.pid.reset()


    def control(self, lin_vel, ang_vel, current_lin_vel, del_time, vel_err):
        # TODO: Implement
        # probably should make a PID controller for steer and throttle

        # create a PID object for velocity and steer
        # create a Yaw_controller to get the steer angle

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_lin_vel)
        next_steer = self.s_lpf.filt(next_steer)
        self.s_lpf.last_val = next_steer

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_lpf.filt(acceleration)
        self.t_lpf.last_val = acceleration

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            brake = -acceleration

        # Return throttle, brake, steer
        return throttle, brake, next_steer
