import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, cp):

        self.pid = PID(kp=1.2, ki=0.005, kd=0.5, mn=cp.decel_limit, mx=cp.accel_limit) # values?
        self.yaw_controller = YawController(
            wheel_base=cp.wheel_base,
            steer_ratio=cp.steer_ratio,
            min_speed=cp.min_speed,
            max_lat_accel=cp.max_lat_accel,
            max_steer_angle=cp.max_steer_angle)


    def reset(self):
        self.pid.reset()


    def control(self, lin_vel, ang_vel, current_lin_vel, del_time, vel_err):
        # TODO: Implement
        # probably should make a PID controller for steer and throttle

        # create a PID object for velocity and steer
        # create a Yaw_controller to get the steer angle

        acceleration = self.pid.step(vel_err, del_time)
        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_lin_vel)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            brake = -acceleration

        # Return throttle, brake, steer
        return throttle, brake, next_steer
