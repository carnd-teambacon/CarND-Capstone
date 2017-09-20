import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self):
        # TODO: Implement
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.min_speed = 3

        self.yaw_controller = YawController(
            wheel_base=self.wheel_base,
            steer_ratio=self.steer_ratio,
            min_speed=self.min_speed,
            max_lat_accel=self.max_lat_accel,
            max_steer_angle=self.max_steer_angle)

        self.pid = PID(kp=0.5, ki=0.3, kd=1) #  will need to tune this


    def control(self, lin_vel, ang_vel, current_lin_vel, dbw_status, del_time, vel_err):
        # TODO: Implement
        # probably should make a PID controller for steer and throttle

        # create a PID object for velocity and steer
        # create a Yaw_controller to get the steer angle

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_lin_vel)
        acceleration = self.pid.step(vel_err, del_time)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            brake = 1.0

        # Return throttle, brake, steer
        return throttle, brake, next_steer
