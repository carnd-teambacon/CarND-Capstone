
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self):
        # TODO: Implement
        


        pass

    def control(self, lin_vel, ang_vel, current_lin_vel, dbw_status):
        # TODO: Implement
        # probably should make a PID controller for steer and throttle
        
        # create a PID object for velocity and steer
        # create a Yaw_controller to get the steer angle

        self.Steer_angle = yaw_controller(wheel_base = wheel_base, steer_ratio = steer_ratio,
         								  min_speed = min_speed, max_lat_accel = max_lat_accel, 
         								  max_steer_angle = max_steer_angle)

        next_steer = self.Steer_angle.get_steering(lin_vel, ang_vel, current_lin_vel)

        self.Steer_PID = PID(kp = 0, ki = 0, kd = 0)    # will need to tune this
        self.Throttle_PID = PID(kp = 1, ki = 0, kd = 0) #  will need to tune this

        

        # find the velocity error and angle error for the PID controller





        # Return throttle, brake, steer
        return 1., 0., 0.
