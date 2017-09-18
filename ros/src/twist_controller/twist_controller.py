
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
        


        pass

    def control(self, lin_vel, ang_vel, current_lin_vel, dbw_status, del_time, vel_err):
        # TODO: Implement
        # probably should make a PID controller for steer and throttle
        
        # create a PID object for velocity and steer
        # create a Yaw_controller to get the steer angle

        self.Steer_angle = yaw_controller(wheel_base = self.wheel_base, steer_ratio = self.steer_ratio,
         								  min_speed = self.min_speed, max_lat_accel = self.max_lat_accel, 
         								  max_steer_angle = self.max_steer_angle)

        next_steer = self.Steer_angle.get_steering(lin_vel, ang_vel, current_lin_vel)

        self.Throttle_PID = PID(kp = 0.5, ki = 0, kd = 0) #  will need to tune this
        throtte = Throttle_PID.step(vel_err, del_time)

        if lin_vel.twist.twist.linear.x > 0:
        	brake = 0
        else:
        	brake = 1



        # Return throttle, brake, steer
        return throttle, brake, next_steer
