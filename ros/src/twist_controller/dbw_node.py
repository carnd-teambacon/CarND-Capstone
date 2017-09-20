#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
import math

from twist_controller import TwistController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)


        # other variables needed
        # current_pose, current_velocity, timestamp for PID

        self.current_pose = None # is there a topic for current pose? need to subscribe to it
        self.current_velocity = None # is there a topic for current velocity? need to subscribe to it
        self.previous_timestamp = rospy.get_time()
        self.current_timestamp = 0.0
        self.del_time = 0.0
        self.dbw_enabled = False
        self.latest_twist_cmd = None

        # TODO: Create `TwistController` object
        self.controller = TwistController()

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.loop()

    def dbw_enabled_cb(self, dbw_enabled):
        self.dbw_enabled = dbw_enabled

    def current_velocity_cb(self, current_velocity):
        self.current_velocity = current_velocity

    def twist_cmd_cb(self, twist_cmd):
        self.latest_twist_cmd = twist_cmd

    def loop(self):
        rate = rospy.Rate(10) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>
            #                                                     <del_time>)

            # in order to use the controller we need to the following
            # for throttle controller: we need the current speed error and the del time
            # for the steering error: we need the CTE to guide us back to the center, but it seems like the
            #   yaw controller will return a steering angle, do we still need a CTE?
            rospy.loginfo("""DBW enabled: {}""".format(self.dbw_enabled))

            if self.dbw_enabled and self.current_velocity is not None and self.latest_twist_cmd is not None:

                self.current_timestamp = rospy.get_time()
                self.del_time = self.current_timestamp - self.previous_timestamp
                self.previous_timestamp = self.current_timestamp
                vel_err_x = self.current_velocity.twist.linear.x - self.latest_twist_cmd.twist.linear.x

                # @TODO final waypoints to calculate lin and ang velocity??
                # try with twist_cmd -> You will subscribe to `/twist_cmd` message which provides the proposed linear and
                # angular velocities.
                throttle, brake, steering = self.controller.control(
                    lin_vel=abs(self.latest_twist_cmd.twist.linear.x),
                    ang_vel=self.latest_twist_cmd.twist.angular.z,
                    current_lin_vel=self.current_velocity.twist.linear.x,
                    dbw_status=self.dbw_enabled,
                    del_time=self.del_time,
                    vel_err=vel_err_x)

                rospy.loginfo("""publish: t={} b={} s={}""".format(throttle, brake, steering))

                self.publish(throttle, brake, steering)
            else:
                pass
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
