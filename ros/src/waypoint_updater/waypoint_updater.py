#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DEBUG_MODE = False

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below        
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.cur_pose = None
        self.waypoints = None        

        rospy.spin()

    def pose_cb(self, msg):
        self.cur_pose = msg.pose        
        self.publish()

    def waypoints_cb(self, lane):
        # do this once and not all the time
        if self.waypoints is None:
            self.waypoints = lane.waypoints
            self.publish()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. We will implement it later
        if msg > -1:
            self.set_waypoint_velocity(self.waypoints, int(msg.data), 0)
       # pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def closest_waypoint(self, pose, waypoints):
        # simply take the code from the path planning module and re-implement it here
        closest_len = 100000
        closest_waypoint = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for index, waypoint in enumerate(self.waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                closest_len = dist
                closest_waypoint = index
                
        return closest_waypoint

    def next_waypoint(self, pose, waypoints):
        # same concepts from path planning in here
        closest_waypoint = self.closest_waypoint(pose, waypoints)
        map_x = waypoints[closest_waypoint].pose.pose.position.x
        map_y = waypoints[closest_waypoint].pose.pose.position.y

        heading = math.atan2((map_y - pose.position.y), (map_x - pose.position.x))
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        angle = abs(yaw - heading)

        if angle > (math.pi / 4):        
            closest_waypoint += 1
        
        return closest_waypoint

    def publish(self):
        
        if self.cur_pose is not None:
            next_waypoint_index = self.next_waypoint(self.cur_pose, self.waypoints)
            lookahead_waypoints = self.waypoints[next_waypoint_index:next_waypoint_index+LOOKAHEAD_WPS]
            
            # set the velocity for lookahead waypoints
            for i in range(len(lookahead_waypoints) - 1):                
                # convert 10 miles per hour to meters per sec
                self.set_waypoint_velocity(lookahead_waypoints, i, (10 * 1609.34) / (60 * 60))

            if DEBUG_MODE:
                posx = self.waypoints[next_waypoint_index].pose.pose.position.x
                posy = self.waypoints[next_waypoint_index].pose.pose.position.y
                rospy.loginfo("Closest waypoint: [index=%d posx=%f posy=%f]", next_waypoint_index, posx, posy)

            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = lookahead_waypoints
            
            self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')