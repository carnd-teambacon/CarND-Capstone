#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import tf.transformations as tft
import cv2
import yaml
import sys
import math
import numpy
import os
import PIL 
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'

#from keras.models import load_model
#from keras.preprocessing.image import load_img, img_to_array
#from pyquaternion import Quaternion

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.tl_img_crop_pub = rospy.Publisher('/tl_img_crop', Image, queue_size=1)
        #self.tl_center_img_pub = rospy.Publisher('/tl_center_img', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.styx = (str(rospy.get_param('/styx')).upper() == ('true').upper())
        self.is_site_image = False

        self.debug = 0

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(5) #Hz
        while not rospy.is_shutdown():
            if self.pose is not None and self.waypoints is not None and self.has_image:
                light_wp, state = self.process_traffic_lights()

                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1

                '''
                Broadcast /base_link to /world transform based on pose, if not styx (because server/bridge already takes care of it there)
                '''
                # styx = str(rospy.get_param("/styx")) == 'true'
                # rospy.loginfo("styx: " + str(styx))
                # if not styx:
                #     tf_position = (self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z)
                #     tf_orientation = (self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w)
                #     br = tf.TransformBroadcaster()
                #     br.sendTransform(tf_position, tf_orientation, rospy.Time.now(), "base_link", "world")
                        
                rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        
        if not self.has_image:
            rospy.loginfo("Received image from simulator")
        self.has_image = True
        self.camera_image = msg
        

    def get_closest_index(self, pose, pose_list):
        """Identifies the closest lights to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            post_list: a list of positions (of traffic lights)
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        #TODO(denise) implement optimized algorithm
        min_dist = 1e100
        index = 0
        for i, pt in enumerate(pose_list):
            dist = math.hypot(pt.pose.pose.position.x-pose.position.x, pt.pose.pose.position.y-pose.position.y)
            if dist < min_dist:
                min_dist = dist
                index = i
        return index


    def project_to_image_plane(self, ptx, pty, ptz, offsetX, offsetY):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        cx = image_width/2
        cy = image_height/2

        # get transform between pose of camera and world frame
        transT = None
        rotT = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (transT, rotT) = self.listener.lookupTransform("/base_link",
                  "/world", now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None, None

        # simply using the current pose instead of waitForTransform doesn't seem to work - they're not the same
        # transT2 = (-self.pose.pose.position.x, -self.pose.pose.position.y, -self.pose.pose.position.z)
        # rotT2 = (-self.pose.pose.orientation.x, -self.pose.pose.orientation.y, -self.pose.pose.orientation.z, self.pose.pose.orientation.w)

        # rospy.loginfo("trans1: " + str(transT))
        # rospy.loginfo("trans2: " + str(transT2))
        # rospy.loginfo("rot1: " + str(rotT))
        # rospy.loginfo("rot2: " + str(rotT2))

        rpy = tf.transformations.euler_from_quaternion(rotT)
        yaw = rpy[2]

        #(ptx, pty, ptz) = (point_in_world.x, point_in_world.y, point_in_world.z)

        point_to_cam = (ptx * math.cos(yaw) - pty * math.sin(yaw),
                        ptx * math.sin(yaw) + pty * math.cos(yaw), 
                        ptz)
        point_to_cam = [sum(x) for x in zip(point_to_cam, transT)]

        point_to_cam[1] = point_to_cam[1] + offsetX
        point_to_cam[2] = point_to_cam[2] + offsetY

        rospy.loginfo_throttle(3, "traffic light location: " + str(ptx) + "," + str(pty) + "," + str(ptz))
        #rospy.loginfo_throttle(3, "cam to world trans: " + str(transT))
        #rospy.loginfo_throttle(3, "cam to world rot: " + str(rotT))
        #rospy.loginfo_throttle(3, "roll, pitch, yaw: " + str(rpy))
        rospy.loginfo_throttle(3, "camera to traffic light: " + str(point_to_cam))

        ##########################################################################################
        # DELETE THIS MAYBE - MANUAL TWEAKS TO GET THE PROJECTION TO COME OUT CORRECTLY IN SIMULATOR
        # just override the simulator parameters. probably need a more reliable way to determine if 
        # using simulator and not real car
        if self.styx:            
            fx = 2574
            fy = 2744
            point_to_cam[2] -= 1.0
            cx = image_width/2 - 30
            cy = image_height + 50
            self.is_site_image = False
        else:
            point_to_cam[2] -= 1.0
            self.is_site_image = True
        ##########################################################################################

        x = -point_to_cam[1] * fx / point_to_cam[0]; 
        y = -point_to_cam[2] * fy / point_to_cam[0]; 

        x = int(x + cx)
        y = int(y + cy) 
        rospy.loginfo_throttle(3, "traffic light pixel (x,y): " + str(x) + "," + str(y))

        return (x, y)


    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        ###TODO(denise) Replace with CV later
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        if self.styx:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        else: 
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        height, width, channels = cv_image.shape

        ptx = light.pose.pose.position.x 		
        pty = light.pose.pose.position.y 		
        ptz = light.pose.pose.position.z 		
 		
        x_center, y_center = self.project_to_image_plane(ptx, pty, ptz, 0, 0)		
        x_top, y_top = self.project_to_image_plane(ptx, pty, ptz, 1.0, 1.0)
        x_bottom, y_bottom = self.project_to_image_plane(ptx, pty, ptz, -1.0, -1.0)
        
        if x_bottom > width or y_bottom > height or x_top < 0 or y_top < 0:
            return TrafficLight.UNKNOWN

        cpy = cv_image.copy()		
 		
        #x_top = self.cap_value(x_top, 0, 600)		
        #x_bottom = self.cap_value(x_bottom, 0, 600)		
        #y_top = self.cap_value(y_top, 0, 800)
        #y_bottom = self.cap_value(y_bottom, 0, 800)
        # 
        #rospy.loginfo_throttle(4, "top left: " + str(x_top) + "," + str(y_top))		#rospy.loginfo_throttle(4, "bottom left: " + str(x_bottom) + "," + str(y_bottom))
        # 
        #if image is too small, ignore
        if x_bottom is None or x_top is None or y_top is None or y_bottom is None:
            return TrafficLight.UNKNOWN
        if x_bottom - x_top < 20 or y_bottom-y_top < 40:
            return TrafficLight.UNKNOWN		
        
        crop_img = cpy[int(y_top):int(y_bottom), int(x_top):int(x_bottom)]	
        
        # publish the image with traffic light with markers on center, top left, and bottom right	
        
        cv2.circle(cpy,(x_center, y_center), 8, (255,0,255), -1)	
        # cv2.circle(cpy,(x_top, y_top), 8, (255,0,255), -1)
        # cv2.circle(cpy,(x_bottom, y_bottom), 8, (255,0,255), -1)
        tl_img_crop_msg = self.bridge.cv2_to_imgmsg(cpy, encoding="bgr8")
        self.tl_img_crop_pub.publish(tl_img_crop_msg)

        #Get classification
        #classification, show_img = self.light_classifier.get_classification(cv_image) #(crop_img)
        classification, show_img = self.light_classifier.get_classification(crop_img, self.is_site_image)
     
        # tl_img_crop_msg = self.bridge.cv2_to_imgmsg(show_img, encoding="bgr8")
        # self.tl_img_crop_pub.publish(tl_img_crop_msg)

        return classification

    def generate_light(self, x, y, z):
        
        light = TrafficLight()
        light.pose = PoseStamped()
        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z
        return light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        #pose = geometry_msgs.msg.PoseStamped()
        

        if(self.pose and self.waypoints):
            #TODO (denise) make sure the point is not behind me
            light_position = self.get_closest_index(self.pose.pose, self.lights)
           # state = self.lights[light_position].state
            light_wp = self.get_closest_index(self.lights[light_position].pose.pose, self.waypoints.waypoints)
            lines = list()
            for light_pos in stop_line_positions:
                light =  self.generate_light(light_pos[0], light_pos[1], 0.)
                lines.append(light)
            line_wp = self.get_closest_index(lines[light_position].pose.pose, self.waypoints.waypoints)
            #TODO (denise) this should be the final state used
            state = self.get_light_state(self.lights[light_position])
            #rospy.loginfo_throttle(2, "Light: " + str(state))
            return line_wp, state


        if light:
            state = self.get_light_state(light)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

    def cap_value(self, value, min, max):

        if value < min:
            vaue = min
        elif value > max:
            value = max

        return value


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')