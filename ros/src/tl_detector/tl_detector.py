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
from pyquaternion import Quaternion

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        #model = load_model('light_classifier_model.h5')
        #model.summary()

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        
        print('image_cb')
        self.has_image = True
        self.camera_image = msg
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
            i = 1
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_index(self, pose, pose_list):
        """Identifies the closest lights to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

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


    def project_to_image_plane(self, point_in_world):
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

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (transT, rotT) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")


        rot = [rotT[0], rotT[1], rotT[2], rotT[3]]
        quat = Quaternion(numpy.array(rot))
        point_in_world_vec = numpy.array([point_in_world.x, point_in_world.y, point_in_world.z])
        trans_vec = numpy.array(([transT[0], transT[1], transT[2]]))
        point_in_camera = quat.rotate(point_in_world_vec) + trans_vec

        print(point_in_camera)

        x = point_in_camera[0] * fx / (-point_in_camera[2]); 
        y = point_in_camera[1] * fy / (-point_in_camera[2]); 

        x = int(x+ image_width/2)
        y = int(y+ image_height/2) 

        #x = 0
        #y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        ###TODO(denise) Replace with CV later
        print('get light state')
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")



        x, y = self.project_to_image_plane(light.pose.pose.position)

        #pil_image = Image.fromstring("L", cv.GetSize(cv_image), cv_image.tostring())
        #crop image

        #img_array = img_to_array(pil_image.resize((64, 64), Image.ANTIALIAS))
        #img_array = img_array/255
        #img_array =  img_array[None, :]
        #model.predict(img_array)





        #TODO (denise) what should this size be?
        #stoplights are about 1067x356 mm
        #cv2.rectangle(cv_image,(x,y),(x-10,y+10),(0,255,0),3)
        #image for debugging
        #cv2.imwrite('test.png',cv_image)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None
        light_positions = self.config['light_positions']
        
        
        if(self.pose and self.waypoints):

            #TODO find the closest visible traffic light (if one exists)
            light_position = self.get_closest_index(self.pose.pose, self.lights)
            state = self.lights[light_position].state
            light_wp = self.get_closest_index(self.lights[light_position].pose.pose, self.waypoints.waypoints)
            #TODO (denise) this should be the final state used
            calc_state = self.get_light_state(self.lights[light_position])
            return light_wp, state


        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
