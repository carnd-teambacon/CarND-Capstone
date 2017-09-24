from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #self.model = load_model('light_classifier_model.h5')
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO (denise) implement yellow and green and compare areas
        
        output = image.copy()
        hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

        red = cv2.inRange(hsv, np.array([160,140,50]) , np.array([180,255,255]))
        yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([30, 255, 255]))
        green = cv2.inRange(hsv, np.array([50, 100, 100]), np.array([70, 255, 255]))

        _,contours_red,hierarchy = cv2.findContours(red, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        _,contours_green,hierarchy = cv2.findContours(green, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        _,contours_yellow,hierarchy = cv2.findContours(yellow, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        #cv2.drawContours(red, contours_red, -1, (0,0,100), 3)
       # cv2.drawContours(green, contours_green, -1, (0,0,100), 3)
        #cv2.drawContours(yellow, contours_yellow, -1, (0,0,100), 3)

        red_area = 0
        for cnt in contours_red:
            red_area = red_area + cv2.contourArea(cnt)
        green_area = 0
        for cnt in contours_green:
            green_area = green_area + cv2.contourArea(cnt)
        yellow_area = 0
        for cnt in contours_yellow:
            yellow_area = yellow_area + cv2.contourArea(cnt)


        if red_area > green_area and red_area > yellow_area:
            return TrafficLight.RED
        if yellow_area > green_area and yellow_area > red_area:
            return TrafficLight.YELLOW
        if green_area > red_area and green_area > yellow_area:
            return TrafficLight.GREEN
            
        return TrafficLight.UNKNOWN
