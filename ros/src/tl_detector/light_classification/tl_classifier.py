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

        # define range of blue color in HSV 
        lower_red = np.array([160,140,50]) 
        upper_red = np.array([180,255,255])

        imgThreshHigh = cv2.inRange(hsv, lower_red, upper_red)
        thresh = imgThreshHigh.copy()

        _,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(hsv, contours, -1, (0,0,255), 3)

        cv2.imwrite('contours.jpg', hsv)

        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            max_area = max_area + area

        height, width, channels = output.shape

        if max_area > 50:
            return TrafficLight.RED

        return TrafficLight.UNKNOWN
