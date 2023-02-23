#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

  

class ImageSubscriber:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=True)

        # Create a subscriber for the image topic
        self.sub = rospy.Subscriber('image_topic', Image, self.callback)

        # Create a CvBridge object to convert ROS messages to OpenCV images
        self.bridge = CvBridge()

    def callback(self, data):
    
       
        # Convert the ROS message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
         
        
       
        
        # Process the image (e.g. apply filters, detect edges, etc.)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        canny_image = cv2.Canny(gray_image, 100, 200)
      
        # Display the processed image using OpenCV
        
        cv2.imshow('Orignal Image' ,cv_image)
        cv2.imshow('Processed Image',canny_image)
        
        cv2.waitKey(1)
        
    
if __name__ == '__main__':
    try:
        # Create an instance of the ImageSubscriber class
        subscriber = ImageSubscriber()
        
       
 

        # Spin the ROS node
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

