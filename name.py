#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_ros_msg.msg import ArUcoMarkers

class ArucoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_detector', anonymous=True)

        # Create publisher for custom ROS message
        self.pub = rospy.Publisher('aruco_markers', ArUcoMarkers, queue_size=10)

        # Create bridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to image topic
        rospy.Subscriber('image_topic', Image, self.process_image)

    def process_image(self, img_msg):
        # Convert ROS Image message to OpenCV image
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Initialize ArUco detector
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        # Get bounding boxes of markers
        bounding_boxes = []
        for i in range(len(ids)):
            x, y, w, h = cv2.boundingRect(corners[i])
            bounding_boxes.append((x, y, w, h, ids[i]))

        # Populate custom ROS message
        aruco_msg = ArUcoMarkers()
        aruco_msg.bounding_boxes = bounding_boxes

        # Publish message
        self.pub.publish(aruco_msg)

    def run(self):
        # Spin ROS node
        rospy.spin()

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    aruco_detector.run()

