#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraPublisher:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_publisher', anonymous=True)

        # Create a publisher for the image topic
        self.pub = rospy.Publisher('image_topic', Image, queue_size=10)

        # Create a CvBridge object to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Initialize the camera
        self.camera = cv2.VideoCapture(0)

    def run(self):
        # Continuously capture and publish images
        while not rospy.is_shutdown():
            ret, frame = self.camera.read()
            if ret:
                try:
                    # Convert the OpenCV image to a ROS message
                    img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')

                    # Publish the message to the image topic
                    self.pub.publish(img_msg)

                except CvBridgeError as e:
                    print(e)

if __name__ == '__main__':
    try:
        # Create an instance of the CameraPublisher class
        publisher = CameraPublisher()

        # Run the publisher
        publisher.run()

    except rospy.ROSInterruptException:
        pass

