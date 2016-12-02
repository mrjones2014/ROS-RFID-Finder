#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import imutils


def publish_camera_data():
    rospy.init_node("camera_node")
    camera = cv2.VideoCapture(0)  # create a camera object from the first available attached camera
    pub = rospy.Publisher("camera_rgb_raw", Image, queue_size=10)  # create the publisher to send the Image messages
    rate = rospy.Rate(10)  # 10hz
    rospy.loginfo("camera_node started...")
    while not rospy.is_shutdown():
        (_, image) = camera.read()  # read the image from the camera
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_converter = cv_bridge.CvBridge()
        image_message = image_converter.cv2_to_imgmsg(image)  # convert cv2 image matrix to ROS image message
        pub.publish(image_message)
        rate.sleep()

if __name__ == "__main__":
    publish_camera_data()
