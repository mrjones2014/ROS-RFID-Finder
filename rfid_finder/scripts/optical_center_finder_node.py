#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import gc

upperBound = (79, 171, 235)
lowerBound = (44, 56, 113)


def is_center(image_message, publisher):
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message)
    except cv_bridge.CvBridgeError, e:
        rospy.logerr(e.message)

    if image:
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        (height, width) = image.shape[:2]
        mask = cv2.inRange(hsv, lowerBound, upperBound)
        mask = cv2.erode(mask, None, iterations=2)  # make the selection closer to a
        mask = cv2.dilate(mask, None, iterations=2)  # regular polygon, if possible

        # find contours in the masked image and keep the largest one
        (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)

            # approximate the contour
            moments = cv2.moments(c)
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            object_center = (center_x, center_y)

            # determine if the object is centered horizontally, plus/minus 5 px
            if ((width / 2) - 5) < object_center[0] < ((width / 2) + 5):
                rospy.loginfo("## Object centered! ##")
                publisher.publish(0)
            else:
                publisher.publish((width / 2) - object_center[0])
    gc.collect()  # force garbage collection; contours potentially are very large

if __name__ == "__main__":
    rospy.init_node("optical_center_finder")
    pub = rospy.Publisher("optical_center_found", Int64, queue_size=10)
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, is_center, callback_args=pub, queue_size=10)
    rospy.loginfo("Node `optical_center_finder` started.")
    rospy.spin()
