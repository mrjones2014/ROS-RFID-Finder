#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import gc

upperBound = (79, 171, 235)
lowerBound = (44, 56, 113)


def dist_from_centered(image_message, publisher):
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message)
    except cv_bridge.CvBridgeError, e:
        rospy.logerr(e.message)

    if image is not None:
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        (height, width) = image.shape[:2]
        mask = cv2.inRange(hsv, lowerBound, upperBound)
        mask = cv2.erode(mask, None, iterations=2)  # make the selection closer to a
        mask = cv2.dilate(mask, None, iterations=2)  # regular polygon, if possible

        # find contours in the masked image and keep the largest one; the if/elif is to support
        # both versions 3.0.0 and 3.1.0 of OpenCV; the return type changed between versions.
        contours = None
        if cv2.__version__ == '3.1.0':
            (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        elif cv2.__version__ == '3.0.0':
            (contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            raise RuntimeError("OpenCV version 3.0.0 or 3.1.0 required! Installed version: " + cv2.__version__)

        if contours is not None:
            c = max(contours, key=cv2.contourArea)  # c is the largest contour

            # approximate the contour
            moments = cv2.moments(c)
            center_x = int(moments["m10"] / moments["m00"])  # x coord of center of object
            center_y = int(moments["m01"] / moments["m00"])  # y coord of center of object
            object_center = (center_x, center_y)

            # determine if the object is centered horizontally, plus/minus 5 px, and if so, publish value of 0
            if ((width / 2) - 5) < object_center[0] < ((width / 2) + 5):
                rospy.loginfo("## Object centered! ##")
                publisher.publish(0)
            else:
                # otherwise, publish difference between x coords of center of object and center of frame
                publisher.publish((width / 2) - object_center[0])
    gc.collect()  # force garbage collection; the list of contours potentially is very large


if __name__ == "__main__":
    rospy.init_node("optical_center_finder")
    pub = rospy.Publisher("optical_center_found", Int64, queue_size=10)
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, dist_from_centered, callback_args=pub, queue_size=10)
    rospy.loginfo("Node `optical_center_finder` started.")
    rospy.spin()
