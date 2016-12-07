#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from geometry_msgs.msg import Twist
import gc
import imutils
from std_msgs.msg import String


"""
TODO:
-Use different camera (USB webcam) to solve color issues
-Figure out how to set up USB webcam as ROS node
-Increase dead zone turn speed slightly; <= 0.2 is not enough to move the bot at all
"""

upperBound = (18, 222, 235)
lowerBound = (0, 155, 158)


def move_to_object(image_message, publisher):
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message, "bgr8")  # convert image message to OpenCV image matrix
    except cv_bridge.CvBridgeError, e:
        rospy.logerr(e.message)
        print e.message

    if image is not None:
        image = imutils.resize(image, width=600)  # resize the image for displaying onscreen
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # convert image to HSV color space
        (height, width) = image.shape[:2]
        mask = cv2.inRange(hsv, lowerBound, upperBound)  # create a mask layer based on color bounds
        mask = cv2.erode(mask, None, iterations=2)  # make the selection closer to a
        mask = cv2.dilate(mask, None, iterations=2)  # regular polygon, if possible

        # find contours in the masked image and keep the largest one
        if cv2.__version__ == "3.1.0":  # Because the return tupled changed in version 3.1.0
            (_, contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            (contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None and len(contours) > 0:  # if a contour is found...
            c = max(contours, key=cv2.contourArea)  # let c be the largest contour

            # approximate the centroid of the contour
            moments = cv2.moments(c)
            centroid_x = int(moments["m10"] / moments["m00"])  # x coord of centroid of object
            centroid_y = int(moments["m01"] / moments["m00"])  # y coord of centroid of object
            object_centroid = (centroid_x, centroid_y)

            # draw the contour and centroid of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (centroid_x, centroid_y), 7, (255, 255, 255), -1)
            cv2.putText(image, "center", (centroid_x - 20, centroid_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # determine if the object is centered horizontally, plus/minus 10 px, and if so, publish value of 0
            if ((width / 2) - 10) < object_centroid[0] < ((width / 2) + 10):
                rospy.loginfo("## Object centered! ##")
                message_val = 0
            else:
                # otherwise, publish difference between x coords of center of object and center of frame
                message_val = (width / 2) - object_centroid[0]
            gc.collect()  # force garbage collection; the list of contours potentially is very large
            vel = Twist()
            if message_val < 0:  # object is to right of center; rotate left
                vel.angular.z = -0.6
                if message_val > -20:  # slow down speed as we get closer
                    vel.angular.z = -0.4
                vel.linear.x = 0
            elif message_val > 0:  # object is to left of center; rotate right
                vel.angular.z = 0.6
                if message_val > 20:  # slow down speed as we get closer
                    vel.angular.z = 0.4
                vel.linear.x = 0
            else:  # object is centered; go forward
                vel.angular.z = 0
                vel.linear.x = 0.6
            rospy.loginfo("optical_center_finder reported: " + str(message_val))
            publisher.publish(vel)  # publish the velocity commands as a Twist message
        cv2.imshow("img", image)  # show the image
        cv2.waitKey(1)  # refresh contents of image frame


def on_rfid_found(string_msg):
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)


if __name__ == "__main__":
    rospy.init_node("find_object")  # initialize the node
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)  # set up a publisher to control Turtlebot
    rospy.Subscriber("camera/rgb/image_raw", Image, move_to_object, callback_args=robot)  # camera subscriber
    rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)  # rfid data subscriber
    rospy.loginfo("Node `find_object` started...")  # loginfo that the node has been set up
    rospy.spin()  # keeps the script from exiting until the node is killed
