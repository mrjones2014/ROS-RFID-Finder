#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv_bridge
from geometry_msgs.msg import Twist
import cv2
import gc
import imutils


upperBound = (179, 184, 171)
lowerBound = (0,  55, 138)


def move_to_object(image_message, publisher):
    bridge = cv_bridge.CvBridge()
    image = None
    try:
        image = bridge.imgmsg_to_cv2(image_message)
    except cv_bridge.CvBridgeError, e:
        rospy.logerr(e.message)
        print e.message

    if image is not None:
        image = imutils.resize(image, width=600)
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        (height, width) = image.shape[:2]
        mask = cv2.inRange(hsv, lowerBound, upperBound)
        mask = cv2.erode(mask, None, iterations=2)  # make the selection closer to a
        mask = cv2.dilate(mask, None, iterations=2)  # regular polygon, if possible

        # find contours in the masked image and keep the largest one
        cnts = None
        (cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cv2.__version__ == "3.1.0":  # Because the return tupled changed in version 3.1.0
            (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if cnts is not None and len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)  # c is the largest contour

            # approximate the contour
            moments = cv2.moments(c)
            center_x = int(moments["m10"] / moments["m00"])  # x coord of center of object
            center_y = int(moments["m01"] / moments["m00"])  # y coord of center of object
            object_center = (center_x, center_y)

            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (center_x, center_y), 7, (255, 255, 255), -1)
            cv2.putText(image, "center", (center_x - 20, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # determine if the object is centered horizontally, plus/minus 5 px, and if so, publish value of 0
            if ((width / 2) - 5) < object_center[0] < ((width / 2) + 5):
                rospy.loginfo("## Object centered! ##")
                message_val = 0
            else:
                # otherwise, publish difference between x coords of center of object and center of frame
                message_val = (width / 2) - object_center[0]
            gc.collect()  # force garbage collection; the list of contours potentially is very large
            vel = Twist()
            if message_val < 0:  # object is to right of center; rotate left
                vel.angular.z = -0.4
                if message_val > -20:
                    vel.angular.z = -0.2
                vel.linear.x = 0
            elif message_val > 0:  # object is to left of center; rotate right
                vel.angular.z = 0.4
                if message_val > 20:
                    vel.angular.z = 0.2
                vel.linear.x = 0
            else:  # object is centered; go forward
                vel.angular.z = 0
                vel.linear.x = 0.4
            rospy.loginfo("optical_center_finder reported: " + str(message_val))
            publisher.publish(vel)
        cv2.imshow("img", image)
        cv2.waitKey(1)


def on_rfid_found(string_msg):
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)


if __name__ == "__main__":
    rospy.init_node("find_object")
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.Subscriber("camera/rgb/image_rect_color", Image, move_to_object, callback_args=robot)
    #rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)
    print "Publisher and subscribers established..."
    rospy.spin()
