#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def rotate_towards_object(int64_message, publisher):
    vel = Twist()
    message_val = int64_message.data
    if message_val < 0:  # object is to right of center; rotate left
        vel.angular.z = -0.8
        vel.linear.x = 0
    elif message_val > 0:  # object is to left of center; rotate right
        vel.angular.z = 0.8
        vel.linear.x = 0
    else:  # object is centered; go forward
        vel.angular.z = 0
        vel.linear.x = 1
    rospy.loginfo("optical_center_finder reported: " + message_val)
    publisher.publish(vel)


def on_rfid_found(string_msg):
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)


if __name__ == "__main__":
    rospy.init_node("find_object")
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.Subscriber("optical_center_found", Int64, rotate_towards_object, callback_args=robot)
    rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)