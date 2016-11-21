import rospy
from std_msgs.msg import Int64
from std_msgs.msg import String
from geometry_msgs.msg import Twist

object_found = False


def rotate_towards_object(int64_message, publisher):
    twist = Twist()
    message_val = int64_message.data
    global object_found
    if -999999 < message_val < 0:  # object is to right of center; rotate left
        twist.angular.z = -0.2
        twist.linear.x = 0
        object_found = True
    elif message_val > 0:  # object is to left of center; rotate right
        twist.angular.z = 0.2
        twist.linear.x = 0
        object_found = True
    else:  # object is centered; go forward
        twist.angular.z = 0
        twist.linear.x = 0.5
        object_found = True
    rospy.loginfo("optical_center_finder reported: " + message_val)
    publisher.publish(twist)


def on_rfid_found(string_msg):
    tag_id = string_msg.data
    rospy.loginfo("Found RFID tag with ID: " + tag_id)


if __name__ == "__main__":
    rospy.init_node("find_object")
    robot = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.Subscriber("optical_center_found", Int64, rotate_towards_object, callback_args=robot)
    rospy.Subscriber("rfid_data", String, on_rfid_found, queue_size=10)

    global object_found
    while not object_found:
        twist = Twist()
        twist.angular.z = 0.5
        robot.publish(twist)
