#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_turtle(command):
    # Create a new Twist message
    twist_msg = Twist()

    # Set linear and angular velocities based on the command
    if command == "前":
        twist_msg.linear.x = 0.2  # Forward velocity
    elif command == "右":
        twist_msg.angular.z = -0.2  # Clockwise angular velocity
    elif command == "左":
        twist_msg.angular.z = 0.2  # Counter-clockwise angular velocity
    elif command == "後ろ":
        twist_msg.linear.x = -0.2  # Backward velocity
    elif command == "止まれ":
        twist_msg.linear.x = 0.0  # Stop linear motion
        twist_msg.angular.z = 0.0  # Stop angular motion
    else:
        rospy.logwarn("Unknown command: %s", command)
        return

    # Publish the Twist message to control the turtle
    twist_pub.publish(twist_msg)

def main():
    rospy.init_node('turtle_control')
    rospy.Subscriber('voice_commands', String, move_turtle)
    rospy.spin()

if __name__ == '__main__':
    try:
        twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        main()
    except rospy.ROSInterruptException:
        pass