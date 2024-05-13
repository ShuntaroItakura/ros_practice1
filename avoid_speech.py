#! /usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback_laser(msg):
    global distance
    data = np.array(msg.ranges)
    data = np.where(data<msg.range_min, msg.range_max, data)
    distance = min(min(data[0:30]), min(data[330:360]))

def callback_shutdown():
    twist_pub.publish(stop_cmd)

twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
scan_sub = rospy.Subscriber('scan', LaserScan, callback_laser)

rospy.init_node('auto') 

distance = 100

rate = rospy.Rate(10)

rospy.on_shutdown(callback_shutdown)

# required for stabilizing rospy.Time.now() in Gazebo simulation
rospy.sleep(0.0) 

while not rospy.is_shutdown():
    twist_cmd = Twist()
    if distance < 0.3:
       
        twist_cmd.angular.z = 0.2
    else:
        twist_cmd.linear.x = 0.2
        

    if msg:
        if results["gram_id"]=="greeding":
            if results["slot_id"][0]=="forward":
                twist_cmd.linear.x += 0.1  # Forward velocity
            elif results["slot_id"][0]=="right":
                twist_cmd.angular.z += -0.2  # Clockwise angular velocity
            elif results["slot_id"][0]=="left":
                twist_cmd.angular.z += 0.2  # Counter-clockwise angular velocity
            elif results["slot_id"][0]=="backward":
                twist_cmd.linear.x += -0.1  # Backward velocity
            elif results["slot_id"][0]=="stop":
                twist_cmd.linear.x = 0.0  # Stop linear motion
                twist_cmd.angular.z = 0.0  # Stop angular motione
            elif results["slot_id"][0]=="ninetyright":
                twist_cmd.angular.z += -0.2
                print(twist_cmd)
                twist_pub.publish(twist_cmd)
                time.sleep(7.85)
                twist_cmd.linear.x = 0.0  # Stop linear motion
                twist_cmd.angular.z = 0.0  # Stop angular motione
            elif results["slot_id"][0]=="ninetyleft":
                twist_cmd.angular.z += 0.2
                print(twist_cmd)
                twist_pub.publish(twist_cmd)
                time.sleep(7.85)
                twist_cmd.linear.x = 0.0  # Stop linear motion
                twist_cmd.angular.z = 0.0  # Stop angular motione
            else:
                rospy.logwarn("Unknown command: %s", command)

    twist_pub.publish(twist_cmd)

    rate.sleep()