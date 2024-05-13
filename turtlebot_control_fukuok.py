#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
import yaml
import time
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

grammar = """
[GRAMMAR]
greeding : $slot_greeding
bring_known_obj : $slot_drink * $slot_person * <持って行って|届けて|取って>
bring_unknown_obj : $slot_any を $slot_person * <持って行って|届けて|取って>




[SLOT]
$slot_greeding
forward : 前
right : 右
left : 左
backward:後ろ
stop:止まれ
ninetyleft:レフト
ninetyright:ライト

$slot_drink
drink1 : ジュース
drink2 : コーラ|コーク
drink3 : お茶|緑茶

$slot_person
person1 : 中村|中村さん
person2 : 田中
person3 : 太郎

"""

def callback_laser(msg):
    global distance
    data = np.array(msg.ranges)
    data = np.where(data<msg.range_min, msg.range_max, data)
    distance = min(min(data[0:30]), min(data[330:360]))

def callback_shutdown():
    cmd_vel_pub.publish(stop_cmd)

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
scan_sub = rospy.Subscriber('scan', LaserScan, callback_laser)

rospy.init_node('auto') 

move_cmd = Twist()
rotate_cmd = Twist()
stop_cmd = Twist()

move_cmd.linear.x = 0.2
move_cmd.linear.y = 0
move_cmd.linear.z = 0
move_cmd.angular.x = 0
move_cmd.angular.y = 0
move_cmd.angular.z = 0

rotate_cmd.linear.x = 0
rotate_cmd.linear.y = 0
rotate_cmd.linear.z = 0
rotate_cmd.angular.x = 0
rotate_cmd.angular.y = 0
rotate_cmd.angular.z = 0.2

distance = 100

rate = rospy.Rate(10)

rospy.on_shutdown(callback_shutdown)

# required for stabilizing rospy.Time.now() in Gazebo simulation
rospy.sleep(0.0) 

def main():
    rospy.init_node('gspeech_example2')


    pub_grammar = rospy.Publisher( "grammar_lu/grammar", String, queue_size=10, latch=True )
    pub_synthesis = rospy.Publisher( "google_speech/utterance", String, queue_size=10 )	

    time.sleep(2)
    pub_grammar.publish( grammar )


    while not rospy.is_shutdown():
        msg = rospy.wait_for_message( 'grammar_lu/results',  String )
        results = yaml.safe_load(msg.data)
        twist_msg = Twist()

        print( results["text"] )
        print( results["gram_id"] )
        print( results["slot_id"] )
        print( results["slot_str"] )
        print("--------")

        if distance < 0.3:
            cmd_vel_pub.publish(stop_cmd)
            if results["gram_id"]=="greeding":
                if results["slot_id"][0]=="forward":
                    twist_msg.linear.x += 0.1  # Forward velocity
                elif results["slot_id"][0]=="right":
                    twist_msg.angular.z += -0.2  # Clockwise angular velocity
                elif results["slot_id"][0]=="left":
                    twist_msg.angular.z += 0.2  # Counter-clockwise angular velocity
                elif results["slot_id"][0]=="backward":
                    twist_msg.linear.x += -0.1  # Backward velocity
                elif results["slot_id"][0]=="stop":
                    twist_msg.linear.x = 0.0  # Stop linear motion
                    twist_msg.angular.z = 0.0  # Stop angular motione
                elif results["slot_id"][0]=="ninetyright":
                    twist_msg.angular.z += -0.2
                    print(twist_msg)
                    twist_pub.publish(twist_msg)
                    time.sleep(7.85)
                    twist_msg.linear.x = 0.0  # Stop linear motion
                    twist_msg.angular.z = 0.0  # Stop angular motione
                elif results["slot_id"][0]=="ninetyleft":
                    twist_msg.angular.z += 0.2
                    print(twist_msg)
                    twist_pub.publish(twist_msg)
                    time.sleep(7.85)
                    twist_msg.linear.x = 0.0  # Stop linear motion
                    twist_msg.angular.z = 0.0  # Stop angular motione
                else:
                    rospy.logwarn("Unknown command: %s", command)
            print(twist_msg)
            twist_pub.publish(twist_msg)

            if __name__ == '__main__':
                twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                main()
                
        else:
            cmd_vel_pub.publish(move_cmd)

        rate.sleep()





