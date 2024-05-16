#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import yaml
import time
from geometry_msgs.msg import Twist

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
straight:前進
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
            elif results["slot_id"][0]=="straight":
                twist_msg.linear.x += 0.1
                print(twist_msg)
                twist_pub.publish(twist_msg)
                time.sleep(8.0)
                twist_msg.linear.x = 0.0  # Stop linear motion
                twist_msg.angular.z = 0.0 
            elif results["slot_id"][0]=="ninetyright":
                twist_msg.angular.z += -0.2
                print(twist_msg)
                twist_pub.publish(twist_msg)
                time.sleep(8.0)
                twist_msg.linear.x = 0.0  # Stop linear motion
                twist_msg.angular.z = 0.0  # Stop angular motione
            elif results["slot_id"][0]=="ninetyleft":
                twist_msg.angular.z += 0.2
                print(twist_msg)
                twist_pub.publish(twist_msg)
                time.sleep(8.0)
                twist_msg.linear.x = 0.0  # Stop linear motion
                twist_msg.angular.z = 0.0  # Stop angular motione
            else:
                rospy.logwarn("Unknown command: %s", command)
            
        print(twist_msg)
        twist_pub.publish(twist_msg)




if __name__ == '__main__':
    # rospy.Subscriber("chatter",String,callback1)
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    main()
