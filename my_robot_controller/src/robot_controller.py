#!/usr/bin/env python
# -*- coding: utf-8 -*-
## testing
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
import random
import rospy
from darwin import Darwin
from walker import Walker


def robomasterEP_move():
    rospy.init_node('tbk', anonymous=True)

    pub = rospy.Publisher('/robot1/triangle_joint_controller/command', Float64, queue_size=1000)
    msg = Float64()

    # Publish the Twist message
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        msg.data = -1.0
        pub.publish(msg)
        rate.sleep()



def Darwin_move():
    rospy.init_node("walker_demo")
    
    rospy.loginfo("Instantiating Darwin Client")
    darwin=Darwin('/darwin1/')
    walker=Walker(darwin)
    rospy.sleep(1)
 
    rospy.loginfo("Darwin Walker Demo Starting")


    darwin.set_walk_velocity(0.2,0,0)
    rospy.sleep(3)
    darwin.set_walk_velocity(0.2,0,1)
    rospy.sleep(3)
    darwin.set_walk_velocity(0,0.2,1)
    rospy.sleep(3)
    darwin.set_walk_velocity(0,-0.2,1)
    rospy.sleep(3)
    darwin.set_walk_velocity(-0.2,0,1)
    rospy.sleep(3)
    darwin.set_walk_velocity(0.1,0.2,0)
    rospy.sleep(5)
    darwin.set_walk_velocity(0,0,0)
    
    rospy.loginfo("Darwin Walker Demo Finished")


def main():
    ns_s = ['/robot1','/robot2','/robot3','/robot4']
    rospy.init_node('walking demo', anonymous=True)
    move_pub1 = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
    twist_msg1 = Twist()
    twist_msg1.linear.x = 0.1
    twist_msg1.linear.y = 0
    twist_msg1.angular.z = 0

    move_pub2 = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10)
    twist_msg2 = Twist()
    twist_msg2.linear.x = -0.1
    twist_msg2.linear.y = -0.0
    twist_msg2.angular.z = 0.1

    move_pub3 = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=10)
    twist_msg3 = Twist()
    twist_msg3.linear.x = 0.2
    twist_msg3.linear.y = 0.1
    twist_msg3.angular.z = 0

    move_pub4 = rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=10)
    twist_msg4 = Twist()
    twist_msg4.linear.x = 0.1
    twist_msg4.linear.y = -0.1
    twist_msg4.angular.z = -0.1

    darwin1=Darwin('/darwin1/')
    darwin2=Darwin('/darwin2/')
    walker1=Walker(darwin1)
    walker2=Walker(darwin2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        move_pub1.publish(twist_msg1)
        move_pub2.publish(twist_msg2)
        move_pub3.publish(twist_msg3)
        move_pub4.publish(twist_msg4)
        darwin1.set_walk_velocity(1,0.2,0.1)
        darwin2.set_walk_velocity(0.2,-0.1, 0)

        rate.sleep()

if __name__ == '__main__':
    try:
        # robomasterEP_move()
        # Darwin_move()
        main()
    except rospy.ROSInterruptException:
        pass
