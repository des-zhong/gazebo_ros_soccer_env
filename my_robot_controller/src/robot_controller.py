#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState, ModelStates
from tf.transformations import euler_from_quaternion
import math
import random
import numpy as np
import rospy
from darwin import Darwin
from walker import Walker

STATE = {}
FOOTBALL = [0, 0, 0]


def ModelMSG(msg):
    global STATE
    global FOOTBALL
    names = msg.name
    for i in range(len(names)):
        key = names[i]
        x = msg.pose[i].position.x
        y = msg.pose[i].position.y
        z = msg.pose[i].position.z
        roll_q = msg.pose[i].orientation
        (_, _, theta) = euler_from_quaternion([roll_q.x, roll_q.y, roll_q.z, roll_q.w])
        STATE[key] = [x, y, z, theta]
    rospy.loginfo(STATE)


def robomaster_controller():
    Velocity = np.ones(3 * 6) * 0.1
    return Velocity


def darwin_controller():
    Velocity = [0.1, 0.1, 0, -0.0, -0.2, 0.2]
    # Velocity = np.zeros(3 * 2)
    return Velocity


def main():
    rospy.init_node('walking demo', anonymous=True)
    ns_s = ['robot1', 'robot2', 'robot3', 'robot4']
    robomasterEP_msg = [Twist() for _ in range(4)]
    robomasterEP_pub = [rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10),
                        rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10),
                        rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=10),
                        rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=10)]
    model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, ModelMSG)

    darwin1 = Darwin('/darwin1/')
    darwin2 = Darwin('/darwin2/')
    darwin = [darwin1, darwin2]
    Walker(darwin1)
    Walker(darwin2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        EPvelocity = robomaster_controller()
        DWvelocity = darwin_controller()
        for i in range(4):
            robomasterEP_msg[i].linear.x = EPvelocity[i * 3]
            robomasterEP_msg[i].linear.y = EPvelocity[i * 3 + 1]
            robomasterEP_msg[i].angular.z = EPvelocity[i * 3 + 2]
            robomasterEP_pub[i].publish(robomasterEP_msg[i])

        for i in range(2):
            vx = DWvelocity[i * 3]
            vy = DWvelocity[i * 3 + 1]
            w = DWvelocity[i * 3 + 2]
            darwin[i].set_walk_velocity(vx, vy, w)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
