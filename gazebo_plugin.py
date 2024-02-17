import rospy
import time
import os
import subprocess
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState, ModelStates
from tf.transformations import euler_from_quaternion
from my_robot_controller.src.darwin import Darwin
from my_robot_controller.src.walker import Walker
from std_srvs.srv import Empty
from geometry_msgs.msg import Wrench
import transforms3d

TIME_DELTA = 0.01

# 球门长度除2
gate_y = 0.5063

# 把全局速度转换到在局部坐标系下的速度，roll_quaternion即msg.pose[i].orientation
def vector_global_to_local_with_quaternion(vector_global, roll_quaternion):
    roll = np.array(roll_quaternion)
    roll = roll/np.linalg.norm(roll)
    rotation_matrix = transforms3d.quaternions.quat2mat(roll)
    vector_global_np = np.array(vector_global)
    vector_local_np = np.matmul(rotation_matrix, vector_global_np)
    vector_local = vector_local_np.tolist()
    return vector_local

class GazeboEnv:
    def __init__(self, launchfile):
        port = "11311"
        subprocess.Popen(["roscore", "-p", port])
        print("Roscore launched!")
        rospy.init_node("gym", anonymous=True)

        subprocess.Popen(["roslaunch", "-p", port, launchfile])
        print("Gazebo launched!")
        self.robomasterEP_pub = [rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=10)]
        self.football_pub = rospy.Publisher("/football/force", Wrench, queue_size=10)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._ModelState_callback)
        self.state = {} #存储[x,y,theta]
        self.STATE = {} #存储[x,y,z,roll.x,roll,y,roll,z,roll,w]
        self.set_self_state = ModelStates()
        self.cnt = 0

        self.darwin = [Darwin('/darwin1/'), Darwin('/darwin2/')]
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        # self.set_state = rospy.Publisher("gazebo/set_model_state", ModelStates, queue_size=10)
        Walker(self.darwin[0])
        Walker(self.darwin[1])

    # def _init_positions(self):
    #     self.model_state_sub.model_name = ['football', 'robot1', 'robot2', 'robot3', 'robot4', 'darwin1', 'darwin2']
    #     positions = []
    #     for i in range(7):
    #         self.set_self_state[i].pose.position.x = 0.0
    #         self.set_self_state[i].pose.position.y = 0.0
    #         self.set_self_state[i].pose.position.z = 0.0
    #         self.set_self_state[i].pose.orientation.x = 0.0
    #         self.set_self_state[i].pose.orientation.y = 0.0
    #         self.set_self_state[i].pose.orientation.z = 0.0
    #         self.set_self_state[i].pose.orientation.w = 1.0

    def _ModelState_callback(self, msg):
        names = msg.name
        for i in range(len(names)):
            key = names[i]
            x = msg.pose[i].position.x
            y = msg.pose[i].position.y
            z = msg.pose[i].position.z
            roll_q = msg.pose[i].orientation
            (_, _, theta) = euler_from_quaternion([roll_q.x, roll_q.y, roll_q.z, roll_q.w])
            self.state[key] = [x, y, z, theta]
            self.STATE[key] = [x, y, z, roll_q.w, roll_q.x, roll_q.y, roll_q.z]

    # 判定球是否出界或进球门,flag=0表示正常，flag=-1表示出界，flag=1表示1队进球，2表示2队进球
    def _if_terminate(self):
        flag = 0
        done = False
        football = self.state['football']
        [x, y, _, _] = football
        if x < -3:
            done = True
            if -gate_y < y < gate_y:
                flag = 2
            else:
                flag = -1
            return done, flag
        if x > 3:
            done = True
            if -gate_y < y < gate_y:
                flag = 2
            else:
                flag = -1
            return done, flag
        if y > 2 or y < -2:
            done = True
            flag = -1
        return done, flag

    def step(self, EPvelocity, DWvelocity):
        
        football_msg = Wrench()
        if self.cnt<1:
            [a, b, c] = [0.1, 0, 0]
        else:
            [a,b,c] = [0,0,0]
        
        [_,_,_, w, x, y, z] = self.STATE['football']
        # [a,b,c] =  vector_global_to_local_with_quaternion([a,b,c],[w, x, y, z])
        football_msg.force.x = a
        football_msg.force.y = b
        football_msg.force.z = c
        self.football_pub.publish(football_msg)
        reward = np.zeros(6)
        for i in range(4):
            ep_msg = Twist()
            ep_msg.linear.x = EPvelocity[i * 3]
            ep_msg.linear.y = EPvelocity[i * 3 + 1]
            ep_msg.angular.z = EPvelocity[i * 3 + 2]
            self.robomasterEP_pub[i].publish(ep_msg)

        for i in range(2):
            vx = DWvelocity[i * 3]
            vy = DWvelocity[i * 3 + 1]
            w = DWvelocity[i * 3 + 2]
            # self.darwin[i].set_walk_velocity(vx, vy, w)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        rospy.sleep(TIME_DELTA)
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        done, flag = self._if_terminate()
        if flag == 1:
            reward += np.array([1, 1, 0, 0, 1, 0])
        if flag == 2:
            reward += np.array([0, 0, 1, 0, 0, 1])
        self.cnt+=1
        return self.state, done, reward

    def reset(self):
        self.cnt = 0
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
        
        return self.state
