import rospy
import time
import os
import subprocess
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from gazebo_msgs.msg import ContactsState, ModelStates
from tf.transformations import euler_from_quaternion
from my_robot_controller.src.darwin import Darwin
from my_robot_controller.src.walker import Walker
from std_srvs.srv import Empty
from geometry_msgs.msg import Wrench
import transforms3d
from argument import get_args

args = get_args()
TIME_DELTA = args.time_delta
torque = args.gripper_torque
gripper_center = args.gripper_center 
gripper_width = args.gripper_width
gripper_height = args.gripper_height

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

        rospy.init_node("gym", anonymous=True)
        self.ns = ['robot1','robot2','robot3','robot4']
        subprocess.Popen(["roslaunch", "-p", port, launchfile])
        self.robomasterEP_pub = [rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=10),
                                 rospy.Publisher("/robot4/cmd_vel", Twist, queue_size=10)]
        self.football_pub = rospy.Publisher("/football/force", Wrench, queue_size=10)
        self.model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._ModelState_callback)

        self.gripper_pub_left_1 = rospy.Publisher("/robot1/left_gripper_joint_1_controller/command",Float64, queue_size=10)
        self.gripper_pub_right_1 = rospy.Publisher("/robot1/right_gripper_joint_1_controller/command",Float64, queue_size=10)
        self.gripper_pub = [[rospy.Publisher('/'+self.ns[i]+"/left_gripper_joint_1_controller/command",Float64, queue_size=10),
                            rospy.Publisher('/'+self.ns[i]+"/right_gripper_joint_1_controller/command",Float64, queue_size=10)] for i in range(4)]

        self.state = {} #存储[x,y,theta]
        self.prev_state = {} # 上一时刻的状态，t=0时刻与state相同
        self.STATE = {} #存储[x,y,z,roll.x,roll,y,roll,z,roll,w]
        self.prev_STATE = {}
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
            self.state[key] = [x, y, theta]
            self.STATE[key] = [x, y, z, roll_q.w, roll_q.x, roll_q.y, roll_q.z]
        self.prev_state = self.state.copy()
        self.prev_STATE = self.STATE.copy()
    # 判定球是否出界或进球门,flag=0表示正常，flag=-1表示出界，flag=1表示1队进球，2表示2队进球
    def _if_terminate(self):
        flag = 0
        done = False
        football = self.state['football']
        [x, y, _] = football
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
    
    # 判定球权，若球不属于任何人，返回0，若属于roboti，则返回i正整数
    def ball_possession(self):
        [bx, by, _] = self.state['football']
        for i in range(4):
            [x, y, theta] = self.state[self.ns[i]]
            cos = np.cos(theta)
            sin = np.sin(theta)
            dx = bx-x
            dy = by-y
            [dx, dy] = [dx * cos + dy * sin, dy * cos - dx * sin]
            if abs(dx-gripper_center) < gripper_width and abs(dy) < gripper_height:
                return i + 1
        return 0

            
        
    
    def _launch_football(self, football_f=[0,0,0]):
        football_msg = Wrench()
        football_msg.force.x = football_f[0]
        football_msg.force.y = football_f[1]
        football_msg.force.z = football_f[2]
        self.football_pub.publish(football_msg)

    def gripper(self, i, if_close): 
        gripper_pub = self.gripper_pub[i]
        if if_close:
            J1 = torque
        else:
            J1 = -torque
        gripper_pub[0].publish(-J1)
        gripper_pub[1].publish(J1)
    



    # 输入：
    # EPvelocity: 4*3=12维向量，robomasterEP的速度
    # DWelocity：2*3=6维向量，darwin机器人的速度
    # football_f：给football施加的力作为发射器
    # grip：4维bool向量，False表示张开夹爪，True表示关闭夹爪
    # 输出：
    # state
    # bool型变量done
    # reward为6维向量[robot1,robot2,robot3,robot4,darwin1,darwin2](顺序可调换，暂定为此顺训)
    def step(self, EPvelocity, DWvelocity, football_f, grip):
        # 控制夹爪
        for i in range(4):
            self.gripper(i, grip[i])

        # 控制足球
        self._launch_football(football_f)

        reward = np.zeros(6)

        # 控制robomasterEP机器人的平面移动
        for i in range(4):
            ep_msg = Twist()
            ep_msg.linear.x = EPvelocity[i * 3]
            ep_msg.linear.y = EPvelocity[i * 3 + 1]
            ep_msg.angular.z = EPvelocity[i * 3 + 2]
            self.robomasterEP_pub[i].publish(ep_msg)

        # 控制darwin机器人
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

