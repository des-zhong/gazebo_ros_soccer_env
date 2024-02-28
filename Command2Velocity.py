import numpy as np
from argument import get_args
args = get_args()

# command为robomasterEP的command，为4*6维矩阵，每行代表一个robomasterEP需要执行的指令，如若第一行为[1,0,0,0,0,0]且持有球则表示robot1执行第一个指令
# 指令暂定为：
# 有球时：射门，传球，跑动, 无球时：接球, 跑动, 防守
max_vel = 1
min_vel = 0.2
avg_vel = (max_vel+min_vel)/2
gripper_center = args.gripper_center
gripper_width = args.gripper_width
gripper_l = gripper_center + gripper_width
TIME_DELTA = args.time_delta


def vel_global2local(vglobal, theta):
    cos = np.cos(theta)
    sin = np.sin(theta)
    vlocal = [vglobal[0]*cos + vglobal[1]*sin, vglobal[1]*cos - vglobal[0]*sin, vglobal[2]]
    return vlocal

def shoot(name, state, prev_state, possession):
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    if name[-1]==str(possession):
        [x, y, theta] = state[name]
        football_f = [0.05*np.cos(theta), 0.05*np.sin(theta), 0]
    else:
        vel, grip = catch(name, state, prev_state, possession)
    return vel, grip, football_f 

def pass_ball():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip, football_f

def run1():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip

def catch(name, state, prev_state, possession):
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    [bx, by, _] = state['football']
    [prev_bx, prev_by, _] = prev_state['football']
    vbx, vby = (bx-prev_bx)/TIME_DELTA, (by-prev_by)/TIME_DELTA
    [x, y, theta] = state[name]
    theta0 = np.arctan((by-y)/(bx-x))
    dx, dy = bx - x + gripper_l*np.cos(theta0), by - y + gripper_l*np.sin(theta0)
    min_t = np.sqrt(dx**2+dy**2)/max_vel
    w = abs(theta0 - theta)
    if w>3.1415926:
        w = 4*w/min_t
    else:
        w = -4*w/min_t
    vx = avg_vel * dx/np.sqrt(dx**2+dy**2) + vbx
    vy = avg_vel * dy/np.sqrt(dx**2+dy**2) + vby
    coef = np.min([max_vel/np.sqrt(vx**2+vy**2), 1])
    coef = np.max([min_vel/np.sqrt(vx**2+vy**2), 1])
    vel = [vx*coef, vy*coef, w]
    if name[-1]==str(possession):
        grip = True
    return vel_global2local(vel, theta), grip

def run2():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip

def defense():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip



def FSM(command, env):
    state = env.state
    prev_state = env.prev_state
    vel = []
    grip = [False, False, False, False]
    football_f = [0, 0, 0]
    football_f = []
    possession = env.ball_possession()
    for i in range(4):
        v = [0, 0, 0]
        g = False
        f = np.array([0, 0, 0])
        if command[i, 0]:
            v, g, f = shoot(env.ns[i], state, prev_state, possession)
        elif command[i, 1]:
            v, g, f = pass_ball()
        elif command[i, 2]:
            v, g = run1()
        elif command[i, 3]:
            v, g = catch(env.ns[i], state, prev_state, possession)
        elif command[i, 4]:
            v, g = run2()
        elif command[i, 5]:
            v, g = defense()
        vel += v
        grip[i] = g
        football_f.append(f)
    if possession == 0:
        return vel, [0, 0, 0], grip
    else:
        return vel, football_f[possession-1], grip
    


