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
TIME_DELTA = args.time_delta


def vel_global2local(vglobal, theta):
    cos = np.cos(theta)
    sin = np.sin(theta)
    vlocal = [vglobal[0]*cos + vglobal[1]*sin, vglobal[1]*cos - vglobal[0]*sin, vglobal[2]]
    return vlocal

def shoot0():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip, football_f

def pass_ball1():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip, football_f

def run2():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip

def catch3(name, state, prev_state, possession):
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    [bx, by, _] = state['football']
    [prev_bx, prev_by, _] = prev_state['football']
    vbx, vby = (bx-prev_bx)/TIME_DELTA, (by-prev_by)/TIME_DELTA
    [x, y, theta] = state[name]
    dx, dy = bx - x + gripper_center*np.cos(theta), by - y + gripper_center*np.sin(theta)
    min_t = np.sqrt(dx**2+dy**2)/max_vel
    w = abs(np.arctan(dy/dx) - theta)
    if w>3.1415926:
        w = 4*w/min_t
    else:
        w = -4*w/min_t
    vx = avg_vel * dx/np.sqrt(dx**2+dy**2) + vbx
    vy = avg_vel * dy/np.sqrt(dx**2+dy**2) + vby
    coef = np.min([max_vel/np.sqrt(vx**2+vy**2), 1])
    coef = np.max([min_vel/np.sqrt(vx**2+vy**2), 1])
    vel = [vx*coef, vy*coef, w]
    print(abs(np.arctan(dy/dx) - theta), min_t,w)
    if name[-1]==str(possession):
        grip = True
    return vel_global2local(vel, theta), grip

def run4():
    vel = [0, 0, 0]
    grip = False
    football_f = [0, 0, 0]
    return vel, grip

def defense5():
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
    possession = env.ball_possession()
    for i in range(4):
        v = [0, 0, 0]
        g = False
        f = [0, 0, 0]
        if command[i, 0]:
            v, g, f = shoot0()
        elif command[i, 1]:
            v, g, f = pass_ball1()
        elif command[i, 2]:
            v, g = run2()
        elif command[i, 3]:
            v, g = catch3(env.ns[i], state, prev_state, possession)
        elif command[i, 4]:
            v, g = run4()
        elif command[i, 5]:
            v, g = defense5()
        vel += v
        grip[i] = g
        if possession == i-1:
            football_f = f
    return vel, football_f, grip


