from gazebo_plugin import GazeboEnv
import numpy as np
import time
import Command2Velocity as C2V
from argument import get_args

args = get_args()
launchfile = args.launchfile
env = GazeboEnv(launchfile)
t1 = time.time()
for iter in range(1):
    # state = env.reset()
    for i in range(800):
        DWvelocity = np.array([0.1, 0, 0, 0.0, -0.2, 0])
        commandEP = np.zeros((4,6))
        commandEP[0, 3] = 1
        if i>300:
            commandEP[0, 0] = 1
            commandEP[0, 3] = 0
        EPvelocity, football_f, grip = C2V.FSM(commandEP, env)
        state, done, reward = env.step(EPvelocity, DWvelocity, football_f, grip)
        if done:
            break
t2 = time.time()
print('time = ', t2 - t1)
