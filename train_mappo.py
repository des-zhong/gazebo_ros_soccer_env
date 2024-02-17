from gazebo_plugin import GazeboEnv
import numpy as np
import time

launchfile = './my_robot_controller/launch/load_env.launch'
env = GazeboEnv(launchfile)
t1 = time.time()
for iter in range(1):
    # state = env.reset()
    for i in range(1000):
        EPvelocity = np.array([1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0]) * 0.3
        DWvelocity = np.array([0.1, 0, 0, 0.0, -0.2, 0])
        state, done, reward = env.step(EPvelocity, DWvelocity)
        if done:
            break
        # print(state, '\n')
t2 = time.time()
print('time = ', t2 - t1)
