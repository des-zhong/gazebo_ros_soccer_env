import argparse


def get_args():
    parser = argparse.ArgumentParser("Reinforcement Learning experiments for multiagent environments")
    parser.add_argument("--launchfile", type=str, default='./my_robot_controller/launch/load_env.launch')
    parser.add_argument("--gripper-center", type=float, default=0.265, help="center of the gripper relative to the robomasterEP center")
    parser.add_argument("--gripper-width", type=float, default=0.03, help="param for possesion of the ball")
    parser.add_argument("--gripper-height", type=float, default=0.015, help="param for possesion of the ball")
    parser.add_argument("--gripper-torque", type=int, default=10, help="torque for the gripper to grip football")
    parser.add_argument("--max-launch-impulse", type=float, default=0.1, help="max launch impulse when trying to shoot the ball")
    parser.add_argument("--time-delta", type=float, default=0.01, help="time delta")
    args = parser.parse_args()

    return args