import gymnasium  as gym 
from omni.isaac.lab_tasks.utils import parse_env_cfg


def main():
    task = 'Isaac-Open-Washer-Franka-v0'
    env_cfg = parse_env_cfg(
        'Isaac-Open-Washer-Franka-v0', use_fabric=False, use_gpu=False
    )


    env = gym.make(task, cfg=env_cfg, render_mode='rgb_array')
