from __future__ import annotations

import torch 
from typing import TYPE_CHECKING
import omni.isaac.lab.utils.math as math_utils 
from omni.isaac.lab.assets import ArticulationData
from omni.isaac.lab.sensors import FrameTransformerData

if TYPE_CHECKING :
    from omni.isaac.lab.envs import ManagerBasedRLEnv

def rel_ee_object_distance(env:ManagerBasedRLEnv) -> torch.Tensor:
    ee_tf_data: FrameTransformerData = env.scene['ee_frame'].data
    object_data: ArticulationData = env.scene['object'].data 
    # what is the object
    return object_data.root_pos_w - ee_tf_data.target_pos_w[..., 0, :] # TODO how could it be 0 ?


def rel_ee_drawer_distance(env: ManagerBasedRLEnv) -> torch.Tensor:
    ee_tf_data: FrameTransformerData = env.scene['ee_frame'].data
    washer_tf_data: FrameTransformerData = env.scene['washer_frame'].data 
    # what is the object
    return washer_tf_data.target_pos_w[..., 0, :] - ee_tf_data.target_pos_w[..., 0, :] # TODO how could it be 0 ?

def fingertips_pos(env:ManagerBasedRLEnv) -> torch.Tensor:
    ee_tf_data: FrameTransformerData = env.scene['ee_frame'].data
    fingertips_pos = ee_tf_data.target_pos_w[..., 1:, :] - env.scene.env_origins.unsqueeze(1)

    return fingertips_pos.view(env.num_envs, -1 )



def ee_pos(env:ManagerBasedRLEnv) -> torch.Tensor:
    ee_tf_data: FrameTransformerData = env.scene['ee_frame'].data 
    ee_pos = ee_tf_data.target_pos_w[..., 0, :] -env.scene.env_origins
    return ee_pos



def ee_quat(env: ManagerBasedRLEnv, make_quat_unique: bool=False) -> torch.Tensor:

    ee_tf_data: FrameTransformerData =env.scene['ee_frame'].data 
    ee_quat = ee_tf_data.target_quat_w[..., 0,:]
    return math_utils.quat_unique(ee_quat) if make_quat_unique else ee_quat