from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab_tasks.manager_based.manipulation.refrigerator import mdp 
from omni.isaac.lab_tasks.manager_based.manipulation.refrigerator.refrigerator_env_cfg import (
    FRAME_MARKER_SMALL_CFG,
    RefrigeratorEnvCfg,
)


from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG


@configclass
class FrankaRefrigeratorEnvCfg(RefrigeratorEnvCfg):
    def __post_init__(self):
        super().__post_init__()
    
        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.actions.body_joint_pos = mdp.JointPositionActionCfg(
            asset_name='robot',
            joint_names=['panda_joint.*'],
            scale=1.0,
            use_default_offset=True,
        )

        self.actions.finger_joint_pos = mdp.BinaryJointPositionActionCfg(
            asset_name='robot',
            joint_names=['panda_finger.*'],
            open_command_expr={'panda_finger_.*':0.04},
            close_command_expr={'panda_finger.*': 0.0}
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path='{ENV_REGEX_NS}/Robot/panda_link0',
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path='/Visuals/EndEffectorTransFormer'),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path='{ENV_REGEX_NS}/Robot/panda_hand',
                    name='ee_tcp',
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.1034),
                    )
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path='{ENV_REGEX_NS}/Robot/panda_leftfinger',
                    name='tool_leftfinger',
                    offset=OffsetCfg(
                        pos=(0,0,0.046),
                    )
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path='{ENV_REGEX_NS}/Robot/panda_rightfinger',
                    name='tool_rightfinger',
                    offset=OffsetCfg(
                        pos=(0,0, 0.046)
                    )
                )
            ]
        )
        self.rewards.approach_gripper_handle.params['offset'] = 0.04
        self.rewards.grasp_handle.params['open_joint_pos'] = 0.04
        self.rewards.grasp_handle.params['asset_cfg'].joint_names = ['panda_finger_.*']



@configclass
class FrankaRefrigeratorEnvCfg_PLAY(FrankaRefrigeratorEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        

        self.scene.num_envs = 50 
        self.scene.env_spacing = 2.5 

        self.observations.policy.enable_corruption = False