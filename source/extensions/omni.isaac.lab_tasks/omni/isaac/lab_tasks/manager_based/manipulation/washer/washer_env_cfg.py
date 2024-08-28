from dataclasses import MISSING

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators.actuator_cfg import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup 
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm 
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer import OffsetCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp 


from omni.isaac.lab.markers.config import FRAME_MARKER_CFG

FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers['frame'].scale = (0.10, 0.10, 0.10)
JOINT_NAME  = "joint_0"
LINK_NAME = 'link_0'
FRAME_NAME = None
## scene definition

@configclass
class WasherSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = MISSING

    ee_frame: FrameTransformerCfg = MISSING

    washer = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Washer",
        spawn=sim_utils.UsdFileCfg(
            # usd_path="/home/lr-2002/code/IsaacLab/source/extensions/omni.isaac.lab_assets/data/7130/mobility.usd",
            usd_path="/home/lr-2002/Downloads/7778/mobility/mobility/mobility.usd",
            scale=(0.8, 0.8, 0.8),
            # usd_path="/home/lr-2002/Downloads/dishwasher1.usd",
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True,
                enabled_self_collisions=True,
            ),
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_depenetration_velocity=5.0,
        ),
            ),
        init_state=ArticulationCfg.InitialStateCfg(
            # pos=(1.4, -0.15, 0.5), # TODO 
            pos=(1.3, -0.15, 0.45), # TODO 
            rot=(1.0, 0.0, 0.0, 0.0),
            joint_pos={
                # TODO 
                JOINT_NAME: 0.0
            }
        ),
        actuators={
            'drawer': ImplicitActuatorCfg(
                joint_names_expr=[JOINT_NAME],
                effort_limit=100.0,
                velocity_limit=50.0,
                stiffness=0.0,
                damping=0.0,
                friction=0.8,
            )
        }
    )

    washer_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Washer/base",
        debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/WasherFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Washer/link_0", # TODO,
                name="link_0_frame", # TODO 
                offset=OffsetCfg(
                    pos = (-0.5, 0.45,0.09 ),
                    rot = (1.0, 0.0, 0.0, 0.0),
                )
            )
        ]
    )


    plane = AssetBaseCfg(
        prim_path='/World/GroundPlane',
        init_state=AssetBaseCfg.InitialStateCfg(),
        spawn=sim_utils.GroundPlaneCfg(),
        collision_group=-1,
    )

    light = AssetBaseCfg(
        prim_path='/World/light',
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000)
    )


@configclass
class CommandCfg:
    null_command = mdp.NullCommandCfg()


@configclass
class ActionsCfg:
    body_joint_pos: mdp.JointPositionActionCfg = MISSING
    finger_joint_pos: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    @configclass 
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        washer_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={'asset_cfg': SceneEntityCfg("washer", joint_names=["joint_0"])}, # TODO 
        )
        washer_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={'asset_cfg': SceneEntityCfg("washer", joint_names=["joint_0"])}, # TODO 
        )
        #this should be joint (PHYSICS REVOLUTE JOINT AND SO ON )
        rel_ee_drawer_distance = ObsTerm(func=mdp.rel_ee_drawer_distance)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True 
            self.concatenate_terms = True
        
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    robot_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode='startup',
        params={
            'asset_cfg': SceneEntityCfg('robot', body_names='.*'),
            'static_friction_range': (0.8, 1.25),
            'dynamic_friction_range': (0.8, 1.25),
            'restitution_range': (0, 0),
            'num_buckets': 16,
        }
    )

    washer_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode='startup',
        params={
            'asset_cfg': SceneEntityCfg('washer', body_names="link_0"), # TODO 
            ## THIS SOULD BE LINK with frame 
            'static_friction_range': (1, 1.25),
            'dynamic_friction_range': (1.25, 1.25),
            'restitution_range': (0, 0),
            'num_buckets': 16,
        }
    )


    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode='reset')

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset, 
        mode='reset',
        params={
            'position_range': (-0.1, 0.1),
            'velocity_range': (0.0, 0.0)
        }
    )


@configclass
class RewardCfg:

    """Reward terms for the MDP."""

    # 1. Approach the handle
    approach_ee_handle = RewTerm(func=mdp.approach_ee_handle, weight=2.0, params={"threshold": 0.3})
    align_ee_handle = RewTerm(func=mdp.align_ee_handle, weight=0.4)

    # 2. Grasp the handle
    approach_gripper_handle = RewTerm(func=mdp.approach_gripper_handle, weight=5.0, params={"offset": MISSING})
    align_grasp_around_handle = RewTerm(func=mdp.align_grasp_around_handle, weight=0.125)
    grasp_handle = RewTerm(
        func=mdp.grasp_handle,
        weight=0.8,
        params={
            "threshold": 0.04,
            "open_joint_pos": MISSING,
            "asset_cfg": SceneEntityCfg("robot", joint_names=MISSING),
        },
    )

    # 3. Open the drawer
    open_drawer_bonus = RewTerm(
        func=mdp.open_drawer_bonus,
        weight=1,
        params={"asset_cfg": SceneEntityCfg("washer", joint_names=["joint_0"], joint_ids=[0])},
    )
    multi_stage_open_drawer = RewTerm(
        func=mdp.multi_stage_open_drawer,
        weight=7.0,
        params={"asset_cfg": SceneEntityCfg("washer", joint_names=["joint_0"], joint_ids=[0])},
    )

    # 4. Penalize actions for cosmetic reasons
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-1e-2)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0001)


@configclass
class TerminationCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    



@configclass
class WasherEnvCfg(ManagerBasedRLEnvCfg):
    scene: WasherSceneCfg = WasherSceneCfg(num_envs=1024, env_spacing=10)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandCfg = CommandCfg()
    rewards: RewardCfg = RewardCfg()
    terminations : TerminationCfg=TerminationCfg()
    events: EventCfg= EventCfg()


    def __post_init__(self):
        self.decimation = 1 
        self.episode_length_s = 8.0
        self.viewer.eye = (-2, 2, 2)
        self.viewer.lookat = (0.8, 0, 0.5)
        self.sim.dt = 1/ 60 
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.2 
        self.sim.physx.bounce_threshold_velocity = 0.01

        self.sim.physx.friction_correlation_distance = 0.00625