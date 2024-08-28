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
## scene definition

@configclass
class RefrigeratorSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = MISSING

    ee_frame: FrameTransformerCfg = MISSING

    refrigerator = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Refrigerator",
        spawn=sim_utils.UsdFileCfg(
            usd_path="/home/lr-2002/refrigerator.usd",
            # scale=(0.5, 0.5, 0.5),
            activate_contact_sensors=False,),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(-2,0, 0.4), # TODO 
            rot=(0.0, 0.0, 0.0,0.0),
            joint_pos={
                # TODO 
                'refrigerator_54_link_joint_1': 0.0, 
                "refrigerator_54_link_joint_2": 0.0,
            }
        ),
        actuators={
                # TODO
            'uppper': ImplicitActuatorCfg(
                joint_names_expr=['refrigerator_54_link_joint_1'],
                effort_limit=87.0,
                velocity_limit=100.0,
                stiffness=10.0,
                damping=1.0,
            ),
            'down': ImplicitActuatorCfg(
                joint_names_expr=['refrigerator_54_link_joint_2'],
                effort_limit=87.0,
                velocity_limit=100.0,
                stiffness=10.0,
                damping=1.0,
            )
        }
    )

    refrigerator_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Refrigerator/refrigerator_54_link",
        debug_vis=True,
        visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/RefrigeratorFrameTransformer"),
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Refrigerator/refrigerator_54_link_4", # TODO,
                name="handle", # TODO 
                offset=OffsetCfg(
                    pos = (0, 0 ,0 ),
                    rot = (1, 0, 0, 0),
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
        prim_path='/Wolrd/light',
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
        refrigerator_joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={'asset_cfg': SceneEntityCfg("refrigerator", joint_names=["refrigerator_54_link_joint_1"])}, # TODO 
        )
        refrigerator_joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={'asset_cfg': SceneEntityCfg("refrigerator", joint_names=["refrigerator_54_link_joint_1"])}, # TODO 
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

    refrigerator_physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode='startup',
        params={
            'asset_cfg': SceneEntityCfg('refrigerator', body_names="refrigerator_54_link_joint_1"), # TODO 
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
    approach_ee_handle = RewTerm(func=mdp.approach_ee_handle, weight=2.0, params={"threshold": 0.2})
    align_ee_handle = RewTerm(func=mdp.align_ee_handle, weight=0.)

    # 2. Grasp the handle
    approach_gripper_handle = RewTerm(func=mdp.approach_gripper_handle, weight=5.0, params={"offset": MISSING})
    align_grasp_around_handle = RewTerm(func=mdp.align_grasp_around_handle, weight=0.125)
    grasp_handle = RewTerm(
        func=mdp.grasp_handle,
        weight=0.8,
        params={
            "threshold": 0.03,
            "open_joint_pos": MISSING,
            "asset_cfg": SceneEntityCfg("robot", joint_names=MISSING),
        },
    )

    # 3. Open the drawer
    open_drawer_bonus = RewTerm(
        func=mdp.open_drawer_bonus,
        weight=0,
        params={"asset_cfg": SceneEntityCfg("refrigerator", joint_names=["refrigerator_54_link_joint_1"], joint_ids=[0])},
    )
    multi_stage_open_drawer = RewTerm(
        func=mdp.multi_stage_open_drawer,
        weight=7.0,
        params={"asset_cfg": SceneEntityCfg("refrigerator", joint_names=["refrigerator_54_link_joint_1"], joint_ids=[0])},
    )

    # 4. Penalize actions for cosmetic reasons
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-1e-2)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0001)


@configclass
class TerminationCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    



@configclass
class RefrigeratorEnvCfg(ManagerBasedRLEnvCfg):
    scene: RefrigeratorSceneCfg = RefrigeratorSceneCfg(num_envs=1024, env_spacing=5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandCfg = CommandCfg()
    rewards: RewardCfg = RewardCfg()
    terminations : TerminationCfg=TerminationCfg()
    events: EventCfg= EventCfg()


    def __post_init__(self):
        self.decimation = 1 
        self.episode_length_s = 0.0 
        self.viewer.eye = (-2, 2, 2)
        self.viewer.lookat = (0.8, 0, 0.5)
        self.sim.dt = 1/ 60 
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01

        self.sim.physx.friction_correlation_distance = 0.00625