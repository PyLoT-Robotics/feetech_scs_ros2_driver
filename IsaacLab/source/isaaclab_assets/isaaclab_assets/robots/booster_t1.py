import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg


BOOSTER_T1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/absolute/path/to/booster_t1.usd",  # ←自分のUSD
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            fix_root_link=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    ),

    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.95),
        joint_pos={
            # 頭
            "AAHead_yaw": 0.0,
            "Head_pitch": 0.2,

            # 腰
            "Waist": 0.0,

            # 腕（少し下げる）
            "Left_Shoulder_Pitch": -0.3,
            "Right_Shoulder_Pitch": -0.3,

            # 脚（直立安定姿勢）
            "Left_Hip_Pitch": -0.4,
            "Right_Hip_Pitch": -0.4,
            "Left_Knee_Pitch": 0.8,
            "Right_Knee_Pitch": 0.8,
            "Left_Ankle_Pitch": -0.4,
            "Right_Ankle_Pitch": -0.4,
        },
        joint_vel={".*": 0.0},
    ),

    soft_joint_pos_limit_factor=0.9,

    actuators={
        # 脚（歩行の主役）
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*Hip_.*",
                ".*Knee_Pitch",
                ".*Ankle_.*",
            ],
            effort_limit_sim=300,
            stiffness=120.0,
            damping=8.0,
            armature=0.01,
        ),

        # 腕
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*Shoulder_.*",
                ".*Elbow_.*",
            ],
            effort_limit_sim=150,
            stiffness=40.0,
            damping=6.0,
            armature=0.005,
        ),

        # 頭 + 腰
        "upper": ImplicitActuatorCfg(
            joint_names_expr=[
                "AAHead_yaw",
                "Head_pitch",
                "Waist",
            ],
            effort_limit_sim=80,
            stiffness=30.0,
            damping=4.0,
        ),
    },
)
