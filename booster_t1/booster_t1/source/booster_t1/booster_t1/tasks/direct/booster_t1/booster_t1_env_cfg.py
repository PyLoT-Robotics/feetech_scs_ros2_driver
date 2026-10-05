from dataclasses import dataclass, field
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.assets import ArticulationCfg

@dataclass
class BoosterT1EnvCfg(DirectRLEnvCfg):
    # mutable なデフォルトは必ず default_factory
    robot_cfg: ArticulationCfg = field(default_factory=ArticulationCfg)

    max_cart_pos: float = 2.0
    initial_pole_angle_range: tuple[float, float] = (-0.05, 0.05)
    rew_scale_alive: float = 1.0
    rew_scale_terminated: float = -1.0
    rew_scale_pole_pos: float = -1.0
    rew_scale_cart_vel: float = -0.1
    rew_scale_pole_vel: float = -0.1

# env を初期化する際に USD パスを設定
def setup_booster_t1_env_cfg(cfg: BoosterT1EnvCfg):
    cfg.robot_cfg.file = "/home/koki/booster_gym/resources/T1/booster_serial.usd"
    return cfg
