#!/usr/bin/env python3.12
import numpy as np
import rclpy
from phi_p3dx_navigation.main import NavigationNode

# ── Constantes do robô ─────────────────────────────────────────────
L = 0.331
R = 0.09751

# ── Velocidades ─────────────────────────────────────────────────────
V_FWD   = 0.4
V_CURVE = 0.15
W_TURN  = np.deg2rad(40)
W_FIND  = -np.deg2rad(25)

# ── Wall following PD ──────────────────────────────────────────────
WALL_TARGET  = 0.50
KP           = 0.8
KD           = 0.4
W_MAX_FOLLOW = np.deg2rad(20)

# ── Limiares ────────────────────────────────────────────────────────
FRONT_STOP = 0.50
WALL_LOST  = 1.20

# ── Índices das regiões do laser (180 beams) ────────────────────────
IDX_FRONT_L = 75
IDX_FRONT_R = 105
IDX_FRNT_RL = 45
IDX_FRNT_RR = 75
IDX_RIGHT_L = 0
IDX_RIGHT_R = 45

# ── Estados ─────────────────────────────────────────────────────────
FORWARD     = 'FORWARD'
TURN_LEFT   = 'TURN_LEFT'
FOLLOW_WALL = 'FOLLOW_WALL'
FIND_WALL   = 'FIND_WALL'


class WallFollower(RobotController):
    """Implementa apenas a lógica de wall-following."""

    def __init__(self):
        super().__init__('wall_follower_controller', timer_period=0.05)
        self.state = FORWARD
        self.prev_error = 0.0
        self.get_logger().info('WallFollower (PD lateral) iniciado')

    # ── Leituras regionais ──────────────────────────────────────────

    @property
    def d_front(self) -> float:
        return self.get_region_distance(IDX_FRONT_L, IDX_FRONT_R)

    @property
    def d_front_right(self) -> float:
        return self.get_region_distance(IDX_FRNT_RL, IDX_FRNT_RR)

    @property
    def d_right(self) -> float:
        return self.get_region_distance(IDX_RIGHT_L, IDX_RIGHT_R)

    # ── Máquina de estados ──────────────────────────────────────────

    def _next_state(self) -> str:
        obs_front = self.d_front < FRONT_STOP
        obs_fr = self.d_front_right < FRONT_STOP
        wall_here = self.d_right < WALL_LOST

        if obs_front or obs_fr:
            return TURN_LEFT
        if wall_here:
            return FOLLOW_WALL
        if self.state in (FOLLOW_WALL, FIND_WALL):
            return FIND_WALL
        return FORWARD
        
    def _control_loop(self):
        if not self.has_laser_data():
            return

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()