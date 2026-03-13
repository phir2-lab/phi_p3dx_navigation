#!/usr/bin/env python3.12
"""
potential_field.py – Campo Potencial sobre NavigationNode
==========================================================
Herda NavigationNode (main.py).
Obstáculos via /laser_scan (já processado pela classe base).
Goal via /goal_pose do RViz ("2D Nav Goal").
"""

import math
import numpy as np
import rclpy

from phi_p3dx_navigation.main import NavigationNode

MAX_VEL   = 0.3
MAX_OMEGA = 1.0


class PotentialFieldController(NavigationNode):

    KA = 1.2
    KT = 0.5
    KR = 0.8
    G_STAR = 0.5

    EPS_VELOCITY = 0.05
    EPS_ANGULAR = math.radians(10.0)

    def __init__(self):
        super().__init__(node_name='potential_field_controller',
                         timer_period=0.05)

    def on_goal(self) -> None:
        gx, gy = self.goal
        self.get_logger().info(f'Novo goal → ({gx:.2f}, {gy:.2f})')

    def _attractive(self) -> np.ndarray:
        gx, gy = self.goal
        return -self.KA * np.array([[self.x - gx],
                                    [self.y - gy]])

    def _repulsive(self) -> np.ndarray:
        rep = np.zeros((2, 1))

        if not self.has_laser_data():
            return rep

        angles = (self.laser_angle_min
                  + np.arange(len(self.laser_ranges))
                  * self.laser_angle_increment)
        valid  = np.isfinite(self.laser_ranges)
        rs     = self.laser_ranges[valid]
        angs   = angles[valid]

        xs = self.x + rs * np.cos(self.theta + angs)
        ys = self.y + rs * np.sin(self.theta + angs)

        for ox, oy in zip(xs, ys):
            g = (self.x - ox) ** 2 + (self.y - oy) ** 2
            if g > self.G_STAR or g < 1e-6:
                continue
            scalar = -self.KR * ((1.0 / self.G_STAR) - (1.0 / g)) / (g ** 3)
            rep += scalar * np.array([[self.x - ox],
                                      [self.y - oy]])
        return rep

    @staticmethod
    def _orientation_error(theta: float, theta_d: float) -> float:
        if (-math.pi < theta_d <= -math.pi / 2) and (math.pi / 2 < theta <= math.pi):
            theta_d += 2.0 * math.pi
        elif (-math.pi < theta <= -math.pi / 2) and (math.pi / 2 < theta_d <= math.pi):
            theta += 2.0 * math.pi
        return theta_d - theta

    def _control_loop(self) -> None:
        if not self.has_goal() or not self.has_laser_data():
            return

        att = self._attractive()
        rep = self._repulsive()
        res = att + rep

        theta_d   = float(np.arctan2(res[1, 0], res[0, 0]))
        theta_err = self._orientation_error(self.theta, theta_d)
        intensity = float(np.linalg.norm(att))

        # Goal atingido
        if intensity < self.EPS_VELOCITY:
            self.stop()
            self.clear_goal()
            self.get_logger().info('Goal atingido.')
            return

        # Corrige heading antes de avançar
        if abs(theta_err) > self.EPS_ANGULAR:
            v = 0.0
            w = float(np.clip(self.KT * theta_err, -MAX_OMEGA, MAX_OMEGA))
        else:
            v = float(np.clip(intensity, 0.0, MAX_VEL))
            w = float(np.clip(self.KT * theta_err, -MAX_OMEGA, MAX_OMEGA))

        self.publish_velocity(v, w)

        self.get_logger().debug(
            f'x={self.x:.2f} y={self.y:.2f} '
            f'θ={math.degrees(self.theta):+.1f}° '
            f'θ_d={math.degrees(theta_d):+.1f}° '
            f'err={math.degrees(theta_err):+.1f}° '
            f'v={v:.3f} ω={math.degrees(w):+.1f}°/s '
            f'|Fa|={intensity:.3f}'
        )


# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
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