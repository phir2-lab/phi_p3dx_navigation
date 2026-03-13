#!/usr/bin/env python3
"""
RobotController – classe base independente de algoritmo
========================================================
Centraliza:
  • callbacks de odometria e laser
  • acesso limpo aos dados dos sensores
  • publicação de velocidade (cmd_vel)
"""

import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavigationNode(Node):
    """
    Classe principal de leitura de sensores e publicação de comandos.
    Independente de algoritmo...
    """

    def __init__(self, node_name: str = 'navigation_node', timer_period: float = 0.05):
        super().__init__(node_name)

        # odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # laser
        self.laser_ranges = np.array([])
        self.laser_angle_min = 0.0
        self.laser_angle_max = 0.0
        self.laser_angle_increment = 0.0

        # publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscribers
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(LaserScan, '/laser_scan', self._cb_laser, 10)

        # timer de controle
        self.create_timer(timer_period, self._control_loop)

    # CALLBACKS 
    def _cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _cb_laser(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=float)
        self.laser_ranges = np.where(
            (r >= msg.range_min) & (r <= msg.range_max), r, np.inf
        )
        self.laser_angle_min = msg.angle_min
        self.laser_angle_max = msg.angle_max
        self.laser_angle_increment = msg.angle_increment

    # LEITURA DOS SENSORES
    def get_pose(self) -> tuple:
        return self.x, self.y, self.theta

    def get_region_distance(self, idx_start: int, idx_end: int) -> float:
        """Retorna a menor distância na faixa de índices [idx_start, idx_end]."""
        if len(self.laser_ranges) == 0:
            return float('inf')
        return float(np.min(self.laser_ranges[idx_start: idx_end + 1]))

    def has_laser_data(self) -> bool:
        return len(self.laser_ranges) > 0

    def publish_velocity(self, v: float, w: float):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    # CONTROLE – sobrescrever na subclasse 
    def _control_loop(self):
        """Sobrescrever este método na subclasse com a lógica de controle."""
        pass
