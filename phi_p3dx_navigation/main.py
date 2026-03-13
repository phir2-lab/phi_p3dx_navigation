#!/usr/bin/env python3.12
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class NavigationNode(Node):
    def __init__(self, node_name: str = 'navigation_node',
                 timer_period: float = 0.05):
        super().__init__(node_name)

        #odometria
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        #laser
        self.laser_ranges: np.ndarray = np.array([])
        self.laser_angle_min: float = 0.0
        self.laser_angle_max: float = 0.0
        self.laser_angle_increment: float = 0.0

        #goal recebido do RViz via /goal_pose
        self.goal: tuple | None = None   
        self.goal_theta: float = 0.0     

        #publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        #subscribers
        self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)

        self.create_subscription(
            LaserScan, '/laser_scan', self._cb_laser,
            qos_profile=qos_profile_sensor_data)

        # /goal_pose é publicado pelo botão "2D Nav Goal" do RViz2
        self.create_subscription(
            PoseStamped, '/goal_pose', self._cb_goal, 10)

        #timer de controle
        self.create_timer(timer_period, self._control_loop)

        self.get_logger().info(
            f'[{node_name}] iniciado — '
            'use "2D Nav Goal" no RViz para enviar um goal.'
        )

    def _cb_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.on_odom()

    def _cb_laser(self, msg: LaserScan) -> None:
        r = np.array(msg.ranges, dtype=float)
        self.laser_ranges = np.where(
            (r >= msg.range_min) & (r <= msg.range_max), r, np.inf
        )
        self.laser_angle_min = msg.angle_min
        self.laser_angle_max = msg.angle_max
        self.laser_angle_increment = msg.angle_increment
        self.on_laser()

    def _cb_goal(self, msg: PoseStamped) -> None:
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self.goal_theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'Novo goal → x={self.goal[0]:.2f}  '
            f'y={self.goal[1]:.2f}  θ={math.degrees(self.goal_theta):.1f}°'
        )
        self.on_goal()

    
    def on_odom(self) -> None:
        pass

    def on_laser(self) -> None:
        pass

    def on_goal(self) -> None:
        pass

    #leitura de sensores
    def get_pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.theta

    def has_laser_data(self) -> bool:
        return len(self.laser_ranges) > 0

    def get_region_distance(self, idx_start: int, idx_end: int) -> float:
        if not self.has_laser_data():
            return float('inf')
        region = self.laser_ranges[idx_start: idx_end + 1]
        return float(np.min(region)) if len(region) > 0 else float('inf')

    def get_front_distance(self, half_angle_deg: float = 15.0) -> float:
        if not self.has_laser_data():
            return float('inf')
        n      = len(self.laser_ranges)
        center = n // 2
        half   = int(math.radians(half_angle_deg)
                     / max(self.laser_angle_increment, 1e-9))
        return self.get_region_distance(
            max(0, center - half), min(n - 1, center + half)
        )

    #goal
    def has_goal(self) -> bool:
        return self.goal is not None

    def clear_goal(self) -> None:
        self.goal = None
        self.get_logger().info('Objetivo atingido.')

    def distance_to_goal(self) -> float:
        if self.goal is None:
            return float('inf')
        return math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

    def angle_to_goal(self) -> float:
        if self.goal is None:
            return 0.0
        desired = math.atan2(self.goal[1] - self.y, self.goal[0] - self.x)
        err = desired - self.theta
        return math.atan2(math.sin(err), math.cos(err))

    def publish_velocity(self, v: float, w: float) -> None:
        twist = Twist()
        twist.linear.x  = float(v)
        twist.angular.z = float(w)
        self.pub_cmd.publish(twist)

    def stop(self) -> None:
        self.publish_velocity(0.0, 0.0)

    #loop de controle
    def _control_loop(self) -> None:
        """
        Sobrescrever na subclasse com a lógica de controle.
        Executado pelo timer (padrão 20 Hz).
        """
        pass