#!/usr/bin/env python3.12
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def yaw_from_quaternion(x, y, z, w):
    return np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

L = 0.331
R = 0.09751

V_FWD    =  0.4
V_CURVE  =  0.15
W_TURN   =  np.deg2rad(40)    
W_FIND   = -np.deg2rad(25)    
WALL_TARGET  = 0.50   
KP           = 0.8    
KD           = 0.4    
W_MAX_FOLLOW = np.deg2rad(20)  

FRONT_STOP   = 0.50   
WALL_LOST    = 1.20   

IDX_FRONT_L   = 75    
IDX_FRONT_R   = 105   
IDX_FRNT_RL   = 45    
IDX_FRNT_RR   = 75    
IDX_RIGHT_L   = 0     
IDX_RIGHT_R   = 45    

FORWARD     = 'FORWARD'
TURN_LEFT   = 'TURN_LEFT'
FOLLOW_WALL = 'FOLLOW_WALL'
FIND_WALL   = 'FIND_WALL'


class WallFollowerController(Node):

    def __init__(self):
        super().__init__('wall_follower_controller')

        self.x = self.y = self.theta = 0.0

        self.d_front       = np.inf
        self.d_front_right = np.inf
        self.d_right       = np.inf

        self.state         = FORWARD
        self.prev_error    = 0.0    
        self.twist         = Twist()

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry,  '/odom',       self._cb_odom,  10)
        self.create_subscription(LaserScan, '/laser_scan', self._cb_laser, 10)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('WallFollower iniciado')


    def _cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _cb_laser(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=float)
        r = np.where(
            (r >= msg.range_min) & (r <= msg.range_max),
            r, np.inf
        )
        self.d_front       = float(np.min(r[IDX_FRONT_L  : IDX_FRONT_R  + 1]))
        self.d_front_right = float(np.min(r[IDX_FRNT_RL  : IDX_FRNT_RR  + 1]))
        self.d_right       = float(np.min(r[IDX_RIGHT_L  : IDX_RIGHT_R  + 1]))

    def _next_state(self):
        obs_front = self.d_front       < FRONT_STOP
        obs_fr    = self.d_front_right < FRONT_STOP
        wall_here = self.d_right       < WALL_LOST

        if obs_front or obs_fr:
            return TURN_LEFT

        if wall_here:
            return FOLLOW_WALL

        if self.state in (FOLLOW_WALL, FIND_WALL):
            return FIND_WALL   

        return FORWARD

    def _control_loop(self):
        self.state = self._next_state()

        if self.state == FORWARD:
            v, w = V_FWD, 0.0
            self.prev_error = 0.0   

        elif self.state == TURN_LEFT:
            v, w = 0.0, W_TURN
            self.prev_error = 0.0

        elif self.state == FOLLOW_WALL:
            v = V_FWD

            error      = self.d_right - WALL_TARGET
            d_error    = error - self.prev_error   
            self.prev_error = error

            w = float(np.clip(
                -(KP * error + KD * d_error),
                -W_MAX_FOLLOW,
                 W_MAX_FOLLOW
            ))

        elif self.state == FIND_WALL:
            v, w = V_CURVE, W_FIND
            self.prev_error = 0.0

        wr = ((2.0*v) + (w*L)) / (2.0*R)
        wl = ((2.0*v) - (w*L)) / (2.0*R)

        self.twist.linear.x  = v
        self.twist.angular.z = w
        self.pub_cmd.publish(self.twist)

        self.get_logger().info(
            f'[{self.state:11s}] '
            f'front={self.d_front:.2f} '
            f'fr={self.d_front_right:.2f} '
            f'right={self.d_right:.2f} '
            f'err={self.d_right - WALL_TARGET:+.2f} | '
            f'v={v:.2f} w={np.degrees(w):+.1f}° '
            f'wl={wl:.2f} wr={wr:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()