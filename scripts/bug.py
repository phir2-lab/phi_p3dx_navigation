#!/usr/bin/env python3.12
import math
import rclpy
from phi_p3dx_navigation.main import NavigationNode


class Bug2Navigator(NavigationNode):

    def __init__(self):
        super().__init__(node_name='bug2_navigator', timer_period=0.05)

        self.mode = 'idle'

        self._line_m          = 0.0
        self._line_b          = 0.0
        self._line_calculated = False
        self._left_start_line = False   

        self._hit_x            = 0.0
        self._hit_y            = 0.0
        self._dist_goal_at_hit = float('inf')

    def on_goal(self) -> None:
        self.mode             = 'go_to_goal'
        self._line_calculated = False
        self._left_start_line = False   
        self.get_logger().info('iniciando navegação para o goal.')

    def _control_loop(self) -> None:
        if not self.has_laser_data() or not self.has_goal():
            return

        if not self._line_calculated:
            self._start_goal_line()

        if self.mode == 'go_to_goal':
            self._go_to_goal()
        elif self.mode == 'follow_wall':
            self._follow_wall()

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Controller()
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