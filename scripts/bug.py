#!/usr/bin/env python3.12
import math
import rclpy
from phi_p3dx_navigation.main import NavigationNode


class Bug2Controller(NavigationNode):

    FORWARD_SPEED = 0.25
    TURN_FAST     = 0.8
    TURN_SLOW     = 0.15
    TURN_GOAL     = 0.4

    DIST_OBSTACLE  = 0.35
    DIST_WALL_KEEP = 0.40
    DIST_TOO_CLOSE = 0.20
    DIST_GOAL      = 0.20
    YAW_PRECISION  = math.radians(3.0)

    LEAVE_HIT_MIN_DIFF = 0.20
    DIST_TO_LINE_PREC  = 0.10

    def __init__(self):
        super().__init__(node_name='bug2_controller', timer_period=0.05)

        self.mode = 'idle'

        self._line_m          = 0.0
        self._line_b          = 0.0
        self._line_calculated = False
        self._left_start_line = False   # FIX: garante que o robô saiu da linha antes de checar saída

        self._hit_x            = 0.0
        self._hit_y            = 0.0
        self._dist_goal_at_hit = float('inf')

    # ------------------------------------------------------------------

    def on_goal(self) -> None:
        self.mode             = 'go_to_goal'
        self._line_calculated = False
        self._left_start_line = False   # reseta a cada novo goal
        self.get_logger().info('iniciando navegação para o goal.')

    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self.has_laser_data() or not self.has_goal():
            return

        if not self._line_calculated:
            self._start_goal_line()

        if self.mode == 'go_to_goal':
            self._go_to_goal()
        elif self.mode == 'follow_wall':
            self._follow_wall()

    # ------------------------------------------------------------------

    def _start_goal_line(self) -> None:
        gx, gy = self.goal
        dx = gx - self.x
        if abs(dx) < 1e-6:
            dx = 1e-6
        self._line_m = (gy - self.y) / dx
        self._line_b = self.y - self._line_m * self.x
        self._line_calculated = True

    def _dist_to_start_goal_line(self) -> float:
        y_line = self._line_m * self.x + self._line_b
        return abs(y_line - self.y)

    # ------------------------------------------------------------------

    def _go_to_goal(self) -> None:
        if self.distance_to_goal() < self.DIST_GOAL:
            self.stop()
            self.clear_goal()
            self.mode = 'idle'
            self.get_logger().info('goal atingido!')
            return

        front = self.get_front_distance(half_angle_deg=20.0)
        if front < self.DIST_OBSTACLE:
            self._hit_x            = self.x
            self._hit_y            = self.y
            self._dist_goal_at_hit = self.distance_to_goal()
            self._left_start_line  = False   # ao entrar em follow_wall, reseta flag
            self.mode              = 'follow_wall'
            self.get_logger().info(
                f'obstáculo detectado ({front:.2f} m). Iniciando follow_wall.'
            )
            self.publish_velocity(0.0, self.TURN_FAST)
            return

        err = self.angle_to_goal()
        if abs(err) > self.YAW_PRECISION:
            w = self.TURN_GOAL if err > 0 else -self.TURN_GOAL
            self.publish_velocity(0.0, w)
        else:
            self.publish_velocity(self.FORWARD_SPEED, 0.0)

    # ------------------------------------------------------------------

    def _follow_wall(self) -> None:

        # ── Condição de saída Bug2 ─────────────────────────────────────
        # Passo 1: marca quando o robô efetivamente se afastou da linha
        if self._dist_to_start_goal_line() > self.DIST_TO_LINE_PREC * 3:
            self._left_start_line = True

        # Passo 2: só verifica saída depois de ter saído da linha ao menos uma vez
        if self._left_start_line:
            if self._dist_to_start_goal_line() < self.DIST_TO_LINE_PREC:
                dist_now = self.distance_to_goal()
                diff = self._dist_goal_at_hit - dist_now
                if diff > self.LEAVE_HIT_MIN_DIFF:
                    self.mode             = 'go_to_goal'
                    self._line_calculated = False
                    self._left_start_line = False
                    self.get_logger().info(
                        f'saindo da parede (ganho={diff:.2f} m).'
                    )
                    return

        # ── Seguir parede (parede à direita) ──────────────────────────
        n  = len(self.laser_ranges)
        lf = self.get_region_distance(int(n * 0.70), int(n * 0.85))
        f  = self.get_region_distance(int(n * 0.42), int(n * 0.58))
        rf = self.get_region_distance(int(n * 0.15), int(n * 0.30))
        r  = self.get_region_distance(0,             int(n * 0.12))

        d     = self.DIST_WALL_KEEP
        close = self.DIST_TOO_CLOSE

        if f > d and rf > d and r > d:
            # sem parede – vira direita devagar para encontrá-la
            self.publish_velocity(self.FORWARD_SPEED, -self.TURN_SLOW)
        elif f < d:
            # frente bloqueada – vira esquerda rápido
            self.publish_velocity(0.0, self.TURN_FAST)
        elif rf < close or r < close:
            # muito perto da parede – afasta virando esquerda
            self.publish_velocity(self.FORWARD_SPEED, self.TURN_SLOW)
        elif rf < d or r < d:
            # distância boa da parede – segue em frente
            self.publish_velocity(self.FORWARD_SPEED, 0.0)
        elif lf < d:
            # parede apareceu esquerda-frente – vira direita
            self.publish_velocity(self.FORWARD_SPEED, -self.TURN_SLOW)
        else:
            self.publish_velocity(self.FORWARD_SPEED, 0.0)

        self.get_logger().debug(
            f'follow_wall | lf={lf:.2f} f={f:.2f} rf={rf:.2f} r={r:.2f}'
        )


# ---------------------------------------------------------------------------

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