import math
from config import LOOKAHEAD_DISTANCE, MAX_STEER


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _calc_target_index(x, y, route_x, route_y, lookahead_distance):
    distances = []
    for rx, ry in zip(route_x, route_y):
        dx = rx - x
        dy = ry - y
        distances.append(math.hypot(dx, dy))

    nearest_index = min(range(len(distances)), key=lambda i: distances[i])

    target_index = nearest_index
    while (
        target_index < len(route_x) - 1
        and distances[target_index] < lookahead_distance
    ):
        target_index += 1

    return target_index


class PurePursuitController:
    def __init__(self, wheelbase=2.8, lookahead_distance=LOOKAHEAD_DISTANCE, max_steer=MAX_STEER):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        self.max_steer = max_steer

    def control(self, ego_state, trajectory):
        route_x = trajectory["x"]
        route_y = trajectory["y"]

        if len(route_x) == 0:
            return 0.0

        x = ego_state["x"]
        y = ego_state["y"]
        yaw = ego_state["yaw"]

        target_index = _calc_target_index(
            x, y, route_x, route_y, self.lookahead_distance
        )

        tx = route_x[target_index]
        ty = route_y[target_index]

        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = _normalize_angle(alpha)

        delta = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead_distance
        )

        delta = max(-self.max_steer, min(self.max_steer, delta))
        return delta