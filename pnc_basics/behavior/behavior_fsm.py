from enum import Enum, auto


class BehaviorState(Enum):
    KEEP_LANE = auto()
    FOLLOW = auto()
    LANE_CHANGE = auto()
    STOP = auto()


class SceneInput:
    def __init__(
        self,
        ego_speed: float,
        front_vehicle_exists: bool,
        front_distance: float,
        front_speed: float,
        lane_change_request: bool,
        left_lane_available: bool,
        need_stop: bool,
        lane_change_completed: bool,
    ):
        self.ego_speed = ego_speed
        self.front_vehicle_exists = front_vehicle_exists
        self.front_distance = front_distance
        self.front_speed = front_speed
        self.lane_change_request = lane_change_request
        self.left_lane_available = left_lane_available
        self.need_stop = need_stop
        self.lane_change_completed = lane_change_completed


class BehaviorFSM:
    def __init__(self):
        self.state = BehaviorState.KEEP_LANE

        # thresholds
        self.follow_distance_threshold = 20.0
        self.clear_distance_threshold = 25.0
        self.slow_vehicle_speed_threshold = 5.0

    def update(self, scene: SceneInput) -> BehaviorState:
        """
        Update FSM state based on current scene input.
        """
        if self.state == BehaviorState.KEEP_LANE:
            if scene.need_stop:
                self.state = BehaviorState.STOP

            elif (
                scene.lane_change_request
                and scene.left_lane_available
            ):
                self.state = BehaviorState.LANE_CHANGE

            elif (
                scene.front_vehicle_exists
                and scene.front_distance < self.follow_distance_threshold
            ):
                self.state = BehaviorState.FOLLOW

            else:
                self.state = BehaviorState.KEEP_LANE

        elif self.state == BehaviorState.FOLLOW:
            if scene.need_stop:
                self.state = BehaviorState.STOP

            elif (
                scene.lane_change_request
                and scene.left_lane_available
            ):
                self.state = BehaviorState.LANE_CHANGE

            elif (
                (not scene.front_vehicle_exists)
                or scene.front_distance > self.clear_distance_threshold
            ):
                self.state = BehaviorState.KEEP_LANE

            else:
                self.state = BehaviorState.FOLLOW

        elif self.state == BehaviorState.LANE_CHANGE:
            if scene.need_stop:
                self.state = BehaviorState.STOP

            elif scene.lane_change_completed:
                if (
                    scene.front_vehicle_exists
                    and scene.front_distance < self.follow_distance_threshold
                ):
                    self.state = BehaviorState.FOLLOW
                else:
                    self.state = BehaviorState.KEEP_LANE

            else:
                self.state = BehaviorState.LANE_CHANGE

        elif self.state == BehaviorState.STOP:
            if scene.need_stop:
                self.state = BehaviorState.STOP

            else:
                if (
                    scene.front_vehicle_exists
                    and scene.front_distance < self.follow_distance_threshold
                ):
                    self.state = BehaviorState.FOLLOW
                else:
                    self.state = BehaviorState.KEEP_LANE

        return self.state


def demo():
    fsm = BehaviorFSM()

    scenarios = [
        SceneInput(
            ego_speed=10.0,
            front_vehicle_exists=False,
            front_distance=100.0,
            front_speed=0.0,
            lane_change_request=False,
            left_lane_available=False,
            need_stop=False,
            lane_change_completed=False,
        ),
        SceneInput(
            ego_speed=10.0,
            front_vehicle_exists=True,
            front_distance=15.0,
            front_speed=6.0,
            lane_change_request=False,
            left_lane_available=False,
            need_stop=False,
            lane_change_completed=False,
        ),
        SceneInput(
            ego_speed=8.0,
            front_vehicle_exists=True,
            front_distance=12.0,
            front_speed=4.0,
            lane_change_request=True,
            left_lane_available=True,
            need_stop=False,
            lane_change_completed=False,
        ),
        SceneInput(
            ego_speed=8.0,
            front_vehicle_exists=True,
            front_distance=30.0,
            front_speed=8.0,
            lane_change_request=True,
            left_lane_available=True,
            need_stop=False,
            lane_change_completed=True,
        ),
        SceneInput(
            ego_speed=6.0,
            front_vehicle_exists=False,
            front_distance=100.0,
            front_speed=0.0,
            lane_change_request=False,
            left_lane_available=False,
            need_stop=True,
            lane_change_completed=False,
        ),
        SceneInput(
            ego_speed=0.0,
            front_vehicle_exists=False,
            front_distance=100.0,
            front_speed=0.0,
            lane_change_request=False,
            left_lane_available=False,
            need_stop=False,
            lane_change_completed=False,
        ),
    ]

    for i, scene in enumerate(scenarios):
        state = fsm.update(scene)
        print(f"Step {i}: {state.name}")


if __name__ == "__main__":
    demo()