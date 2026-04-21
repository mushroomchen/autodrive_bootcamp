import carla

from config import TARGET_SPEED, DT, MAX_THROTTLE, MAX_BRAKE
from carla_env.client import connect_carla, spawn_ego_vehicle, cleanup_actors
from carla_env.scene_builder import build_scene
from behavior.behavior_fsm import BehaviorFSM, SceneInput
from planning.reference_line import build_reference_lines
from planning.path_generator import generate_candidate_paths
from planning.trajectory_generator import generate_candidate_trajectories
from planning.collision_checker import check_collision
from planning.cost_evaluator import evaluate_cost
from control.pure_pursuit import PurePursuitController
from control.pid_speed_control import PIDController


def main():
    actor_list = []

    client, world, carla_map = connect_carla()
    ego_vehicle = spawn_ego_vehicle(world)
    actor_list.append(ego_vehicle)

    fsm = BehaviorFSM()
    lateral_controller = PurePursuitController()
    longitudinal_controller = PIDController(kp=0.5, ki=0.0, kd=0.1)

    try:
        while True:
            world.tick()

            obs, ego_state = build_scene(world, ego_vehicle, carla_map)

            scene = SceneInput(
                ego_speed=ego_state["speed"],
                front_vehicle_exists=obs["front_vehicle_exists"],
                front_distance=obs["front_dist"],
                front_speed=obs["front_speed"],
                lane_change_request=obs["lane_change_request"],
                left_lane_available=obs["left_lane_available"],
                need_stop=obs["need_stop"],
                lane_change_completed=obs["lane_change_done"],
            )

            behavior_state = fsm.update(scene)

            reference_lines = build_reference_lines(carla_map, ego_state, behavior_state)
            candidate_paths = generate_candidate_paths(reference_lines, behavior_state)
            candidate_trajectories = generate_candidate_trajectories(
                candidate_paths, ego_state, target_speed=TARGET_SPEED
            )

            feasible = []
            for traj in candidate_trajectories:
                if not check_collision(traj, obs):
                    cost = evaluate_cost(traj, obs, behavior_state)
                    feasible.append((traj, cost))

            if not feasible:
                control = carla.VehicleControl(throttle=0.0, brake=0.3, steer=0.0)
                ego_vehicle.apply_control(control)
                continue

            best_traj = min(feasible, key=lambda x: x[1])[0]

            steer = lateral_controller.control(ego_state, best_traj)

            speed_error = best_traj["target_speed"] - ego_state["speed"]
            u = longitudinal_controller.control(speed_error, DT)

            if u >= 0.0:
                throttle = min(u, MAX_THROTTLE)
                brake = 0.0
            else:
                throttle = 0.0
                brake = min(-u, MAX_BRAKE)

            control = carla.VehicleControl(
                throttle=float(throttle),
                brake=float(brake),
                steer=float(steer),
            )
            ego_vehicle.apply_control(control)

    finally:
        cleanup_actors(actor_list)


if __name__ == "__main__":
    main()