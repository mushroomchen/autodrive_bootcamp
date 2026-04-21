def generate_candidate_trajectories(candidate_paths, ego_state, target_speed=8.0):
    trajectories = []
    for path in candidate_paths:
        traj = {
            "x": path["x"],
            "y": path["y"],
            "yaw": [ego_state["yaw"]] * len(path["x"]),
            "speed": [target_speed] * len(path["x"]),
            "target_speed": target_speed,
        }
        trajectories.append(traj)
    return trajectories