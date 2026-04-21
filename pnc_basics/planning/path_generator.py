def generate_candidate_paths(reference_lines, behavior_state):
    return [
        {
            "x": [0.0, 5.0, 10.0],
            "y": [0.0, 0.0, 0.0],
            "target_lane": "current",
        }
    ]