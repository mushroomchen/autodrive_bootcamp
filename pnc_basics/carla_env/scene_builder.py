import math

def _get_speed(vehicle):
    vel = vehicle.get_velocity()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)


def build_scene(world, ego_vehicle, carla_map):
    transform = ego_vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation

    ego_state = {
        "x": location.x,
        "y": location.y,
        "yaw": math.radians(rotation.yaw),
        "speed": _get_speed(ego_vehicle),
    }

    # 先给一个最小占位版本，后面再补 nearby actors / obstacle / lane change safety
    obs = {
        "front_vehicle_exists": False,
        "front_dist": 999.0,
        "front_speed": 0.0,
        "lane_change_request": False,
        "left_lane_available": True,
        "need_stop": False,
        "lane_change_done": False,
        "obstacles": [],
        "current_waypoint": carla_map.get_waypoint(location),
    }
    return obs, ego_state