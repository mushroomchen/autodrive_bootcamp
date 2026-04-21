import carla
from config import CARLA_HOST, CARLA_PORT, CARLA_TIMEOUT, SYNC_MODE, FIXED_DELTA_SECONDS

def connect_carla():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(CARLA_TIMEOUT)

    world = client.get_world()
    carla_map = world.get_map()

    if SYNC_MODE:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
        world.apply_settings(settings)

    return client, world, carla_map


def spawn_ego_vehicle(world, spawn_index=0, blueprint_filter="vehicle.tesla.model3"):
    blueprint_library = world.get_blueprint_library()    
    ego_bp = blueprint_library.find(blueprint_filter)

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in current CARLA map.")
    
    spawn_point = spawn_points[spawn_index % len(spawn_points)]
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)

    if ego_vehicle is None:
        raise RuntimeError("Failed to spawn ego vehicle.")

    return ego_vehicle

def cleanup_actors(actor_list):
    for actor in actor_list:
        if actor is not None:
            actor.destroy()

        