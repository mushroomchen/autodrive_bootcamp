import carla
import time

def main():
    import carla
import time


def main():
    # 1. Connect to CARLA server
    client = carla.Client("172.24.160.1", 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # 2. Choose a vehicle blueprint
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]

    # 3. Choose a spawn point
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0]

    # 4. Spawn vehicle
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        print("Failed to spawn vehicle.")
        return

    print("Vehicle spawned:", vehicle)
    print("Spawn point:", spawn_point)
    print("Vehicle location:", vehicle.get_location())

    # 5. Move spectator above the vehicle
    spectator = world.get_spectator()

    try:
        # 5. Apply simple control for a few seconds
        for _ in range(900):
            # control = carla.VehicleControl(
            #     throttle=0.4,
            #     steer=0.0,
            #     brake=0.0
            # )
            vehicle.set_autopilot(True)
            
            vehicle_transform = vehicle.get_transform()
            location = vehicle_transform.location

            spectator.set_transform(
                carla.Transform(
                    carla.Location(x=location.x, y=location.y, z=50.0),
                    carla.Rotation(pitch=-90.0)
                )
            )

            time.sleep(0.05)

        print("Finished simple control test.")

    finally:
        print("Destroying vehicle...")
        vehicle.destroy()


if __name__ == "__main__":
    main()