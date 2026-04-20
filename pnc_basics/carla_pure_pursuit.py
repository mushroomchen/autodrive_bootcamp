import carla
import math
import time
import numpy as np
import matplotlib.pyplot as plt

def get_speed(vehicle):
    """
    Compute vehicle speed magniture in m/s
    """

    v = vehicle.get_velocity()
    return math.sqrt(v.x**2+v.y**2+v.z**2)

def build_route(carla_map, start_location, step_distance=2.0, num_points = 150):
    """
    Build a simple waypoint route by repeatedly calling waypoint.next().

    carla_map: CARLA map object
    start_location: starting carla.Location
    step_distance: spacing between consecutive waypoints
    num_points: maximum number of route points
    """

    route_x = []
    route_y = []

    waypoint = carla_map.get_waypoint(start_location)
    
    if waypoint is None:
        raise RuntimeError("Failed to get a valid driving waypoint from start_location.")

    for _ in range(num_points):
        route_x.append(waypoint.transform.location.x)
        route_y.append(waypoint.transform.location.y)

        next_wps = waypoint.next(step_distance)
        if len(next_wps) == 0:
            break
        
        # For the first demo, just choose the first continuation
        waypoint = next_wps[0]

        if waypoint is None:
            break
    
    return np.array(route_x), np.array(route_y)

def calc_target_index(x, y, route_x, route_y, lookahead_distance):
    """
    Find the target point index on the route for the pure pursuit.
    """
    dx = route_x -x
    dy = route_y -y
    distances = np.hypot(dx,dy)

    nearest_index = int(np.argmin(distances))
    
    target_index = nearest_index
    while target_index < len(route_x) - 1 and distances[target_index] < lookahead_distance:
        target_index += 1
    
    return target_index

def pure_pursuit_control(x, y, yaw, route_x, route_y, wheelbase, lookahead_distance):
    """
    Compute desired steering angle delta (in radians) using pure pursuit.
    """
    target_index = calc_target_index(x, y, route_x, route_y, lookahead_distance)

    tx = route_x[target_index]
    ty = route_y[target_index]

    alpha = math.atan2(ty - y, tx - x) - yaw

    # Normalize alpha to [-pi, pi]
    while alpha > math.pi:
        alpha -= 2.0 * math.pi
    while alpha < -math.pi:
        alpha += 2.0 * math.pi

    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead_distance)

    return delta, target_index, tx, ty

def speed_control(target_speed, current_speed, kp=0.25):
    """
    Very simple P controller for longitudinal speed.
    """
    error = target_speed - current_speed

    if error >= 0.0:
        throttle = np.clip(kp * error, 0.0, 0.6)
        brake = 0.0
    else:
        throttle = 0.0
        brake = np.clip(-0.15 * error, 0.0, 0.5)

    return throttle, brake

def main():
    # ------------------------------------------------------------------
    # 1. Connect to CARLA
    # ------------------------------------------------------------------
    client = carla.Client("172.24.160.1", 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()   

    #### clean old vehicles
    actors = world.get_actors().filter("vehicle.*")
    print(f"Found {len(actors)} existing vehicles. Destroying them...")

    for actor in actors:
        actor.destroy()
    #######

    # ------------------------------------------------------------------
    # 2. Spawn vehicle
    # ------------------------------------------------------------------
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_points = carla_map.get_spawn_points()

    print("Current map:", world.get_map().name)
    print("Number of spawn points:", len(carla_map.get_spawn_points()))

    vehicle = None
    spawn_point = None

    for sp in spawn_points:
        vehicle = world.try_spawn_actor(vehicle_bp, sp)
        if vehicle is not None:
            spawn_point = sp
            break

    if vehicle is None:
        print("Failed to spawn vehicle.")
        return

    print("Vehicle spawned successfully.")
    print("Spawn point:", spawn_point)

    # ------------------------------------------------------------------
    # 3. Get max steering angle (for rad -> normalized steer mapping)
    # ------------------------------------------------------------------
    physics_control = vehicle.get_physics_control()
    max_steer_deg = physics_control.wheels[0].max_steer_angle
    max_steer_rad = math.radians(max_steer_deg)

    print(f"Max steer angle: {max_steer_deg:.2f} deg")

    # ------------------------------------------------------------------
    # 4. Build route from current map
    # ------------------------------------------------------------------
    vehicle_location = vehicle.get_location()

    route_x, route_y = build_route(
        carla_map,
        vehicle_location,
        step_distance=2.0,
        num_points=150
    )

    # Draw route in CARLA world for visualization
    for i in range(len(route_x)):
        world.debug.draw_point(
            carla.Location(x=float(route_x[i]), y=float(route_y[i]), z=0.5),
            size=0.08,
            color=carla.Color(255, 0, 0),
            life_time=30.0
        )

    # ------------------------------------------------------------------
    # 5. Control parameters
    # ------------------------------------------------------------------
    wheelbase = 2.8
    lookahead_distance = 5.0
    target_speed = 8.0  # m/s

    spectator = world.get_spectator()

    x_hist = []
    y_hist = []
    target_x_hist = []
    target_y_hist = []

    duration = 25.0
    dt = 0.05
    steps = int(duration / dt)
     
    try:
        for _ in range(steps):
            # ----------------------------------------------------------
            # Read current vehicle state
            # ----------------------------------------------------------
            transform = vehicle.get_transform()
            location = transform.location
            yaw = math.radians(transform.rotation.yaw)
            speed = get_speed(vehicle)

            x = location.x
            y = location.y

            # ----------------------------------------------------------
            # Pure pursuit lateral control
            # ----------------------------------------------------------
            delta, target_index, tx, ty = pure_pursuit_control(
                x=x,
                y=y,
                yaw=yaw,
                route_x=route_x,
                route_y=route_y,
                wheelbase=wheelbase,
                lookahead_distance=lookahead_distance
            )

            # Convert steering angle (rad) to CARLA normalized steer [-1, 1]
            steer_cmd = np.clip(delta / max_steer_rad, -1.0, 1.0)

            # ----------------------------------------------------------
            # Longitudinal speed control
            # ----------------------------------------------------------
            throttle_cmd, brake_cmd = speed_control(target_speed, speed, kp=0.25)

            control = carla.VehicleControl(
                throttle=float(throttle_cmd),
                steer=float(steer_cmd),
                brake=float(brake_cmd)
            )
            vehicle.apply_control(control)

            # ----------------------------------------------------------
            # Move spectator above vehicle
            # ----------------------------------------------------------
            spectator.set_transform(
                carla.Transform(
                    carla.Location(x=x, y=y, z=50.0),
                    carla.Rotation(pitch=-90.0)
                )
            )

            # Save history
            x_hist.append(x)
            y_hist.append(y)
            target_x_hist.append(tx)
            target_y_hist.append(ty)

            time.sleep(dt)

        print("Finished pure pursuit test.")

    finally:
        # ------------------------------------------------------------------
        # 6. Save trajectory plot
        # ------------------------------------------------------------------
        plt.figure(figsize=(8, 6))
        plt.plot(route_x, route_y, "--", label="Reference route")
        plt.plot(x_hist, y_hist, label="Vehicle trajectory")
        plt.scatter(target_x_hist[::10], target_y_hist[::10], s=20, label="Look-ahead points")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("CARLA Pure Pursuit Tracking")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig("carla_pure_pursuit_tracking.png", dpi=200)
        print("Saved figure: carla_pure_pursuit_tracking.png")

        print("Destroying vehicle...")
        vehicle.destroy()

if __name__ == "__main__":
    main()