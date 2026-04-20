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

def build_route(carla_map, start_location, step_distance=2.0, num_plots = 150):
    """
    Build a simple waypoint route by repeatedly calling waypoint.next()
    """
    route_x = []
    route_y = []

    waypiont = carla_map.get_waypoint(start_location)

    for _ in range(num_points):
        route_x.append(waypoint.transform.location.x)
        route_y.append(waypoint.transform.location.y)

        next_wps = waypoint.next(step_distance)
        if len(next_wps) == 0:
            break
        
        # For the first demo, just choose the first continuation
        waypoint = next_wps[0]
    
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

