import matplotlib
matplotlib.use("Agg")

import math
import numpy as np
import matplotlib.pyplot as plt

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=5.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def bicycle_step(state, delta, dt, wheelbase):
    """
    Update vehicle state using kinematic bicycle model.
    
    state: current vehicle state
    delta: steering angle (rad)
    dt: time step (s)
    wheelbase: wheelbase (m)
    """

    state.x += state.v*math.cos(state.yaw)*dt
    state.y += state.v*math.sin(state.yaw)*dt
    state.yaw += state.v*math.tan(state.yaw)/wheelbase*dt

def generate_curved_path(length=5.0, ds=0.5):
    """
    Generate a smooth curved reference path.
    """
    x = np.arange(0.0, length + ds, ds)
    y = 2.0 * np.sin(0.15 * x)
    return x, y

def compute_path_yaw(cx, cy):
    """
    Compute reference yaw angle for each path point
    """

    yaw = []
    for i in range(len(cx)-1):
        dx = cx[i+1] - cx[i]
        dy = cy[i+1] - cy[i]
        yaw.append(math.atan2(dy,dx))
    yaw.append(yaw[-1])
    return np.array(yaw)    

def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0*math.pi
    while angle < -math.pi:
        angle += 2.0*math.pi
    return angle

def calc_nearest_index(state, cx, cy):
    """
    Find nearest path point index to vehicle rear axle.
    """
    dx = cx - state.x
    dy = cy - state.y
    distances = np.hypot(dx, dy)
    nearest_index = int(np.argmin(distances))
    return nearest_index

def calc_nearest_index_local(state, cx, cy, prev_index, search_window=50):
    """
    Find nearest path point using a local search window.

    state: current vehicle state
    cx, cy: reference path
    prev_index: previous nearest index
    search_window: number of points to search forward
    """
    start = prev_index
    end = min(prev_index + search_window, len(cx))

    dx = cx[start:end] - state.x
    dy = cy[start:end] - state.y
    distances = np.hypot(dx, dy)

    local_index = int(np.argmin(distances))
    nearest_index = start + local_index

    return nearest_index

def calc_signed_cross_track_error(state, cx, cy, cyaw, nearest_index):
    """
    Compute signed cross-track error.

    Positive/negative sign is determined relative to path normal direction.
    """    
    dx = state.x - cx[nearest_index]
    dy = state.y - cy[nearest_index]

    # Path normal vector
    path_yaw = cyaw[nearest_index]
    normal_x = -math.sin(path_yaw)
    normal_y = -math.cos(path_yaw)

    e = dx*normal_x + dy*normal_y
    
    return e

def pure_pursuit_target_index(state, cx, cy, lookahead_distance):
    """
    Find look-ahead target point index for pure pursuit.
    """
    dx = cx - state.x
    dy = cy - state.y
    distances = np.hypot(dx, dy)

    nearest_index = int(np.argmin(distances))

    target_index = nearest_index
    while target_index < len(cx) - 1 and distances[target_index] < lookahead_distance:
        target_index += 1

    return target_index

def pure_pursuit_control(state, cx, cy, wheelbase, lookahead_distance):
    """
    Compute steering angle using pure pursuit.
    """
    target_index = pure_pursuit_target_index(state, cx, cy, lookahead_distance)
    tx = cx[target_index]
    ty = cy[target_index]

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
    alpha = normalize_angle(alpha)

    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead_distance)
    return delta, target_index

def stanley_control(state, cx, cy, cyaw, k, eps=1e-3):
    """
    Compute steering angle using Stanley controller.
    """
    nearest_index = calc_nearest_index(state, cx, cy)

    # Heading error
    theta_e = normalize_angle(cyaw[nearest_index] - state.yaw)

    # Cross-track error
    e_y = calc_signed_cross_track_error(state, cx, cy, cyaw, nearest_index)

    # Stanley control law
    delta = theta_e + math.atan2(k * e_y, state.v + eps)
    delta = normalize_angle(delta)

    return delta, nearest_index, e_y

def simulate_pure_pursuit(cx, cy, cyaw, total_time=15.0, dt=0.1):
    """
    Simulate path tracking using pure pursuit.
    """
    wheelbase = 2.8
    lookahead_distance = 3.0

    state = State(x=0.0, y=-2.0, yaw=0.0, v=5.0)

    x_hist = []
    y_hist = []
    e_hist = []

    steps = int(total_time / dt)

    for _ in range(steps):
        delta, target_index = pure_pursuit_control(
            state, cx, cy, wheelbase, lookahead_distance
        )

        bicycle_step(state, delta, dt, wheelbase)

        nearest_index = calc_nearest_index(state, cx, cy)
        e_y = calc_signed_cross_track_error(state, cx, cy, cyaw, nearest_index)

        x_hist.append(state.x)
        y_hist.append(state.y)
        e_hist.append(e_y)

    return np.array(x_hist), np.array(y_hist), np.array(e_hist)

def simulate_stanley(cx, cy, cyaw, total_time=15.0, dt=0.1):
    """
    Simulate path tracking using Stanley controller.
    """
    wheelbase = 2.8
    k = 2.0

    state = State(x=0.0, y=-2.0, yaw=0.0, v=5.0)

    x_hist = []
    y_hist = []
    e_hist = []

    steps = int(total_time / dt)

    for _ in range(steps):
        delta, nearest_index, e_y = stanley_control(
            state, cx, cy, cyaw, k=k
        )

        bicycle_step(state, delta, dt, wheelbase)

        x_hist.append(state.x)
        y_hist.append(state.y)
        e_hist.append(e_y)

    return np.array(x_hist), np.array(y_hist), np.array(e_hist)

def main():
    cx, cy = generate_curved_path()
    cyaw = compute_path_yaw(cx, cy)

    pp_x, pp_y, pp_e = simulate_pure_pursuit(cx, cy, cyaw)
    st_x, st_y, st_e = simulate_stanley(cx, cy, cyaw)

    t_pp = np.arange(len(pp_e)) * 0.1
    t_st = np.arange(len(st_e)) * 0.1

    plt.figure(figsize=(10, 8))

    plt.subplot(2, 1, 1)
    plt.plot(cx, cy, "--", label="Reference path")
    plt.plot(pp_x, pp_y, label="Pure Pursuit")
    plt.plot(st_x, st_y, label="Stanley")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Path Tracking Comparison")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(t_pp, pp_e, label="Pure Pursuit lateral error")
    plt.plot(t_st, st_e, label="Stanley lateral error")
    plt.xlabel("Time (s)")
    plt.ylabel("Lateral error (m)")
    plt.title("Lateral Error Comparison")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig("stanley_vs_pure_pursuit.png", dpi=200)
    plt.show()


if __name__ == "__main__":
    main()