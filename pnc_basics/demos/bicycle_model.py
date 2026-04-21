import math
import numpy as np
import matplotlib.pyplot as plt

def bicycle_step(state, delta, a, dt, wheelbase):
    """
    Kinematic bicylce model
    state = [x, y, yaw, v]
    delta: steering wheel angle (rad)
    a: acceleration (m/s^2)
    dt: time step (s)
    wheelbase: vehicle wheelbase (m)
    """

    x, y, yaw, v = state
    x_next = x + v*math.cos(yaw)*dt
    y_next = y + v*math.sin(yaw)*dt
    yaw_next = yaw + v*math.tan(delta)/wheelbase*dt
    v_next = v + a*dt

    return np.array([x_next, y_next, yaw_next, v_next], dtype=float)

def simulation(delta_deg, total_time = 10.0, dt = 0.1, wheelbase = 2.8, v0 = 5.0, a = 0.0):
    delta = math.radians(delta_deg)
    state = np.array([0,0,0,v0],dtype=float)

    traj = [state.copy()]
    steps = int(total_time/dt)

    for _ in range(steps):
        state = bicycle_step(state, delta, a, dt, wheelbase)
        traj.append(state.copy())

    return np.array(traj)

def main():
    steering_angles = [0,5,10,15,20]
    plt.figure(figsize = (8,6))

    for angle in steering_angles:
        traj = simulation(angle)
        x = traj[:,0]
        y = traj[:,1]
        plt.plot(x,y,label=f"{angle} deg")
    
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Kinematic Bicycle Model: Trajectories under Different Steering Angles")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("bicycle_trajectories.png", dpi=200)
    plt.show()

if __name__ == "__main__":
    main()