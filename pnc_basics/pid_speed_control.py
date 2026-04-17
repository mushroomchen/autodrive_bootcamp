import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def control(self, error:float, dt:float) -> float:
        """
        Compute PID control output
        """
        self.integral += error*dt
        derivative = (error - self.prev_error)/dt
        self.prev_error = error

        u = self.kp*error + self.ki*self.integral + self.kd*derivative    
        return u
    
def speed_reference(t:float)-> float:
    """
    Step input
        0 m/s before 1 s, 10 m/s after 1 s.
    """
    return 0.0 if t < 1.0 else 10.0

def simulate(total_time = 15.0, dt = 0.01):
    """
    Longitudinal vehicle model:
        dv/dt = u - c * v
    """
    c = 0.2  # damping / resistance coefficient
    pid = PIDController(kp=2.0,ki=0.8,kd=0.2)
    times = np.arange(0.0, total_time+dt, dt)
    v=0.0
    speed_hist = []
    ref_hist = []
    control_hist = []

    for t in times:
        v_ref = speed_reference(t)
        error = v_ref - v
        u = pid.control(error, dt)

        # vehicle dynamics
        v_dot = u - c*v
        v = v+v_dot*dt

        speed_hist.append(v)
        ref_hist.append(v_ref)
        control_hist.append(u)

    return times, np.array(ref_hist), np.array(speed_hist), np.array(control_hist)

def compute_overshoot(ref: np.ndarray, y: np.ndarray) -> float:
    """
    Compute overshoot after the step reaches its final value.
    Assumes final reference is constant and positive.
    """

    final_value = ref[-1]
    peak_value = np.max(y)

    if peak_value <= final_value:
        return 0.0

    overshoot_percent = (peak_value - final_value)/final_value*100.0
    return overshoot_percent

def compute_settling_time(times: np.ndarray, ref: np.ndarray, y: np.ndarray, tol=0.02):
    """
    Settling time using ±tol band around final reference.
    tol=0.02 means ±2%.
    """
    final_value = ref[-1]
    lower = final_value*(1.0-tol)
    upper = final_value*(1.0+tol)

    for i in range(len(times)):
        remaining = y[i:]
        if np.all((remaining >= lower) & (remaining <= upper)):
            return times[i]
    
    return None

def main():
    times, ref, speed, control = simulate()

    overshoot = compute_overshoot(ref, speed)
    settling_time = compute_settling_time(times, ref, speed, tol=0.02)
    print(f"Overshoot:{overshoot:.2f}%")
    if settling_time is None:
        print("Setting time: not settled within simulation horizon")
    else:
        print(f"Setting time (2% band): {settling_time: .2f} s")

    plt.figure(figsize =(8,5))
    plt.plot(times, ref, label="Reference speed")
    plt.plot(times, speed, label="Actual Speed")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (s)")
    plt.title("PID Speed Tracking Step Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("pid_speed_step_response.png", dpi = 200)
    plt.show()

if __name__ == "__main__":
    main()