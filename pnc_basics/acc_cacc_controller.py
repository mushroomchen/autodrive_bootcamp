import numpy as np
import matplotlib.pyplot as plt

class ACCController:
    def __init__(self, kp: float, kv: float, d0: float, h: float):
        """
        ACC Controller

        kp: spacing error gain
        kv: relative speed gain
        d0: standstill distance
        h: time headway
        """
        self.kp = kp
        self.kv = kv
        self.d0 = d0
        self.h = h

    def desired_spacing(self, v_ego: float) -> float:
        """
        Compute desired spacing using constant time headway policy
        """
        return self.d0 + self.h*v_ego

    def control(self, d: float, v_ego: float, v_lead: float) -> float:
        """
        Compute acceleration command                        
        d: actual spacing
        v_ego: ego vehicle speed
        v_lead: lead vehicle speed                  
        """                                                                                                                                                                                                         
        d_des = self.desired_spacing(v_ego)
        e = d - d_des
        dv = v_lead - v_ego

        u = self.kp*e + self.kv*dv
        return u


class CACCController:
    def __init__(self, kp: float, kv: float, ka: float, d0: float, h: float):
        """
        CACC controller

        kp: spacing error gain
        kv: relative speed gain
        ka: lead acceleration feedforward gain
        d0: standstil distance
        h: time headway
        """

        self.kp = kp
        self.kv = kv
        self.ka = ka
        self.d0 = d0
        self.h = h

    def desired_spacing(self, v_ego: float) -> float:
        """
        Compute desored spacing using constant time headway policy
        """
        return self.d0 + self.h* v_ego
    
    def control(self, d: float, v_ego: float, v_lead: float, a_lead: float) -> float:
        """
        Compute acceleration command

        d: actual spacing
        v_ego: ego vehicle speed
        v_lead: lead vehicle speed
        a_lead: lead vehicle acceleration (available via communication)
        """

        d_des = self. desired_spacing(v_ego)
        e = d - d_des
        dv = v_lead - v_ego

        u = self.kp*e + self.kv*dv + self.ka*a_lead
        return u

def lead_vehicle_profile(t: float):
    """
    Define a simple lead vehicle motion profile

    Returns:
        v_lead, a_lead
    """

    if t < 2.0:
        a_lead = 0.0
        v_lead = 15.0 
    elif t < 6:
        a_lead = -1.5
        v_lead = 15 + (-1.5)*(t-2)
    elif t < 10:
        a_lead = 1
        v_lead = 9 + 1*(t-6)
    else:
        a_lead = 0
        v_lead = 13
    return v_lead, a_lead

def simulate(controller, controller_type="ACC", total_time=15, dt = 0.02):
    """
    Simulate follower vehicle longitudinal dynamics

    State:
        x_ego, v_ego
        x_lead, v_lead
    """
    times = np.arange(0.0, total_time + dt, dt)

    # Initial conditions
    x_lead = 30
    v_lead = 15

    x_ego = 0
    v_ego = 12
 
    x_lead_hist = []
    v_lead_hist = []
    a_lead_hist = []

    x_ego_hist = []
    v_ego_hist = []
    a_cmd_hist = []
    spacing_hist = []
    spacing_des_hist = []   

    for t in times:
        v_lead, a_lead = lead_vehicle_profile(t)

        # Update lead vehicle
        x_lead = x_lead + v_lead * dt

        # Current spacing
        d = x_lead - x_ego

        # Controller output
        if controller_type == "ACC":
            a_cmd = controller.control(d=d, v_ego=v_ego, v_lead = v_lead)
            d_des = controller.desired_spacing(v_ego)
        elif controller_type == "CACC":
            a_cmd = controller.control(d=d, v_ego=v_ego, v_lead = v_lead, a_lead = a_lead)
            d_des = controller.desired_spacing(v_ego)

        else:
            raise ValueError("controller_type must be 'ACC' or 'CACC' ")
        
        # Simple saturation
        a_cmd = np.clip(a_cmd, -4, 2.0)

        # Ego vehicle longitidinal dynamics
        v_ego = v_ego + a_cmd*dt
        v_ego = max(v_ego, 0.0)
        x_ego = x_ego + v_ego*dt

        # Save history
        x_lead_hist.append(x_lead)
        v_lead_hist.append(v_lead)
        a_lead_hist.append(a_lead)

        x_ego_hist.append(x_ego)
        v_ego_hist.append(v_ego)
        a_cmd_hist.append(a_cmd)
        spacing_hist.append(d)
        spacing_des_hist.append(d_des)

    return {
        "time": times,
        "x_lead": np.array(x_lead_hist),
        "v_lead": np.array(v_lead_hist),
        "a_lead": np.array(a_lead_hist),
        "x_ego": np.array(x_ego_hist),
        "v_ego": np.array(v_ego_hist),
        "a_cmd": np.array(a_cmd_hist),
        "spacing": np.array(spacing_hist),
        "spacing_des": np.array(spacing_des_hist),
    }        


def plot_results(acc_result, cacc_result):
    t = acc_result["time"]

    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, acc_result["v_lead"], label="Lead speed", linestyle="--")
    plt.plot(t, acc_result["v_ego"], label="ACC ego speed")
    plt.plot(t, cacc_result["v_ego"], label="CACC ego speed")
    plt.ylabel("Speed (m/s)")
    plt.title("ACC vs CACC Speed Tracking")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(t, acc_result["spacing"], label="ACC actual spacing")
    plt.plot(t, acc_result["spacing_des"], label="ACC desired spacing", linestyle="--")
    plt.plot(t, cacc_result["spacing"], label="CACC actual spacing")
    plt.ylabel("Spacing (m)")
    plt.title("Spacing Response")
    plt.grid(True)
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, acc_result["a_cmd"], label="ACC acceleration command")
    plt.plot(t, cacc_result["a_cmd"], label="CACC acceleration command")
    plt.xlabel("Time (s)")
    plt.ylabel("Accel (m/s^2)")
    plt.title("Control Command")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig("acc_cacc_demo.png", dpi=200)
    plt.show()

def main():
    acc = ACCController(
        kp = 0.35,
        kv = 0.8,
        d0 = 8.0,
        h=1.2,
    )

    cacc = CACCController(
        kp=0.30,
        kv=0.8,
        ka=0.9,
        d0=8.0,
        h=1.2,       
    )

    acc_result = simulate(acc, controller_type="ACC")
    cacc_result = simulate(cacc, controller_type="CACC")

    plot_results(acc_result, cacc_result)

if __name__ == "__main__":
    main()