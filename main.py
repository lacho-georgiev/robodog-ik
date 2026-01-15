import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import serial
import time

# ----------------------------
# Hardware config (your values)
# ----------------------------
HARDWARE_ENABLED = True
SERIAL_PORT = "/dev/cu.usbserial-2110"
BAUD = 115200

# PCA channels used
CH0 = 0  # upper servo
CH1 = 1  # lower servo

# Your pulse limits and zeros
US0_CH0 = 1500
US_MIN_CH0 = 500
US_MAX_CH0 = 1600

US0_CH1 = 1500
US_MIN_CH1 = 1300
US_MAX_CH1 = 2500

# Initial gain guess (tune later if needed)
K_US_PER_RAD_CH0 = 636.62
K_US_PER_RAD_CH1 = 636.62


# ----------------------------
# Utilities
# ----------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_pi(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a

def angdist(a: float, b: float) -> float:
    return wrap_pi(a - b)

def ang_in_range_deg_std(theta_std: float, lo_deg: float, hi_deg: float) -> bool:
    """Angle range check on standard angle (0=+x), wrapped to [0,360)."""
    a = (math.degrees(wrap_pi(theta_std)) % 360.0)
    lo = lo_deg % 360.0
    hi = hi_deg % 360.0
    if lo <= hi:
        return lo <= a <= hi
    return a >= lo or a <= hi


# ----------------------------
# Angle → PWM mapping
# ----------------------------
def theta_std_to_servo(theta_std: float) -> float:
    """
    Standard: 0 rad = +x
    Your servo "zero": pointing to -x, which is pi rad in standard.
    So servo angle is theta_std - pi.
    """
    return wrap_pi(theta_std - math.pi)

def theta_servo_to_us(theta_servo: float, us0: int, k_us_per_rad: float, usmin: int, usmax: int) -> int:
    us = us0 + k_us_per_rad * theta_servo
    us = clamp(us, usmin, usmax)
    return int(round(us))


# ----------------------------
# Upper servo 2-link IK (O1 horn + rod to reach B)
# ----------------------------
def two_link_servo_ik(O: Tuple[float, float], r_horn: float, L_rod: float, P: Tuple[float, float]) -> List[float]:
    ox, oy = O
    px, py = P[0] - ox, P[1] - oy
    d = math.hypot(px, py)
    if d < 1e-9:
        return []

    if d > (r_horn + L_rod) + 1e-6:
        return []
    if d < abs(L_rod - r_horn) - 1e-6:
        return []

    gamma = math.atan2(py, px)
    cos_alpha = (r_horn*r_horn + d*d - L_rod*L_rod) / (2.0 * r_horn * d)
    cos_alpha = clamp(cos_alpha, -1.0, 1.0)
    alpha = math.acos(cos_alpha)
    return [wrap_pi(gamma + alpha), wrap_pi(gamma - alpha)]


# ----------------------------
# Mechanism model:
# lower actuator is rigid O2->C of 145 mm
# ----------------------------
@dataclass
class Config:
    theta1_std: float
    theta2_std: float
    phi: float
    A: Tuple[float, float]
    B: Tuple[float, float]
    C: Tuple[float, float]
    D: Tuple[float, float]

class Mechanism:
    def __init__(self):
        # fixed anchors
        self.O1 = (0.0, 0.0)
        self.O2 = (0.0, -20.0)

        # lengths (mm)
        self.r1 = 25.0
        self.L1 = 145.0          # AB rod
        self.L_O2C = 145.0       # rigid lower actuator

        self.LBD = 140.0
        self.LCD = 112.0

        # Force the upper servo branch (your chosen side)
        self.theta1_std_range_deg = (80.0, 200.0)

        self.phi_pref = -math.pi / 2

    def _points_from_phi(self, D: Tuple[float, float], phi: float):
        xD, yD = D
        ux, uy = math.cos(phi), math.sin(phi)
        B = (xD - self.LBD * ux, yD - self.LBD * uy)
        C = (xD - self.LCD * ux, yD - self.LCD * uy)
        return B, C

    def _g_phi(self, D: Tuple[float, float], phi: float) -> float:
        # lower constraint: |O2C|^2 - L^2 = 0
        _, C = self._points_from_phi(D, phi)
        cx, cy = C
        ox, oy = self.O2
        return (cx-ox)**2 + (cy-oy)**2 - self.L_O2C**2

    def _find_phi_roots(self, D: Tuple[float, float]) -> List[float]:
        # scan and bisection
        roots: List[float] = []
        N = 720
        phis = [-math.pi + 2*math.pi*i/N for i in range(N+1)]
        gs = [self._g_phi(D, p) for p in phis]

        for i in range(N):
            p0, p1 = phis[i], phis[i+1]
            g0, g1 = gs[i], gs[i+1]
            if g0 == 0.0:
                roots.append(p0)
                continue
            if g0 * g1 < 0.0:
                a, b = p0, p1
                fa, fb = g0, g1
                for _ in range(50):
                    m = 0.5*(a+b)
                    fm = self._g_phi(D, m)
                    if abs(fm) < 1e-6:
                        a = b = m
                        break
                    if fa * fm < 0.0:
                        b, fb = m, fm
                    else:
                        a, fa = m, fm
                roots.append(0.5*(a+b))

        roots = sorted([wrap_pi(r) for r in roots])
        dedup: List[float] = []
        for r in roots:
            if not dedup or abs(angdist(r, dedup[-1])) > 1e-2:
                dedup.append(r)
        return dedup

    def solve_ik_for_D(self, D: Tuple[float, float], prev: Optional[Config]) -> Optional[Config]:
        phi_roots = self._find_phi_roots(D)
        if not phi_roots:
            return None

        best = None
        best_cost = float("inf")

        for phi in phi_roots:
            B, C = self._points_from_phi(D, phi)

            # lower servo standard angle is the direction O2->C
            cx, cy = C
            ox2, oy2 = self.O2
            theta2_std = wrap_pi(math.atan2(cy - oy2, cx - ox2))

            # upper servo candidates (2-link)
            th1_list = two_link_servo_ik(self.O1, self.r1, self.L1, B)
            if not th1_list:
                continue

            for theta1_std in th1_list:
                if not ang_in_range_deg_std(theta1_std, *self.theta1_std_range_deg):
                    continue

                ax = self.O1[0] + self.r1 * math.cos(theta1_std)
                ay = self.O1[1] + self.r1 * math.sin(theta1_std)
                A = (ax, ay)

                if prev is None:
                    cost = (angdist(phi, self.phi_pref)**2)
                else:
                    cost = (
                        5.0*(angdist(theta1_std, prev.theta1_std)**2) +
                        3.0*(angdist(theta2_std, prev.theta2_std)**2) +
                        1.0*(angdist(phi, prev.phi)**2)
                    )

                if cost < best_cost:
                    best_cost = cost
                    best = Config(theta1_std=theta1_std, theta2_std=theta2_std, phi=phi,
                                  A=A, B=B, C=C, D=D)
        return best


# ----------------------------
# Serial transport (Arduino receiver expects: g us0 us1 ms)
# ----------------------------
class ServoHW:
    def __init__(self):
        self.ser = None
        self.connected = False
        if HARDWARE_ENABLED:
            try:
                self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
                time.sleep(2.0)  # let Arduino reboot
                self.connected = True
            except serial.SerialException:
                self.ser = None
                self.connected = False

    def send_smooth(self, us0: int, us1: int, ms: int):
        if not self.ser:
            return
        self.ser.write(f"g {us0} {us1} {ms}\n".encode("ascii"))

    def home(self):
        if not self.ser:
            return
        self.ser.write(b"home\n")


# ----------------------------
# Simulator + real control
# ----------------------------
class Simulator:
    def __init__(self):
        self.mech = Mechanism()
        self.hw = ServoHW()

        self.cfg = self.mech.solve_ik_for_D(D=(-120.0, -180.0), prev=None)
        if self.cfg is None:
            raise RuntimeError("Initial D is not reachable.")

        self.target_D = self.cfg.D
        self.animating = False

        # plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True)

        # ---- VISUAL STYLE (thicker, more visible) ----
        LW_LINK = 6.0          # link line width
        LW_GRID = 1.0
        MS_JOINT = 10          # joint marker size
        MS_TARGET = 12         # target "x" size
        MEW_TARGET = 3.0       # target "x" line thickness

        self.ax.grid(True, linewidth=LW_GRID)

        (self.line_O1A,) = self.ax.plot([], [], linewidth=LW_LINK)
        (self.line_AB,) = self.ax.plot([], [], linewidth=LW_LINK)
        (self.line_O2C,) = self.ax.plot([], [], linewidth=LW_LINK)
        (self.line_BD,) = self.ax.plot([], [], linewidth=LW_LINK)

        (self.pt_O1,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)
        (self.pt_O2,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)
        (self.pt_A,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)
        (self.pt_B,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)
        (self.pt_C,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)
        (self.pt_D,) = self.ax.plot([], [], marker="o", markersize=MS_JOINT)

        (self.pt_target,) = self.ax.plot(
            [], [], marker="x", markersize=MS_TARGET, markeredgewidth=MEW_TARGET
        )

        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

        self.ax.set_xlim(-250, 150)
        self.ax.set_ylim(-320, 80)

        self.serial_status_text = None
        if not self.hw.connected:
            self.serial_status_text = self.ax.text(
                0.02,
                0.98,
                "SERIAL NOT CONNECTED",
                transform=self.ax.transAxes,
                va="top",
                ha="left",
                fontsize=12,
                color="red",
                bbox=dict(facecolor="white", edgecolor="red", alpha=0.8),
            )

        self.update_draw(self.cfg)
        self.update_title(self.cfg)

        # go to home on hardware, then to initial pose
        self.hw.home()
        self._send_cfg_to_hw(self.cfg, smooth_ms=400)

    def _cfg_to_us(self, cfg: Config) -> Tuple[int, int]:
        # Convert IK "standard angles" to your servo convention: 0 = -x
        t1_servo = theta_std_to_servo(cfg.theta1_std)
        t2_servo = theta_std_to_servo(cfg.theta2_std)

        us0 = theta_servo_to_us(t1_servo, US0_CH0, K_US_PER_RAD_CH0, US_MIN_CH0, US_MAX_CH0)
        us1 = theta_servo_to_us(t2_servo, US0_CH1, K_US_PER_RAD_CH1, US_MIN_CH1, US_MAX_CH1)
        return us0, us1

    def _send_cfg_to_hw(self, cfg: Config, smooth_ms: int = 0):
        us0, us1 = self._cfg_to_us(cfg)
        self.hw.send_smooth(us0, us1, smooth_ms if smooth_ms > 0 else 0)

    def update_title(self, cfg: Config, msg: str = ""):
        th1d = (math.degrees(cfg.theta1_std) % 360.0)
        th2d = (math.degrees(cfg.theta2_std) % 360.0)
        us0, us1 = self._cfg_to_us(cfg)
        base = f"theta1_std={th1d:.1f}°, theta2_std={th2d:.1f}° | CH0={us0}us CH1={us1}us"
        if msg:
            base += " | " + msg
        self.ax.set_title(base)

    def update_draw(self, cfg: Config):
        O1 = self.mech.O1
        O2 = self.mech.O2
        A, B, C, D = cfg.A, cfg.B, cfg.C, cfg.D

        self.line_O1A.set_data([O1[0], A[0]], [O1[1], A[1]])
        self.line_AB.set_data([A[0], B[0]],  [A[1], B[1]])
        self.line_O2C.set_data([O2[0], C[0]], [O2[1], C[1]])
        self.line_BD.set_data([B[0], D[0]], [B[1], D[1]])

        self.pt_O1.set_data([O1[0]], [O1[1]])
        self.pt_O2.set_data([O2[0]], [O2[1]])
        self.pt_A.set_data([A[0]], [A[1]])
        self.pt_B.set_data([B[0]], [B[1]])
        self.pt_C.set_data([C[0]], [C[1]])
        self.pt_D.set_data([D[0]], [D[1]])
        self.pt_target.set_data([self.target_D[0]], [self.target_D[1]])

        self.fig.canvas.draw_idle()

    def on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        self.target_D = (event.xdata, event.ydata)
        if not self.animating:
            self.animate_to_target(self.target_D)

    def animate_to_target(self, D_target: Tuple[float, float]):
        self.animating = True
        D0 = self.cfg.D
        steps = 40
        final_cfg = None

        for i in range(1, steps + 1):
            t = i / steps
            D = (D0[0] + t * (D_target[0] - D0[0]),
                 D0[1] + t * (D_target[1] - D0[1]))

            new_cfg = self.mech.solve_ik_for_D(D, prev=self.cfg)
            if new_cfg is None:
                self.update_title(self.cfg, msg="Unreachable (stopped).")
                break

            self.cfg = new_cfg
            final_cfg = new_cfg

            self.update_draw(self.cfg)
            self.update_title(self.cfg)
            plt.pause(0.02)

        # Send only final command to the real hardware (smooth)
        if final_cfg is not None:
            self._send_cfg_to_hw(final_cfg, smooth_ms=400)

        self.animating = False

    def run(self):
        plt.show()


if __name__ == "__main__":
    sim = Simulator()
    sim.run()
