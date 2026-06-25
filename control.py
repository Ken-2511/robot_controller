import math
from dataclasses import dataclass

@dataclass
class Pendulum_PD_Controller:
    kp: float
    kd: float
    mass: float
    gravity: float
    length: float

    def compute_torque(self, q: float, qd: float, qr: float) -> float:
        # Compute the control torque using a PD control law.
        error = math.atan2(math.sin(qr - q), math.cos(qr - q))  # shortest angle error
        return self.kp * error - self.kd * qd + self.mass * self.gravity * self.length * math.sin(q)
    
@dataclass
class Pendulum_CT_Controller:
    kp: float
    kd: float
    mass: float
    gravity: float
    length: float

    def compute_torque(self, q: float, qd: float, qr: float, qrd: float, qrdd: float) -> float:
        # Compute the control torque using a computed torque control law.
        a = qrdd + self.kp * (qr - q) + self.kd * (qrd - qd)
        return self.mass * self.length * self.length * a + self.mass * self.gravity * self.length * math.sin(q)

@dataclass
class Arm2D_PD_Controller:
    kp1: float
    kp2: float
    kp3: float
    kd1: float
    kd2: float
    kd3: float

    def compute_torque(self,
                       q1: float, q2: float, q3: float,
                       q1d: float, q2d: float, q3d: float,
                       q1r: float, q2r: float, q3r: float) -> tuple[float, float, float]:
        e1 = math.atan2(math.sin(q1r - q1), math.cos(q1r - q1))
        e2 = math.atan2(math.sin(q2r - q2), math.cos(q2r - q2))
        e3 = math.atan2(math.sin(q3r - q3), math.cos(q3r - q3))
        tau1 = self.kp1 * e1 - self.kd1 * q1d
        tau2 = self.kp2 * e2 - self.kd2 * q2d
        tau3 = self.kp3 * e3 - self.kd3 * q3d
        return tau1, tau2, tau3