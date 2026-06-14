import math
from dataclasses import dataclass

@dataclass
class Pendulum:
    # static
    mass: float     # kg
    length: float   # m
    gravity: float  # m/s^2
    # dynamic
    q: float        # rad
    qd: float       # rad/s

    def equ_of_motion(self, tau: float) -> float:
        # tau is in N*m; return qdd in rad/s^2.
        return - (self.gravity / self.length) * math.sin(self.q) + tau / (self.mass * self.length * self.length)
    
@dataclass
class Arm2D:
    # static
    mass1: float    # kg
    mass2: float
    mass3: float
    length1: float  # m
    length2: float
    length3: float
    gravity: float  # m/s^2
    # dynamic
    q1: float       # rad
    q2: float
    q3: float
    q1d: float      # rad/s
    q2d: float
    q3d: float

    def FK(self) -> tuple[tuple[float, float],   # Ox1, Oy1
                          tuple[float, float],   # Ox2, Oy2
                          tuple[float, float]]:  # Ox3, Oy3
        # Frame 0 is at (0, 0), with 0 rad pointing horizontally right.
        angle1 = self.q1
        angle2 = self.q1 + self.q2
        angle3 = self.q1 + self.q2 + self.q3

        Ox1 = self.length1 * math.cos(angle1)
        Oy1 = self.length1 * math.sin(angle1)
        Ox2 = Ox1 + self.length2 * math.cos(angle2)
        Oy2 = Oy1 + self.length2 * math.sin(angle2)
        Ox3 = Ox2 + self.length3 * math.cos(angle3)
        Oy3 = Oy2 + self.length3 * math.sin(angle3)
        return (Ox1, Oy1), (Ox2, Oy2), (Ox3, Oy3)

    def IK(self, x_ee, y_ee, theta_ee) -> tuple[float, float, float, bool]:  # q1, q2, q3 in radians, is_reachable 
        # This is a simple geometric IK for 3-DOF planar arm. It does not handle singularities or unreachable targets.
        x_wrist = x_ee - self.length3 * math.cos(theta_ee)
        y_wrist = y_ee - self.length3 * math.sin(theta_ee)
        if not (abs(self.length1 - self.length2) + 0.01 <= math.sqrt(x_wrist**2 + y_wrist**2) <= self.length1 + self.length2 - 0.01):
            return 0.0, 0.0, 0.0, False

        cos_q2 = (x_wrist**2 + y_wrist**2 - self.length1**2 - self.length2**2) / (2 * self.length1 * self.length2)
        q2 = math.atan2(-math.sqrt(1 - cos_q2**2), cos_q2)  # Elbow-up solution
        q1 = math.atan2(y_wrist, x_wrist) + math.atan2(-self.length2 * math.sin(q2), self.length1 + self.length2 * math.cos(q2))
        q3 = theta_ee - q1 - q2
        return q1, q2, q3, True