import math
from math import sin, cos
import numpy as np
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
    m1: float    # kg
    m2: float
    m3: float
    i1: float       # kg*m^2, about the link center of mass
    i2: float
    i3: float
    lc1: float     # m, assume to be along the link
    lc2: float
    lc3: float
    l1: float  # m
    l2: float
    l3: float
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

        Ox1 = self.l1 * math.cos(angle1)
        Oy1 = self.l1 * math.sin(angle1)
        Ox2 = Ox1 + self.l2 * math.cos(angle2)
        Oy2 = Oy1 + self.l2 * math.sin(angle2)
        Ox3 = Ox2 + self.l3 * math.cos(angle3)
        Oy3 = Oy2 + self.l3 * math.sin(angle3)
        return (Ox1, Oy1), (Ox2, Oy2), (Ox3, Oy3)

    def IK(self, x_ee, y_ee, theta_ee) -> tuple[float, float, float, bool]:  # q1, q2, q3 in radians, is_reachable 
        # This is a simple geometric IK for 3-DOF planar arm. It does not handle singularities or unreachable targets.
        x_wrist = x_ee - self.l3 * math.cos(theta_ee)
        y_wrist = y_ee - self.l3 * math.sin(theta_ee)
        if not (abs(self.l1 - self.l2) + 0.01 <= math.sqrt(x_wrist**2 + y_wrist**2) <= self.l1 + self.l2 - 0.01):
            return 0.0, 0.0, 0.0, False

        cos_q2 = (x_wrist**2 + y_wrist**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        q2 = math.atan2(-math.sqrt(1 - cos_q2**2), cos_q2)  # Elbow-up solution
        q1 = math.atan2(y_wrist, x_wrist) + math.atan2(-self.l2 * math.sin(q2), self.l1 + self.l2 * math.cos(q2))
        q3 = theta_ee - q1 - q2
        return q1, q2, q3, True
    
    def gravity_vector(self) -> tuple[float, float, float]:
        m1, m2, m3 = self.m1, self.m2, self.m3
        l1, l2 = self.l1, self.l2
        lc1, lc2, lc3 = self.lc1, self.lc2, self.lc3
        g = self.gravity

        c1 = math.cos(self.q1)
        c12 = math.cos(self.q1 + self.q2)
        c123 = math.cos(self.q1 + self.q2 + self.q3)
        return (
            g * ((m1 * lc1 + m2 * l1 + m3 * l1) * c1 + (m2 * lc2 + m3 * l2) * c12 + m3 * lc3 * c123),
            g * ((m2 * lc2 + m3 * l2) * c12 + m3 * lc3 * c123),
            g * m3 * lc3 * c123,
        )

    def equ_of_motion(self, tau1: float, tau2: float, tau3: float) -> tuple[float, float, float]:  # q1dd, q2dd, q3dd in rad/s^2
        m1, m2, m3 = self.m1, self.m2, self.m3
        i1, i2, i3 = self.i1, self.i2, self.i3
        l1, l2 = self.l1, self.l2
        lc1, lc2, lc3 = self.lc1, self.lc2, self.lc3

        qd = np.array([self.q1d, self.q2d, self.q3d])
        tau = np.array([tau1, tau2, tau3])

        c2 = math.cos(self.q2)
        c3 = math.cos(self.q3)
        c23 = math.cos(self.q2 + self.q3)
        s2 = math.sin(self.q2)
        s3 = math.sin(self.q3)
        s23 = math.sin(self.q2 + self.q3)

        m11 = i1 + i2 + i3 + m1 * lc1**2 + \
            m2 * (l1**2 + lc2**2 + 2 * l1 * lc2 * c2) + \
            m3 * (l1**2 + l2**2 + lc3**2 + 2 * l1 * l2 * c2 + 2 * l1 * lc3 * c23 + 2 * l2 * lc3 * c3)
        m12 = i2 + i3 + m2 * (lc2**2 + l1 * lc2 * c2) + \
            m3 * (l2**2 + lc3**2 + l1 * l2 * c2 + l1 * lc3 * c23 + 2 * l2 * lc3 * c3)
        m13 = i3 + m3 * (lc3**2 + l1 * lc3 * c23 + l2 * lc3 * c3)
        m22 = i2 + i3 + m2 * lc2**2 + m3 * (l2**2 + lc3**2 + 2 * l2 * lc3 * c3)
        m23 = i3 + m3 * (lc3**2 + l2 * lc3 * c3)
        m33 = i3 + m3 * lc3**2
        M = np.array([[m11, m12, m13],
                      [m12, m22, m23],
                      [m13, m23, m33]])

        G = np.array(self.gravity_vector())

        dM = np.zeros((3, 3, 3))
        dM[1] = np.array([
            [-2 * m2 * l1 * lc2 * s2 - 2 * m3 * l1 * (l2 * s2 + lc3 * s23), -m2 * l1 * lc2 * s2 - m3 * l1 * (l2 * s2 + lc3 * s23), -m3 * l1 * lc3 * s23],
            [-m2 * l1 * lc2 * s2 - m3 * l1 * (l2 * s2 + lc3 * s23), 0.0, 0.0],
            [-m3 * l1 * lc3 * s23, 0.0, 0.0],
        ])
        dM[2] = np.array([
            [-2 * m3 * lc3 * (l1 * s23 + l2 * s3), -m3 * lc3 * (l1 * s23 + 2 * l2 * s3), -m3 * lc3 * (l1 * s23 + l2 * s3)],
            [-m3 * lc3 * (l1 * s23 + 2 * l2 * s3), -2 * m3 * l2 * lc3 * s3, -m3 * l2 * lc3 * s3],
            [-m3 * lc3 * (l1 * s23 + l2 * s3), -m3 * l2 * lc3 * s3, 0.0],
        ])

        C = np.zeros(3)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    C[i] += 0.5 * (dM[k, i, j] + dM[j, i, k] - dM[i, j, k]) * qd[j] * qd[k]

        qdd = np.linalg.solve(M, tau - C - G)
        return float(qdd[0]), float(qdd[1]), float(qdd[2])
