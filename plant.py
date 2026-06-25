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
    i1: float       # kg*m^2, about the joint axis
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
    
    def equ_of_motion(self, tau1: float, tau2: float, tau3: float) -> tuple[float, float, float]:  # q1dd, q2dd, q3dd in rad/s^2
        w1 = self.q1d
        w2 = self.q1d + self.q2d
        w3 = self.q1d + self.q2d + self.q3d
        v_com1_sq = (w1 * self.lc1) ** 2
        v_com2_sq = (w1 * self.l1) ** 2 + (w2 * self.lc2) ** 2\
            + 2 * self.l1 * self.lc2 * w1 * w2 * math.cos(self.q2)
        v_com3_sq = (w1 * self.l1) ** 2 + (w2 * self.l2) ** 2 + (w3 * self.lc3) ** 2\
            + 2 * self.l1 * self.l2 * w1 * w2 * math.cos(self.q2)\
            + 2 * self.l1 * self.lc3 * w1 * w3 * math.cos(self.q2 + self.q3)\
            + 2 * self.l2 * self.lc3 * w2 * w3 * math.cos(self.q3)
        kinetic_energy = 0.5 * (self.m1 * v_com1_sq + self.m2 * v_com2_sq + self.m3 * v_com3_sq) + \
            0.5 * (self.i1 * w1 ** 2 + self.i2 * w2 ** 2 + self.i3 * w3 ** 2)
        potential_energy = self.m1 * self.gravity * self.lc1 * math.sin(self.q1) + \
            self.m2 * self.gravity * (self.l1 * math.sin(self.q1) + self.lc2 * math.sin(self.q1 + self.q2)) + \
            self.m3 * self.gravity * (self.l1 * math.sin(self.q1) + self.l2 * math.sin(self.q1 + self.q2) + self.lc3 * math.sin(self.q1 + self.q2 + self.q3))
        