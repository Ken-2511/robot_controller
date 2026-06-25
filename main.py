from gui import Arm2D_Painter, PendulumPainter
from plant import Arm2D, Pendulum
from control import Arm2D_PD_Controller, Pendulum_PD_Controller, Pendulum_CT_Controller
import ctypes
import pygame
import math
import sys


WINDOW_HEIGHT_PX = 1000


def enable_high_dpi() -> None:
    if sys.platform != "win32":
        return

    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass


def main_pd(reference_angle_rad: float) -> None:
    """
    PD Controller with a constant reference angle.
    """
    canvas_size = WINDOW_HEIGHT_PX
    pendulum_length_m = 1.0
    px_per_meter = 300
    render_hz = 60
    sim_hz = 3000
    steps_per_frame = sim_hz // render_hz

    enable_high_dpi()
    pygame.init()
    pygame.display.set_caption("Pendulum Simulation")
    screen = pygame.display.set_mode((canvas_size, canvas_size))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    pen = Pendulum(
        mass=1.0,
        length=pendulum_length_m,
        gravity=9.81,
        q=math.pi / 3,
        qd=0.0,
    )
    controller = Pendulum_PD_Controller(
        kp=25.0,
        kd=10.0,
        mass=1.0,
        gravity=9.81,
        length=1.0,
    )
    painter = PendulumPainter(
        center=(canvas_size // 2, canvas_size // 2),
        swing_radius_m=pendulum_length_m,
        px_per_meter=px_per_meter,
    )
    painter.set_instruction_text(f"SPACE toggles controller | click sets reference")

    running = True
    controller_enabled = True
    painter.set_reference_angle(reference_angle_rad)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                controller_enabled = not controller_enabled
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                reference_angle_rad = painter.angle_from_position(event.pos)
                painter.set_reference_angle(reference_angle_rad)

        for _ in range(steps_per_frame):
            if controller_enabled:
                tau = controller.compute_torque(pen.q, pen.qd, reference_angle_rad)
            else:
                tau = 0.0
            qdd = pen.equ_of_motion(tau=tau)
            pen.qd += qdd / sim_hz
            pen.q += pen.qd / sim_hz

        painter.set_angle(pen.q)

        painter.draw(screen, font)
        pygame.display.flip()

        clock.tick(render_hz)
    
    pygame.quit()


def main_computed_torque() -> None:
    canvas_size = WINDOW_HEIGHT_PX
    pendulum_length_m = 1.0
    px_per_meter = 300
    render_hz = 60
    sim_hz = 3000
    steps_per_frame = sim_hz // render_hz
    
    enable_high_dpi()
    pygame.init()
    pygame.display.set_caption("Pendulum Simulation")
    screen = pygame.display.set_mode((canvas_size, canvas_size))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)

    pen = Pendulum(
        mass=1.0,
        length=pendulum_length_m,
        gravity=9.81,
        q=math.pi / 3,
        qd=0.0,
    )
    controller = Pendulum_CT_Controller(
        kp=25.0,
        kd=10.0,
        mass=1.0,
        gravity=9.81,
        length=1.0,
    )
    painter = PendulumPainter(
        center=(canvas_size // 2, canvas_size // 2),
        swing_radius_m=pendulum_length_m,
        px_per_meter=px_per_meter,
    )

    running = True
    controller_enabled = True
    frame = 0
    qr = 0.0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                controller_enabled = not controller_enabled

        for step in range(steps_per_frame):

            # find the reference signal
            # want: qr(t) = pi/2 + pi/6 * sin(wt)
            # qrd(t) = pi/6 * w * cos(wt)
            # qrdd(t) = - pi/6 * w^2 * sin(t)
            t = (frame * steps_per_frame + step) / sim_hz
            w = math.pi
            qr = math.pi / 2 + math.pi / 6 * math.sin(w * t)
            qrd = math.pi / 6 * w * math.cos(w * t)
            qrdd = - math.pi / 6 * w * w * math.sin(w * t)

            if controller_enabled:
                tau = controller.compute_torque(pen.q, pen.qd, qr, qrd, qrdd)
            else:
                tau = 0.0

            qdd = pen.equ_of_motion(tau=tau)
            pen.qd += qdd / sim_hz
            pen.q += pen.qd / sim_hz

        painter.set_reference_angle(qr)
        painter.set_angle(pen.q)

        painter.draw(screen, font)
        pygame.display.flip()

        clock.tick(render_hz)
        frame += 1
    
    pygame.quit()


def main_arm2d() -> None:
    render_hz = 60
    sim_hz = 3000
    steps_per_frame = sim_hz // render_hz
    target_pose = (0.55, 0.25, math.radians(0.0))

    enable_high_dpi()
    pygame.init()
    pygame.display.set_caption("Arm2D Joint PD")

    painter = Arm2D_Painter(canvas_height_px=WINDOW_HEIGHT_PX)
    screen = pygame.display.set_mode(
        (painter.canvas_width_px, painter.canvas_height_px)
    )
    render_surface = pygame.Surface(
        (painter.render_width_px, painter.render_height_px)
    )
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 12 * painter.render_scale)

    arm = Arm2D(
        m1=1.2,
        m2=0.9,
        m3=0.5,
        i1=0.02,
        i2=0.015,
        i3=0.005,
        lc1=0.225,
        lc2=0.175,
        lc3=0.1,
        l1=0.45,
        l2=0.35,
        l3=0.20,
        gravity=9.81,
        q1=math.radians(-90.0),
        q2=math.radians(0.0),
        q3=math.radians(0.0),
        q1d=0.0,
        q2d=0.0,
        q3d=0.0,
    )
    q1r, q2r, q3r, is_reachable = arm.IK(*target_pose)
    if not is_reachable:
        raise RuntimeError("Target pose is unreachable.")

    painter.set_target_pose(*target_pose)
    controller = Arm2D_PD_Controller(
        kp1=5.0,
        kp2=4.0,
        kp3=3.0,
        kd1=3.0,
        kd2=2.4,
        kd3=1.8,
    )

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for _ in range(steps_per_frame):
            tau1, tau2, tau3 = controller.compute_torque(
                arm.q1, arm.q2, arm.q3,
                arm.q1d, arm.q2d, arm.q3d,
                q1r, q2r, q3r,
            )
            g1, g2, g3 = arm.gravity_vector()
            tau1 += g1
            tau2 += g2
            tau3 += g3
            q1dd, q2dd, q3dd = arm.equ_of_motion(tau1, tau2, tau3)
            arm.q1d += q1dd / sim_hz
            arm.q2d += q2dd / sim_hz
            arm.q3d += q3dd / sim_hz
            arm.q1 += arm.q1d / sim_hz
            arm.q2 += arm.q2d / sim_hz
            arm.q3 += arm.q3d / sim_hz

        painter.draw(render_surface, font, arm)
        pygame.transform.smoothscale(
            render_surface,
            (painter.canvas_width_px, painter.canvas_height_px),
            screen,
        )
        pygame.display.flip()
        clock.tick(render_hz)

    pygame.quit()


if __name__ == "__main__":
    main_arm2d()
    # main_pd(reference_angle_rad=math.pi)
    # main_computed_torque()
