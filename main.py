from gui import PendulumPainter
from plant import Pendulum
from control import Pendulum_PD_Controller, Pendulum_CT_Controller
import pygame
import math


def main_pd(reference_angle_rad: float) -> None:
    """
    PD Controller with a constant reference angle.
    """
    canvas_size = 800
    pendulum_length_m = 1.0
    px_per_meter = 300
    render_hz = 60
    sim_hz = 3000
    steps_per_frame = sim_hz // render_hz

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

    running = True
    controller_enabled = True
    painter.set_reference_angle(reference_angle_rad)
    painter.set_controller_enabled(controller_enabled)
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                controller_enabled = not controller_enabled
                painter.set_controller_enabled(controller_enabled)

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
    canvas_size = 800
    pendulum_length_m = 1.0
    px_per_meter = 300
    render_hz = 60
    sim_hz = 3000
    steps_per_frame = sim_hz // render_hz
    
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
    painter.set_controller_enabled(controller_enabled)
    frame = 0
    qr = 0.0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                controller_enabled = not controller_enabled
                painter.set_controller_enabled(controller_enabled)

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


if __name__ == "__main__":
    main_pd(reference_angle_rad=math.pi)