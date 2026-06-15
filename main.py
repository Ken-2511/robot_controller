from gui import Arm2D_Painter, PendulumPainter
from plant import Arm2D, Pendulum
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
    joint_step_rad = math.radians(2.0)
    mouse_drag_threshold_px = 6

    pygame.init()
    pygame.display.set_caption("Arm2D Viewer")

    painter = Arm2D_Painter()
    screen = pygame.display.set_mode(
        (painter.canvas_width_px, painter.canvas_height_px)
    )
    render_surface = pygame.Surface(
        (painter.render_width_px, painter.render_height_px)
    )
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 12 * painter.render_scale)

    arm = Arm2D(
        mass1=1.2,
        mass2=0.9,
        mass3=0.5,
        inertia1=0.02,
        inertia2=0.015,
        inertia3=0.005,
        com1=0.225,
        com2=0.175,
        com3=0.1,
        length1=0.45,
        length2=0.35,
        length3=0.20,
        gravity=9.81,
        q1=math.radians(35.0),
        q2=math.radians(45.0),
        q3=math.radians(-50.0),
        q1d=0.0,
        q2d=0.0,
        q3d=0.0,
    )

    running = True
    mouse_start_screen = None
    mouse_start_world = None

    def run_ik_once() -> None:
        target_pose = painter.get_target_pose()
        if target_pose is None:
            return

        _q1, _q2, _q3, is_reachable = arm.IK(*target_pose)
        if is_reachable:
            arm.q1, arm.q2, arm.q3 = _q1, _q2, _q3
        else:
            print("Target is unreachable.")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    arm.q1 += joint_step_rad
                elif event.key == pygame.K_a:
                    arm.q1 -= joint_step_rad
                elif event.key == pygame.K_w:
                    arm.q2 += joint_step_rad
                elif event.key == pygame.K_s:
                    arm.q2 -= joint_step_rad
                elif event.key == pygame.K_e:
                    arm.q3 += joint_step_rad
                elif event.key == pygame.K_d:
                    arm.q3 -= joint_step_rad
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mouse_start_screen = event.pos
                mouse_start_world = painter.screen_to_world(event.pos)
            elif event.type == pygame.MOUSEMOTION and mouse_start_screen is not None:
                dx_px = event.pos[0] - mouse_start_screen[0]
                dy_px = event.pos[1] - mouse_start_screen[1]

                if math.hypot(dx_px, dy_px) >= mouse_drag_threshold_px:
                    current_world = painter.screen_to_world(event.pos)
                    if mouse_start_world is None:
                        continue

                    dx_m = current_world[0] - mouse_start_world[0]
                    dy_m = current_world[1] - mouse_start_world[1]
                    theta_rad = math.atan2(dy_m, dx_m)
                    painter.set_target_pose(
                        mouse_start_world[0],
                        mouse_start_world[1],
                        theta_rad,
                    )
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if mouse_start_screen is not None:
                    dx_px = event.pos[0] - mouse_start_screen[0]
                    dy_px = event.pos[1] - mouse_start_screen[1]
                    dragged_far_enough = (
                        math.hypot(dx_px, dy_px) >= mouse_drag_threshold_px
                    )

                    if dragged_far_enough and mouse_start_world is not None:
                        release_world = painter.screen_to_world(event.pos)
                        dx_m = release_world[0] - mouse_start_world[0]
                        dy_m = release_world[1] - mouse_start_world[1]
                        theta_rad = math.atan2(dy_m, dx_m)
                        painter.set_target_pose(
                            mouse_start_world[0],
                            mouse_start_world[1],
                            theta_rad,
                        )
                    else:
                        target_x, target_y = painter.screen_to_world(event.pos)
                        painter.set_target_position(target_x, target_y)

                    run_ik_once()

                mouse_start_screen = None
                mouse_start_world = None

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
