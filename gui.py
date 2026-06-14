import math
import pygame

from plant import Arm2D


class PendulumPainter:
    """Draw-only pendulum view.

    Angle convention:
    - 0 radians points vertically downward.
    - Positive radians rotate counterclockwise around +z, out of the page.
    """

    def __init__(
        self,
        center: tuple[int, int],
        swing_radius_m: float,
        px_per_meter: float,
    ) -> None:
        self.center = pygame.Vector2(center)
        self.length_px = swing_radius_m * px_per_meter
        self.angle_rad = 0.0
        self.reference_angle_rad = 0.0
        self.instruction_text = ""

        self.background_color = (248, 249, 251)
        self.guide_color = (214, 220, 228)
        self.reference_color = (65, 125, 230)
        self.rod_color = (35, 43, 54)
        self.bob_color = (231, 91, 77)
        self.pivot_color = (28, 36, 48)
        self.text_color = (61, 69, 82)

    def set_angle(self, angle_rad: float) -> None:
        self.angle_rad = angle_rad % (2 * math.pi)

    def set_reference_angle(self, angle_rad: float) -> None:
        self.reference_angle_rad = angle_rad % (2 * math.pi)

    def position_from_angle(self, angle_rad: float) -> pygame.Vector2:
        # Pygame's screen y-axis points downward, so this maps from math xy
        # coordinates with +z out of the page into screen coordinates.
        offset = pygame.Vector2(
            math.sin(angle_rad) * self.length_px,
            math.cos(angle_rad) * self.length_px,
        )
        return self.center + offset

    def angle_from_position(self, position: tuple[int, int]) -> float:
        offset = pygame.Vector2(position) - self.center
        return math.atan2(offset.x, offset.y) % (2 * math.pi)
    
    def set_instruction_text(self, text: str) -> None:
        self.instruction_text = text

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        surface.fill(self.background_color)

        center_xy = (round(self.center.x), round(self.center.y))
        length_px = round(self.length_px)

        pygame.draw.circle(
            surface,
            self.guide_color,
            center_xy,
            length_px,
            width=1,
        )
        pygame.draw.line(
            surface,
            (230, 234, 240),
            (center_xy[0], center_xy[1] - length_px),
            (center_xy[0], center_xy[1] + length_px),
            width=1,
        )
        pygame.draw.line(
            surface,
            (230, 234, 240),
            (center_xy[0] - length_px, center_xy[1]),
            (center_xy[0] + length_px, center_xy[1]),
            width=1,
        )

        reference = self.position_from_angle(self.reference_angle_rad)
        reference_xy = (round(reference.x), round(reference.y))
        pygame.draw.line(surface, self.reference_color, center_xy, reference_xy, width=3)
        pygame.draw.circle(surface, self.reference_color, reference_xy, 12, width=3)

        bob = self.position_from_angle(self.angle_rad)
        bob_xy = (round(bob.x), round(bob.y))
        pygame.draw.line(surface, self.rod_color, center_xy, bob_xy, width=6)
        pygame.draw.circle(surface, self.bob_color, bob_xy, 18)
        pygame.draw.circle(surface, (142, 47, 40), bob_xy, 18, width=2)
        pygame.draw.circle(surface, self.pivot_color, center_xy, 10)

        angle_deg = math.degrees(self.angle_rad)
        label = font.render(
            f"angle: {self.angle_rad:5.2f} rad / {angle_deg:6.1f} deg",
            True,
            self.text_color,
        )
        surface.blit(label, (18, 18))

        reference_deg = math.degrees(self.reference_angle_rad)
        reference_label = font.render(
            f"ref:   {self.reference_angle_rad:5.2f} rad / {reference_deg:6.1f} deg",
            True,
            self.reference_color,
        )
        surface.blit(reference_label, (18, 38))

        controller_label = font.render(
            self.instruction_text,
            True,
            self.text_color,
        )
        surface.blit(controller_label, (18, 58))


class Arm2D_Painter:
    """Draw-only 2D arm view in world coordinates.

    World convention:
    - Frame 0 is at (0, 0).
    - +x points right, +y points up.
    - 0 radians points horizontally right.
    """

    def __init__(self) -> None:
        self.world_min = (-0.2, -0.1)
        self.world_max = (1.6, 1.0)
        self.canvas_height_px = 800
        self.render_scale = 2

        world_width_m = self.world_max[0] - self.world_min[0]
        world_height_m = self.world_max[1] - self.world_min[1]
        self.px_per_meter = self.canvas_height_px / world_height_m
        self.canvas_width_px = round(world_width_m * self.px_per_meter)
        self.render_width_px = self.canvas_width_px * self.render_scale
        self.render_height_px = self.canvas_height_px * self.render_scale

        self.background_color = (255, 255, 255)
        self.grid_color = (225, 229, 235)
        self.axis_color = (130, 140, 152)
        self.text_color = (80, 88, 100)
        self.link_color = (34, 47, 62)
        self.joint_color = (238, 92, 66)
        self.base_color = (38, 45, 55)
        self.ee_color = (45, 126, 224)
        self.frame_x_color = (210, 68, 68)
        self.frame_y_color = (54, 142, 82)
        self.target_pose = None

    def set_target_pose(self, x_m: float, y_m: float, theta_rad: float) -> None:
        self.target_pose = (x_m, y_m, theta_rad)

    def get_target_pose(self):
        return self.target_pose

    def set_target_position(self, x_m: float, y_m: float) -> None:
        theta_rad = 0.0
        if self.target_pose is not None:
            theta_rad = self.target_pose[2]
        self.set_target_pose(x_m, y_m, theta_rad)

    def world_to_screen(
        self,
        point_m: tuple[float, float],
        scale: float = 1.0,
    ) -> tuple[int, int]:
        x_m, y_m = point_m
        x_px = (x_m - self.world_min[0]) * self.px_per_meter * scale
        y_px = (self.world_max[1] - y_m) * self.px_per_meter * scale
        return round(x_px), round(y_px)

    def screen_to_world(self, point_px: tuple[int, int]) -> tuple[float, float]:
        x_px, y_px = point_px
        x_m = x_px / self.px_per_meter + self.world_min[0]
        y_m = self.world_max[1] - y_px / self.px_per_meter
        return x_m, y_m

    def _grid_values(self, start: float, stop: float, step: float) -> list[float]:
        first = math.ceil((start - 1e-9) / step)
        last = math.floor((stop + 1e-9) / step)
        return [round(i * step, 10) for i in range(first, last + 1)]

    def _draw_grid(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
        scale: float,
    ) -> None:
        grid_step_m = 0.1
        surface_rect = surface.get_rect()
        x_values = self._grid_values(self.world_min[0], self.world_max[0], grid_step_m)
        y_values = self._grid_values(self.world_min[1], self.world_max[1], grid_step_m)

        for x_m in x_values:
            start = self.world_to_screen((x_m, self.world_min[1]), scale)
            end = self.world_to_screen((x_m, self.world_max[1]), scale)
            color = self.axis_color if math.isclose(x_m, 0.0, abs_tol=1e-9) else self.grid_color
            width = round(2 * scale) if math.isclose(x_m, 0.0, abs_tol=1e-9) else round(scale)
            pygame.draw.line(surface, color, start, end, width=width)

            label = font.render(f"{x_m:.1f}", True, self.text_color)
            label_rect = label.get_rect()
            label_rect.centerx = start[0]
            label_rect.top = round((self.canvas_height_px - 20) * scale)
            label_rect.clamp_ip(surface_rect)
            surface.blit(label, label_rect)

        for y_m in y_values:
            start = self.world_to_screen((self.world_min[0], y_m), scale)
            end = self.world_to_screen((self.world_max[0], y_m), scale)
            color = self.axis_color if math.isclose(y_m, 0.0, abs_tol=1e-9) else self.grid_color
            width = round(2 * scale) if math.isclose(y_m, 0.0, abs_tol=1e-9) else round(scale)
            pygame.draw.line(surface, color, start, end, width=width)

            label = font.render(f"{y_m:.1f}", True, self.text_color)
            label_rect = label.get_rect()
            label_rect.left = round(6 * scale)
            label_rect.centery = start[1]
            label_rect.clamp_ip(surface_rect)
            surface.blit(label, label_rect)

    def _draw_frame0(
        self,
        surface: pygame.Surface,
        font: pygame.font.Font,
        scale: float,
    ) -> None:
        origin = self.world_to_screen((0.0, 0.0), scale)
        x_axis = self.world_to_screen((0.12, 0.0), scale)
        y_axis = self.world_to_screen((0.0, 0.12), scale)

        pygame.draw.line(surface, self.frame_x_color, origin, x_axis, width=round(3 * scale))
        pygame.draw.line(surface, self.frame_y_color, origin, y_axis, width=round(3 * scale))
        pygame.draw.circle(surface, self.base_color, origin, round(6 * scale))

        x_label = font.render("x0", True, self.frame_x_color)
        y_label = font.render("y0", True, self.frame_y_color)
        surface.blit(x_label, (x_axis[0] + round(4 * scale), x_axis[1] - round(8 * scale)))
        surface.blit(y_label, (y_axis[0] + round(4 * scale), y_axis[1] - round(16 * scale)))

    def _draw_end_effector_frame(
        self,
        surface: pygame.Surface,
        end_effector_m: tuple[float, float],
        theta_rad: float,
        scale: float,
    ) -> None:
        self._draw_pose_frame(surface, end_effector_m, theta_rad, scale, 0.08)

    def _draw_pose_frame(
        self,
        surface: pygame.Surface,
        origin_m: tuple[float, float],
        theta_rad: float,
        scale: float,
        axis_length_m: float,
    ) -> None:
        x_end = (
            origin_m[0] + axis_length_m * math.cos(theta_rad),
            origin_m[1] + axis_length_m * math.sin(theta_rad),
        )
        y_end = (
            origin_m[0] - axis_length_m * math.sin(theta_rad),
            origin_m[1] + axis_length_m * math.cos(theta_rad),
        )

        origin = self.world_to_screen(origin_m, scale)
        x_axis = self.world_to_screen(x_end, scale)
        y_axis = self.world_to_screen(y_end, scale)
        pygame.draw.line(surface, self.frame_x_color, origin, x_axis, width=round(3 * scale))
        pygame.draw.line(surface, self.frame_y_color, origin, y_axis, width=round(3 * scale))

    def _draw_target_pose(self, surface: pygame.Surface, font: pygame.font.Font, scale: float) -> None:
        if self.target_pose is None:
            return

        x_m, y_m, theta_rad = self.target_pose
        origin_m = (x_m, y_m)
        origin_px = self.world_to_screen(origin_m, scale)

        pygame.draw.circle(
            surface,
            self.frame_x_color,
            origin_px,
            round(5 * scale),
            width=round(2 * scale),
        )
        self._draw_pose_frame(surface, origin_m, theta_rad, scale, 0.12)

        label = font.render(
            f"target: x={x_m:.3f} m, y={y_m:.3f} m, theta={math.degrees(theta_rad):.1f} deg",
            True,
            self.frame_x_color,
        )
        surface.blit(label, (round(12 * scale), round(30 * scale)))

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, arm: Arm2D) -> None:
        scale = surface.get_height() / self.canvas_height_px

        surface.fill(self.background_color)
        self._draw_grid(surface, font, scale)
        self._draw_frame0(surface, font, scale)

        joint_positions_m = [(0.0, 0.0), *arm.FK()]
        joint_positions_px = [
            self.world_to_screen(point, scale) for point in joint_positions_m
        ]

        if len(joint_positions_px) >= 2:
            pygame.draw.lines(
                surface,
                self.link_color,
                False,
                joint_positions_px,
                width=round(7 * scale),
            )

        for index, point_px in enumerate(joint_positions_px):
            radius = round((9 if index == 0 else 8) * scale)
            color = self.base_color if index == 0 else self.joint_color
            pygame.draw.circle(surface, color, point_px, radius)
            pygame.draw.circle(
                surface,
                (255, 255, 255),
                point_px,
                radius,
                width=round(2 * scale),
            )

        end_effector_m = joint_positions_m[-1]
        end_effector_px = joint_positions_px[-1]
        theta = arm.q1 + arm.q2 + arm.q3
        pygame.draw.circle(
            surface,
            self.ee_color,
            end_effector_px,
            round(11 * scale),
            width=round(3 * scale),
        )
        self._draw_end_effector_frame(surface, end_effector_m, theta, scale)
        self._draw_target_pose(surface, font, scale)

        status = font.render(
            f"end: x={end_effector_m[0]:.3f} m, y={end_effector_m[1]:.3f} m, theta={math.degrees(theta):.1f} deg",
            True,
            self.text_color,
        )
        surface.blit(status, (round(12 * scale), round(12 * scale)))
