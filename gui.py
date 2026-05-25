import math
import pygame


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
