import pygame
import sys
import math
import random

# ------------------------------
# System parameters
A_BAR = 1.0          # maximum acceleration
DT = 0.02            # simulation step, seconds
THRESHOLD = 0.01     # stopping threshold (considered converged)

# Graphics
WIDTH, HEIGHT = 800, 600
PHASE_HEIGHT = 400   # height of the phase plane
POS_HEIGHT = HEIGHT - PHASE_HEIGHT  # height of the 1D strip
SCALE = 50           # pixels per unit
ORIGIN_PHASE = (WIDTH // 2, POS_HEIGHT + PHASE_HEIGHT // 2)

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (200, 50, 50)
BLUE = (50, 50, 200)
GREEN = (50, 180, 50)
GRAY = (180, 180, 180)
LIGHT_GRAY = (220, 220, 220)

class DoubleIntegrator:
    def __init__(self, p0, v0):
        self.p = p0
        self.v = v0
        self.t = 0.0
        self.trajectory = [(p0, v0)]   # for drawing the phase plane path
        self.control = 0.0
        self.finished = False

    def compute_control(self):
        """Optimal bang-bang control based on the sign of B."""
        p, v = self.p, self.v
        B = 2 * A_BAR * p + v * abs(v)
        if abs(B) < 1e-9:
            # brake immediately
            self.control = -math.copysign(A_BAR, v) if v != 0 else 0.0
        elif B > 0:
            self.control = -A_BAR
        else:
            self.control = A_BAR

    def step(self):
        if self.finished:
            return
        # Check if we have converged
        if abs(self.p) < THRESHOLD and abs(self.v) < THRESHOLD:
            self.p = 0.0
            self.v = 0.0
            self.control = 0.0
            self.finished = True
            self.trajectory.append((0.0, 0.0))
            return

        self.compute_control()
        # Euler integration
        self.v += self.control * DT
        self.p += self.v * DT
        self.t += DT
        self.trajectory.append((self.p, self.v))

def draw_1d(screen, p):
    """Draws the 1D position strip."""
    y_center = POS_HEIGHT // 2
    # track line
    pygame.draw.line(screen, GRAY, (50, y_center), (WIDTH - 50, y_center), 2)
    # origin marker
    zero_x = WIDTH // 2
    pygame.draw.line(screen, BLACK, (zero_x, y_center - 10), (zero_x, y_center + 10), 3)
    # position marker
    dot_x = int(zero_x + p * SCALE)
    dot_x = max(30, min(WIDTH - 30, dot_x))
    pygame.draw.circle(screen, RED, (dot_x, y_center), 10)

def draw_phase(screen, system):
    """Phase plane (p horizontal, v vertical)."""
    ox, oy = ORIGIN_PHASE
    # axes
    pygame.draw.line(screen, BLACK, (30, oy), (WIDTH - 30, oy), 2)   # p
    pygame.draw.line(screen, BLACK, (ox, POS_HEIGHT + 10), (ox, HEIGHT - 10), 2) # v
    # switching curve p = -v|v|/(2*A_BAR)
    prev_point = None
    for vy in range(-int(PHASE_HEIGHT/2), int(PHASE_HEIGHT/2), 2):
        v_val = vy / SCALE
        p_val = -v_val * abs(v_val) / (2 * A_BAR)
        px = ox + int(p_val * SCALE)
        py = oy - vy  # v points upward
        if 30 <= px <= WIDTH-30 and POS_HEIGHT+10 <= py <= HEIGHT-10:
            if prev_point:
                pygame.draw.line(screen, BLUE, prev_point, (px, py), 2)
            prev_point = (px, py)
        else:
            prev_point = None
    # trajectory
    if len(system.trajectory) > 1:
        pts = [(ox + int(p * SCALE), oy - int(v * SCALE)) for p, v in system.trajectory]
        pygame.draw.lines(screen, GREEN, False, pts, 2)
    # current state
    cur_x = ox + int(system.p * SCALE)
    cur_y = oy - int(system.v * SCALE)
    pygame.draw.circle(screen, RED, (cur_x, cur_y), 6)

def draw_text(screen, system):
    font = pygame.font.SysFont('Arial', 20)
    texts = [
        f't = {system.t:.2f} s',
        f'p = {system.p:.3f}',
        f'v = {system.v:.3f}',
        f'u = {system.control:.3f}',
        'Left click – new initial conditions, Right click – random, SPACE – start/pause'
    ]
    y = 5
    for line in texts:
        surf = font.render(line, True, BLACK)
        screen.blit(surf, (5, y))
        y += 22

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Double Integrator – Time-Optimal Transfer to Zero")
    clock = pygame.time.Clock()

    p0, v0 = 2.0, -3.0   # default initial conditions
    system = DoubleIntegrator(p0, v0)
    running = False

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    running = not running
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if my > POS_HEIGHT:   # click on phase plane
                    ox, oy = ORIGIN_PHASE
                    p_click = (mx - ox) / SCALE
                    v_click = -(my - oy) / SCALE
                    if event.button == 1:   # left click – set
                        p0, v0 = p_click, v_click
                    elif event.button == 3: # right click – random
                        p0 = random.uniform(-3, 3)
                        v0 = random.uniform(-3, 3)
                    system = DoubleIntegrator(p0, v0)
                    running = False   # reset animation

        if running:
            system.step()

        screen.fill(WHITE)
        draw_1d(screen, system.p)
        draw_phase(screen, system)
        draw_text(screen, system)
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()