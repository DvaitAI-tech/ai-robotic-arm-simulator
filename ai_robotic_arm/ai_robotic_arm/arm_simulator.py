import pygame
import math
from ai_robotic_arm.ai_controller import parse_command

# --- setup ---
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("AI Robotic Arm Simulator")

# --- colors ---
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE  = (50, 150, 255)

# --- arm parameters ---
base_x, base_y = WIDTH // 2, HEIGHT // 2 + 100
length1, length2 = 150, 100
angle1, angle2   = 45, 45
angle_speed      = 2
clock = pygame.time.Clock()

def draw_arm(screen, a1, a2):
    j1x = base_x + length1 * math.cos(math.radians(a1))
    j1y = base_y - length1 * math.sin(math.radians(a1))

    endx = j1x + length2 * math.cos(math.radians(a1 + a2))
    endy = j1y - length2 * math.sin(math.radians(a1 + a2))

    pygame.draw.line(screen, BLUE, (base_x, base_y), (j1x, j1y), 8)
    pygame.draw.line(screen, BLUE, (j1x, j1y), (endx, endy), 6)
    pygame.draw.circle(screen, WHITE, (int(endx), int(endy)), 8)
# --- new helper ---
def handle_ai_command(cmd):
    global angle1, angle2
    result = parse_command(cmd)
    print(result)
    if result is None:
        print("Unknown command")
        return
    target, val = result
    if target == "angle1": angle1 += val
    elif target == "angle2": angle2 += val
    elif target == "action": print(f"Performing {val} action")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        # if event.type == pygame.KEYDOWN:
        #     cmd = input("Enter command: ")
        #     handle_ai_command(cmd)

    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]: angle1 -= angle_speed
    if keys[pygame.K_d]: angle1 += angle_speed
    if keys[pygame.K_w]: angle2 -= angle_speed
    if keys[pygame.K_s]: angle2 += angle_speed

    screen.fill(BLACK)
    draw_arm(screen, angle1, angle2)
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
