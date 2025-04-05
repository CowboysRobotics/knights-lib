import pygame
import sys
import numpy as np

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 600, 600
BOX_SIZE = 400
SQUARE_SIZE = 50
BG_COLOR = (30, 30, 30)
BOX_COLOR = (200, 200, 200)
SQUARE_COLOR = (100, 200, 255)
SLIDER_COLOR = (255, 100, 100)
ARROW_COLOR = (255, 255, 0)
SENSOR_COLOR = (0, 255, 0)
TEXT_COLOR = (255, 255, 255)

box_x = (WIDTH - BOX_SIZE) // 2
box_y = (HEIGHT - BOX_SIZE) // 2

ORIGIN_X = box_x + BOX_SIZE // 2
ORIGIN_Y = box_y + BOX_SIZE // 2

# Coordinate system: The box represents 16 units (so each unit = BOX_SIZE / 16 pixels)
BOX_UNITS = 16
UNIT_SCALE = BOX_SIZE / BOX_UNITS  # pixels per unit

# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Square Follows Cursor with Rotation Slider")

# Initialize font
font = pygame.font.Font(None, 24)

# Initial square position and angle
square_pos = [ORIGIN_X, ORIGIN_Y]
angle = 0

square_coord = [0,0]

# Slider properties
slider_rect = pygame.Rect(100, HEIGHT - 50, 400, 10)
slider_knob = pygame.Rect(100, HEIGHT - 60, 20, 30)

# Distance sensors
front_sensor = [0, 4, 0]
left_sensor = [4, 0, 90]
right_sensor = [-4, 0, -90]
back_sensor = [0, -4, 180]

WALL_DIST = 72

def pt_to_pixel(pt):
  return [
    pt[0] * (BOX_SIZE / 144) + ORIGIN_X,
    -(pt[1] * (BOX_SIZE / 144) - ORIGIN_Y)
  ]
 

def ray_cast(sensor):
  sensor_coord = [
    square_coord[0] + sensor[0] * -np.sin(np.radians(angle)) + sensor[1] * np.cos(np.radians(angle)),
    square_coord[1] + sensor[0] * np.cos(np.radians(angle)) + sensor[1] * np.sin(np.radians(angle))
  ]
 
  dist = 0
  dist_step = 0.2
 
  sensor_angle = np.remainder(angle + sensor[2], 360)
 
  ray_x = sensor_coord[0]
  ray_y = sensor_coord[1]
 
  while (not (np.abs(ray_x) > WALL_DIST or np.abs(ray_y) > WALL_DIST)):
    dist += dist_step
   
    ray_x += dist_step * np.cos(np.radians(sensor_angle))
    ray_y += dist_step * np.sin(np.radians(sensor_angle))
 
  ray_end = [
   ray_x, ray_y
  ]
 
  pygame.draw.line(screen, SENSOR_COLOR, pt_to_pixel(sensor_coord), pt_to_pixel(ray_end), 2)
 
  return dist
 
 

def draw_box():
    pygame.draw.rect(screen, BOX_COLOR, (box_x, box_y, BOX_SIZE, BOX_SIZE), 2)

def draw_square():
    global square_coord
    rotated_square = pygame.Surface((SQUARE_SIZE, SQUARE_SIZE), pygame.SRCALPHA)
    rotated_square.fill((0, 0, 0, 0))
    pygame.draw.rect(rotated_square, SQUARE_COLOR, (0, 0, SQUARE_SIZE, SQUARE_SIZE))
    rotated_square = pygame.transform.rotate(rotated_square, angle)
    rect = rotated_square.get_rect(center=square_pos)
    screen.blit(rotated_square, rect.topleft)
   
    # Draw arrow indicating direction
    arrow_length = 30
    end_x = square_pos[0] + arrow_length * np.cos(np.radians(-angle))
    end_y = square_pos[1] + arrow_length * np.sin(np.radians(-angle))
    pygame.draw.line(screen, ARROW_COLOR, square_pos, (end_x, end_y), 3)

    # update coordinates
    square_coord = [
        (square_pos[0] - ORIGIN_X) * (144 / BOX_SIZE),
        (-square_pos[1] + ORIGIN_Y) * (144 / BOX_SIZE)
    ]

def draw_slider():
    pygame.draw.rect(screen, SLIDER_COLOR, slider_rect)
    pygame.draw.rect(screen, (255, 255, 255), slider_knob)

def draw_position_label():
    position_text = font.render(f"Position: {square_coord[0]:.2f}, {square_coord[1]:.2f} | Angle: {angle}", True, TEXT_COLOR)
    screen.blit(position_text, (10, 10))
 
    right_dist = ray_cast(right_sensor) + right_sensor[0] * np.cos(np.radians(right_sensor[2] - angle))
    front_dist = ray_cast(front_sensor) - front_sensor[1] * np.cos(np.radians(front_sensor[2] - angle))
    left_dist = ray_cast(left_sensor) - left_sensor[0] * np.cos(np.radians(left_sensor[2] - angle))
    back_dist = ray_cast(back_sensor) + back_sensor[1] * np.cos(np.radians(back_sensor[2] - angle))
   
    estimated_pos = [
      (right_dist - left_dist),
      (front_dist - back_dist)
    ]
   
    position_text = font.render(f"Estimation: {estimated_pos[0]:.2f}, {estimated_pos[1]:.2f}", True, TEXT_COLOR)
    screen.blit(position_text, (410, 10))
   
   

def get_slider_value():
    return int((slider_knob.x - slider_rect.x) / slider_rect.width * 360)

# Main loop
running = True
holding_slider = False
while running:
    screen.fill(BG_COLOR)
   
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                if slider_knob.collidepoint(event.pos):
                    holding_slider = True
                else:
                    if box_x < event.pos[0] < box_x + BOX_SIZE and box_y < event.pos[1] < box_y + BOX_SIZE:
                        square_pos = list(event.pos)
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                holding_slider = False
        elif event.type == pygame.MOUSEMOTION and holding_slider:
            slider_knob.x = max(slider_rect.x, min(event.pos[0] - slider_knob.width // 2, slider_rect.x + slider_rect.width - slider_knob.width))
            angle = get_slider_value()
   
    draw_box()
    draw_square()
    draw_slider()
    draw_position_label()
   
    pygame.display.flip()
    pygame.time.delay(10)

pygame.quit()
sys.exit()