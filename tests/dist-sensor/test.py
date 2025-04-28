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
BOX_COLOR = (150, 150, 150)
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
front_sensor = [5, 1, 0]
left_sensor = [-2, 2, 90]
right_sensor = [6, 5, -90]
back_sensor = [-3, -2, 180]

sensors = [front_sensor, left_sensor, right_sensor, back_sensor]

WALL_DIST = 72
SENSOR_SAFE_DIST = 78 # 2 meters ish

def pt_to_pixel(pt):
  return [
    pt[0] * (BOX_SIZE / 144) + ORIGIN_X,
    -(pt[1] * (BOX_SIZE / 144) - ORIGIN_Y)
  ]

def closest_cardinal(angle):
    cardinals = [0, 90, 180, 270]
    angle = angle % 360  # Normalize the angle
    closest = min(cardinals, key=lambda x: abs(x - angle))
    return closest

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

def ray_cast(sensor):
  sensor_coord = [
    square_coord[0] + -sensor[0] * -np.sin(np.radians(angle)) + sensor[1] * np.cos(np.radians(angle)),
    square_coord[1] + -sensor[0] * np.cos(np.radians(angle)) + sensor[1] * np.sin(np.radians(angle))
  ]
 
  dist = 0
  dist_step = 0.05
 
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

def draw_position_label():
    position_text = font.render(f"Position: {square_coord[0]:.2f}, {square_coord[1]:.2f} | Angle: {angle}", True, TEXT_COLOR)
    screen.blit(position_text, (10, 10))
   
    x_estimates = np.array([])
    y_estimates = np.array([])
   
    hit_positions = []

    TOLERANCE = 0.001
   
    for sensor in sensors:
        s_dist = ray_cast(sensor) # Get distance reading

        # If distance is effectively zero, sensor might be outside or faulty, skip it
        # Calculate sensor's absolute angle (degrees and radians)
        sensor_absolute_angle_deg = (angle + sensor[2]) % 360
        sensor_absolute_angle_rad = np.radians(sensor_absolute_angle_deg)

        cardinal_angle = closest_cardinal(sensor_absolute_angle_deg)
       
        s_theta = sensor_absolute_angle_rad
        bot_theta = np.radians(angle)
       
        hit_pos = [
          s_dist * np.cos(s_theta) + square_coord[0]
            + (-sensor[0] * -np.sin(bot_theta) + sensor[1] * np.cos(bot_theta)),
          s_dist * np.sin(s_theta) + square_coord[1]
            + (-sensor[0] * np.cos(bot_theta) + sensor[1] * np.sin(bot_theta))
          ]
         
        hit_positions.append(hit_pos)
       
        if s_dist > SENSOR_SAFE_DIST:
            continue

        if (hit_pos[0] > 0 and np.abs(hit_pos[0]) > np.abs(hit_pos[1])):
            # use for x estimate
            x_dist = np.cos(sensor_absolute_angle_rad) * s_dist + (-sensor[0] * -np.sin(bot_theta) + sensor[1] * np.cos(bot_theta))
            x_estimates = np.append(x_estimates, WALL_DIST - x_dist)
        elif (hit_pos[1] > 0 and np.abs(hit_pos[1]) > np.abs(hit_pos[0])):
            y_dist = np.sin(sensor_absolute_angle_rad) * s_dist + (-sensor[0] * np.cos(bot_theta) + sensor[1] * np.sin(bot_theta))
            y_estimates = np.append(y_estimates, WALL_DIST - y_dist)
        elif (hit_pos[0] < 0 and np.abs(hit_pos[0]) > np.abs(hit_pos[1])):
            # use for x estimate
            x_dist = np.cos(sensor_absolute_angle_rad) * s_dist + (-sensor[0] * -np.sin(bot_theta) + sensor[1] * np.cos(bot_theta))
            x_estimates = np.append(x_estimates,  -WALL_DIST - x_dist)
        elif (hit_pos[1] < 0 and np.abs(hit_pos[1]) > np.abs(hit_pos[0])):
            y_dist = np.sin(sensor_absolute_angle_rad) * s_dist + (-sensor[0] * np.cos(bot_theta) + sensor[1] * np.sin(bot_theta))
            y_estimates = np.append(y_estimates,  -WALL_DIST - y_dist)


    # Calculate average estimates, ignoring NaN if a list is empty
    # Use np.nanmean to handle cases where no sensors hit vertical or horizontal walls
    avg_x = np.nanmean(x_estimates) if x_estimates.any() else 0 # Default to actual if no estimate
    avg_y = np.nanmean(y_estimates) if y_estimates.any() else 0 # Default to actual if no estimate

    # Handle cases where nanmean returns NaN (if input list was empty)
    estimated_pos = [
        avg_x,
        avg_y
    ]
   
    position_text = font.render(f"Estimation: {estimated_pos[0]:.2f}, {estimated_pos[1]:.2f}", True, TEXT_COLOR)
    screen.blit(position_text, (410, 10))
   
    position_text = font.render(f"Estimate Length: {len(x_estimates)}, {len(y_estimates)}", True, TEXT_COLOR)
    screen.blit(position_text, (410, 30))
   
    position_text = font.render(f"F Hit: {hit_positions[0][0]:.1f}, {hit_positions[0][1]:.1f}", True, TEXT_COLOR)
    screen.blit(position_text, (200, 50))
   
    position_text = font.render(f"L Hit: {hit_positions[1][0]:.1f}, {hit_positions[1][1]:.1f}", True, TEXT_COLOR)
    screen.blit(position_text, (10, 50))
   
    position_text = font.render(f"R Hit: {hit_positions[2][0]:.1f}, {hit_positions[2][1]:.1f}", True, TEXT_COLOR)
    screen.blit(position_text, (10, 30))

    position_text = font.render(f"B Hit: {hit_positions[3][0]:.1f}, {hit_positions[3][1]:.1f}", True, TEXT_COLOR)
    screen.blit(position_text, (200, 30))
   

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