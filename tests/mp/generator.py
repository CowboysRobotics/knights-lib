import matplotlib.pyplot as plt
import numpy as np

figure, axis = plt.subplots(2, 1)

def dist_between(x1, y1, x2, y2):
  return np.hypot(x2-x1, y2-y1)

def lerp(start, end, step):
  return start + (end-start) * step

def get_total_time(max_acceleration, max_velocity, distance, elapsed_time):
  # Calculate the time it takes to accelerate to max velocity
  acceleration_dt = max_velocity / max_acceleration

  # If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
  halfway_distance = distance / 2
  acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

  if (acceleration_distance > halfway_distance):
    acceleration_dt = np.sqrt(halfway_distance / (0.5 * max_acceleration))

  acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

  # recalculate max velocity based on the time we have to accelerate and decelerate
  max_velocity = max_acceleration * acceleration_dt

  # we decelerate at the same rate as we accelerate
  deceleration_dt = acceleration_dt

  # calculate the time that we're at max velocity
  cruise_distance = distance - 2 * acceleration_distance
  cruise_dt = cruise_distance / max_velocity
  deceleration_time = acceleration_dt + cruise_dt

  # check if we're still in the motion profile
  return(acceleration_dt + cruise_dt + deceleration_dt)

def motion_profile_pos(max_acceleration, max_velocity, distance, elapsed_time):
  # Calculate the time it takes to accelerate to max velocity
  acceleration_dt = max_velocity / max_acceleration

  # If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
  halfway_distance = distance / 2
  acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

  if (acceleration_distance > halfway_distance):
    acceleration_dt = np.sqrt(halfway_distance / (0.5 * max_acceleration))

  acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2

  # recalculate max velocity based on the time we have to accelerate and decelerate
  max_velocity = max_acceleration * acceleration_dt

  # we decelerate at the same rate as we accelerate
  deceleration_dt = acceleration_dt

  # calculate the time that we're at max velocity
  cruise_distance = distance - 2 * acceleration_distance
  cruise_dt = cruise_distance / max_velocity
  deceleration_time = acceleration_dt + cruise_dt

  # check if we're still in the motion profile
  entire_dt = acceleration_dt + cruise_dt + deceleration_dt
  if (elapsed_time > entire_dt):
    return distance

  # if we're accelerating
  if (elapsed_time < acceleration_dt):
    # use the kinematic equation for acceleration
    return 0.5 * max_acceleration * elapsed_time ** 2

  # if we're cruising
  elif (elapsed_time < deceleration_time):
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
    cruise_current_dt = elapsed_time - acceleration_dt

    # use the kinematic equation for constant velocity
    return acceleration_distance + max_velocity * cruise_current_dt

  # if we're decelerating
  else:
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
    cruise_distance = max_velocity * cruise_dt
    deceleration_time = elapsed_time - deceleration_time

    # use the kinematic equations to calculate the instantaneous desired position
    return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2

curr = [0, 0, np.radians(90)]
target = [4, 4, np.radians(270)]

dist = np.hypot(target[0]-curr[0], target[1]-curr[1]) * 2

curr_tangent = (np.cos(curr[2]) * dist, np.sin(curr[2]) * dist)
target_tangent = (np.cos(target[2]) * dist, np.sin(target[2]) * dist)

t = np.linspace(0, 1, 200)

h1 = lambda t: 2*t**3 - 3*t**2 + 1
h2 = lambda t: -2*t**3 + 3*t**2
h3 = lambda t: t**3 - 2*t**2 + t
h4 = lambda t: t**3 - t**2

x_val = h1(t) * curr[0] + h2(t) * target[0] + h3(t) * curr_tangent[0] + h4(t) * target_tangent[0]
y_val = h1(t) * curr[1] + h2(t) * target[1] + h3(t) * curr_tangent[1] + h4(t) * target_tangent[1]

curr_pos = [curr[0], curr[1]]

total_dist = 0 
for i, val in enumerate(x_val):
    total_dist += dist_between(curr_pos[0], curr_pos[1], val, y_val[i])
    curr_pos = [val, y_val[i]]

DESIRED_VOLTAGE = 120
MAX_VOLTAGE = 127
WHEEL_DIAMETER = 2.75
RPM = 450

max_acceleration = 10 # arbitrary constant
max_velocity = (DESIRED_VOLTAGE/MAX_VOLTAGE) * np.pi * WHEEL_DIAMETER * (RPM / 60.0)

distances = []
times = np.linspace(0, get_total_time(max_acceleration, max_velocity, total_dist, val), 200)

for val in times:
  distances.append(motion_profile_pos(max_acceleration, max_velocity, total_dist, val))

axis[0].plot(times, distances)
axis[0].set_title("Distance Travelled")

axis[1].plot(x_val, y_val)
axis[1].set_title("Path")

plt.show()

