import matplotlib.pyplot as plt
import numpy as np

figure, axis = plt.subplots(2, 2)

def dist_between(x1, y1, x2, y2):
  return np.hypot(x2-x1, y2-y1)

def lerp(start, end, step):
  return start + (end-start) * step

def generate_motion_profile(max_acceleration, max_velocity, distance):
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

  t = np.linspace(0, entire_dt, 200)

  dist_arr = []
  vel_arr = []

  for elapsed_time in t:
    # Distance Calculations
    if (elapsed_time > entire_dt):
      dist_arr.append( distance )

    # if we're accelerating
    elif (elapsed_time < acceleration_dt):
      # use the kinematic equation for acceleration
      dist_arr.append( 0.5 * max_acceleration * elapsed_time ** 2 )

    # if we're cruising
    elif (elapsed_time < deceleration_time):
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
      cruise_current_dt = elapsed_time - acceleration_dt

      # use the kinematic equation for constant velocity
      dist_arr.append( acceleration_distance + max_velocity * cruise_current_dt )

    # if we're decelerating
    else:
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
      cruise_distance = max_velocity * cruise_dt

      # use the kinematic equations to calculate the instantaneous desired position
      dist_arr.append( acceleration_distance + cruise_distance + max_velocity * (elapsed_time - deceleration_time) - 0.5 * max_acceleration * (elapsed_time - deceleration_time) ** 2)
    
    # Velocity Calculations
    if elapsed_time < acceleration_dt:
      vel_arr.append(lerp(0, max_velocity, elapsed_time/acceleration_dt))
    elif elapsed_time > acceleration_dt and elapsed_time < acceleration_dt + cruise_dt:
      vel_arr.append(max_velocity)
    else:
      vel_arr.append(lerp(max_velocity, 0, (elapsed_time-(acceleration_dt + cruise_dt))/acceleration_dt))
  
  return [t, dist_arr, vel_arr]

curr = [0, 0, np.radians(90)]
target = [24, 24, np.radians(90)]

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

max_acceleration = 300 # arbitrary constant
max_velocity = (DESIRED_VOLTAGE/MAX_VOLTAGE) * np.pi * WHEEL_DIAMETER * (RPM / 60.0)

print(max_velocity, max_acceleration)

result = generate_motion_profile(max_acceleration, max_velocity, total_dist)

axis[0][1].plot(result[0], result[2])
axis[0][1].set_title("Velocity")

axis[0][0].plot(result[0], result[1])
axis[0][0].set_title("Distance Travelled")

axis[1][0].plot(x_val, y_val)
axis[1][0].set_title("Path")

plt.show()

