import sys
import numpy as np
import matplotlib.pyplot as plt

def dist_btwn(pt1, pt2):
    return np.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1])

rpm = 450 # rounds per min
wheel_radius = 3.25/2 # inches
wheel_circumfrence = 2 * np.pi * wheel_radius

max_speed = rpm * wheel_circumfrence * (1/60) # speed in inches per second
    
route = []
done = False

# first line entered is the starting pos
for line in sys.stdin:
    pos = []
    for var in line.split():
        if var == "End":
            done = True
            break
        pos.append(int(var))
    if done:
        break
    route.append(pos)

route_dist = 0.0

for i,point in enumerate(route):
    if i > 0:
        route_dist += dist_btwn(point, route[i-1])

print(route_dist)

# https://www.ctrlaltftc.com/advanced/motion-profiling#trapezoidal-motion-profiles

max_accel = 32 # meteres per second squared - delta velocity / delta time

def motion_profile(max_acceleration, max_velocity, distance, time):
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
  if time > entire_dt:
    return distance

  # if we're accelerating
  if (time < acceleration_dt):
    # use the kinematic equation for acceleration
    return 0.5 * max_acceleration * time ** 2

  # if we're cruising
  elif (time < deceleration_time):
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
    cruise_current_dt = time - acceleration_dt

    # use the kinematic equation for constant velocity
    return acceleration_distance + max_velocity * cruise_current_dt

  # if we're decelerating
  else:
    acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
    cruise_distance = max_velocity * cruise_dt
    deceleration_time = time - deceleration_time

    # use the kinematic equations to calculate the instantaneous desired position
    return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2

curr_distance = 0.1
curr_time = 0.1

x = [0]
y1 = [0]
y2 = [0]

print(max_speed)

# Not working
while curr_distance < route_dist:
   
    target = motion_profile(max_accel, max_speed, curr_distance, curr_time)

    curr_time += 0.1
    curr_distance += target * 0.1

    x.append(curr_time)
    y1.append(target)
    y2.append(curr_distance)

plt.plot(x,y1)
plt.title("Time VS Speed")
plt.show()
