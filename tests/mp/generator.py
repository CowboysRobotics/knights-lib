import matplotlib.pyplot as plt
import numpy as np

def curvature(pt1, pt2, pt3):
    # Calculate midpoints of segments between points
    mx1 = (pt1[0] + pt2[0]) / 2.0
    my1 = (pt1[1] + pt2[1]) / 2.0
    mx2 = (pt2[0] + pt3[0]) / 2.0
    my2 = (pt2[1] + pt3[1]) / 2.0
    
    # Calculate slopes of segments between points
    try:
        slope1 = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
        slope2 = (pt3[1] - pt2[1]) / (pt3[0] - pt2[0])
    except ZeroDivisionError:
        return 0.0
    
    # Calculate slopes of perpendicular bisectors
    if slope1 == 0 or slope2 == 0:
        return 0.0
    
    perp_slope1 = -1 / slope1
    perp_slope2 = -1 / slope2
    
    # Check if slopes are parallel (i.e., points are collinear)
    if abs(perp_slope1 - perp_slope2) < 1e-6:
        return 0.0
    
    # Calculate y-intercepts of perpendicular bisectors
    b1 = my1 - perp_slope1 * mx1
    b2 = my2 - perp_slope2 * mx2
    
    # Calculate center coordinates
    center_x = (b2 - b1) / (perp_slope1 - perp_slope2)
    center_y = perp_slope1 * center_x + b1
    
    # Calculate radius
    radius = np.sqrt((pt1[0] - center_x) ** 2 + (pt1[1] - center_y) ** 2)
    
    return 1.0 / radius

class QuinticPath:
    def __init__(self, curr, target, curr_tangent, target_tangent, curr_acceleration, target_acceleration):
        """
        Initializes the Hermite spline path.

        :param curr: (x, y) coordinates of the current point
        :param target: (x, y) coordinates of the target point
        :param curr_tangent: (dx, dy) tangent vector at the current point
        :param target_tangent: (dx, dy) tangent vector at the target point
        :param curr_acceleration: (a) acceleration at current point
        :param target_acceleration: (a) acceleration at target point

        """
        self.curr = curr
        self.target = target
        self.curr_tangent = curr_tangent
        self.target_tangent = target_tangent
        self.curr_acceleration = curr_acceleration
        self.target_acceleration = target_acceleration

    def p00(self):
        return curr[0]

    def p01(self):
        return self.p00() + curr_tangent[0] / 5

    def p02(self):
        return curr_acceleration / 20 + 2 * self.p01() - self.p00()

    def p03(self):
        return target_acceleration / 20 + 2 * self.p04() - self.p05()

    def p04(self):
        return self.p05() - target_tangent[0] / 5

    def p05(self):
        return target[0]


    def p10(self):
        return curr[1]

    def p11(self):
        return self.p10() + curr_tangent[1] / 5

    def p12(self):
        return curr_acceleration / 20 + 2 * self.p11() - self.p10()

    def p13(self):
        return target_acceleration / 20 + 2 * self.p14() - self.p15()

    def p14(self):
        return self.p15() - target_tangent[1] / 5

    def p15(self):
        return target[1]

    def position(self, t):
        """
        Computes the (x, y) position on the spline at parameter t.
        :param t: Parameter in [0, 1]
        :return: (x, y) tuple
        """
        x = (
            self.p00() * ((1 - t) ** 5) +
            self.p01() * 5 * ((1 - t) ** 4) * (t) +
            self.p02() * 10 * ((1 - t) ** 3) * (t ** 2) +
            self.p03() * 10 * ((1 - t) ** 2) * (t ** 3) +
            self.p04() * 5 * ((1 - t)) * (t ** 4) +
            self.p05() * (t ** 5)        
        )
        y = (
            self.p10() * ((1 - t) ** 5) +
            self.p11() * 5 * ((1 - t) ** 4) * (t) +
            self.p12() * 10 * ((1 - t) ** 3) * (t ** 2) +
            self.p13() * 10 * ((1 - t) ** 2) * (t ** 3) +
            self.p14() * 5 * ((1 - t)) * (t ** 4) +
            self.p15() * (t ** 5)
        )
        return x,y


    def derivatives(self, t):
        """
        Computes the (dx, dy) position on the spline at parameter t.
        :param t: Parameter in [0, 1]
        :return: (dx, dy) tuple
        """
        dx = (
            self.p01() * 5 * ((1 - t) ** 4) - 
            self.p00() * 5 * ((1 - t) ** 4) +
            self.p02() * 20 * ((1 - t) ** 3) * (t) - 
            self.p01() * 20 * ((1 - t) ** 3) * (t) +
            self.p03() * 30 * ((1 - t) ** 2) * (t ** 2) - 
            self.p02() * 30 * ((1 - t) ** 2) * (t ** 2) +
            self.p04() * 20 * ((1 - t)) * (t ** 3) - 
            self.p03() * 20 * ((1 - t)) * (t ** 3) +
            self.p05() * 5 * (t ** 4) - 
            self.p04() * 5 * (t ** 4)
        )
        dy = (
            self.p11() * 5 * ((1 - t) ** 4) - 
            self.p10() * 5 * ((1 - t) ** 4) +
            self.p12() * 20 * ((1 - t) ** 3) * (t) - 
            self.p11() * 20 * ((1 - t) ** 3) * (t) +
            self.p13() * 30 * ((1 - t) ** 2) * (t ** 2) - 
            self.p12() * 30 * ((1 - t) ** 2) * (t ** 2) +
            self.p14() * 20 * ((1 - t)) * (t ** 3) - 
            self.p13() * 20 * ((1 - t)) * (t ** 3) +
            self.p15() * 5 * (t ** 4) - 
            self.p14() * 5 * (t ** 4)
        )
        return dx,dy


    def second_derivatives(self, t):
        """
        Computes the second derivatives (d2x/dt2, d2y/dt2) at parameter t.
        :param t: Parameter in [0, 1]
        :return: (d2x/dt2, d2y/dt2) tuple
        """
        dx2 = (
            20*(self.p02() - 2 * self.p01() + self.p00()) * ((1 - t) ** 3) +
            60*(self.p03() - 2 * self.p02() + self.p01()) * ((1 - t) ** 2) * t +
            60*(self.p04() - 2 * self.p03() + self.p02()) * ((1 - t)) * (t ** 2) +
            20*(self.p05() - 2 * self.p04() + self.p03()) * (t **3)          
        )
        dy2 = (
            20*(self.p12() - 2 * self.p11() + self.p10()) * ((1 - t) ** 3) +
            60*(self.p13() - 2 * self.p12() + self.p11()) * ((1 - t) ** 2) * t +
            60*(self.p14() - 2 * self.p13() + self.p12()) * ((1 - t)) * (t ** 2) +
            20*(self.p15() - 2 * self.p14() + self.p13()) * (t **3)
        )
        return dx2, dy2

def dist_between(x1, y1, x2, y2):
  return np.hypot(x2-x1, y2-y1)

def lerp(start, end, step):
  return start + (end-start) * step

def generate_motion_profile(max_acceleration, max_velocity, distance, track_width, path):
  # Calculate the time it takes to accelerate to max velocity
  acceleration_time = max_velocity / max_acceleration

  # If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
  halfway_distance = distance / 2
  acceleration_distance = 0.5 * max_acceleration * acceleration_time ** 2

  if (acceleration_distance > halfway_distance):
    acceleration_time = np.sqrt(halfway_distance / (0.5 * max_acceleration))
    cruise_time = 0
    total_time = 2 * acceleration_time
  else:
    cruise_time = distance / max_velocity - acceleration_time
    total_time = cruise_time + 2 * acceleration_time

  # print(total_time)
  
  # we decelerate at the same rate as we accelerate
  deceleration_time = acceleration_time

  deceleration_distance = 0.5 * max_acceleration * deceleration_time ** 2

  # calculate the time that we're at max velocity
  cruise_distance = max_velocity * cruise_time
  deceleration_time = acceleration_time + cruise_time

  # check if we're still in the motion profile
  
  dist_arr = []
  vel_arr = []

  side_vel_arr = []
  omega_arr = []
  theta_arr = []
  position_arr = []

  times = []

  x = 0
  y = 0

  elapsed_time = 0

  time_step = total_time / 300

  while elapsed_time < total_time:
  # for elapsed_time in range(0.0, total_time, total_time/30):

    curr_dist = 0

    max_speed = 0

    curr_acceleration = max_acceleration

    curvature_at = curvature(path.position((elapsed_time-time_step) / total_time), path.position((elapsed_time) / total_time), path.position((elapsed_time+time_step) / total_time))
    if curvature_at > 1e-6:
      curr_acceleration = min(2/curvature_at, curr_acceleration)
    else:
       curr_acceleration = min(1e4, curr_acceleration)
    
    print(curr_acceleration, curvature_at)
    

    # Distance Calculations
    if (elapsed_time > total_time):
      curr_dist = distance
      curr_velocity = 0
      max_speed = 0
    # if we're accelerating
    elif (elapsed_time < acceleration_time):
      # use the kinematic equation for acceleration
      curr_dist = 0.5 * curr_acceleration * elapsed_time ** 2
      curr_velocity = curr_acceleration * elapsed_time
      max_speed = max_acceleration * elapsed_time
    # if we're cruising
    elif (cruise_time > 0 and elapsed_time < (acceleration_time + cruise_time)):
      acceleration_distance = 0.5 * curr_acceleration * acceleration_time ** 2
      cruise_current_time = elapsed_time - acceleration_time

      # use the kinematic equation for constant velocity
      curr_dist = acceleration_distance + max_velocity * cruise_current_time
      curr_velocity = max_velocity
      max_speed = max_velocity
    # if we're decelerating
    else:
      acceleration_distance = 0.5 * curr_acceleration * acceleration_time ** 2
      cruise_distance = max_velocity * cruise_time
      deceleration_curr_time = (elapsed_time - acceleration_time - cruise_time)
          
      # use the kinematic equations to calculate the instantaneous desired position
      curr_dist = acceleration_distance + cruise_distance + max_velocity * deceleration_curr_time - curr_acceleration * (deceleration_curr_time ** 2) / 2
      curr_velocity = max_velocity - curr_acceleration * (deceleration_curr_time) # inaccuracy here maybe
      max_speed = max_velocity - max_acceleration * (deceleration_curr_time) # inaccuracy here maybe
        
    # Angular Calculations
    x,y = path.position(elapsed_time / total_time)
    dx,dy = path.derivatives(elapsed_time / total_time)
    dx2,dy2 = path.second_derivatives(elapsed_time / total_time)

    # # VELOCITY CURVING
    # curvature_at = curvature(path.position((elapsed_time-time_step) / total_time), [x,y], path.position((elapsed_time+time_step) / total_time))
    # if curvature_at > 1e-6:
    #   max_speed = 1.5/curvature_at
    # else:
    #    max_speed = 1e4
    
    # print(max_speed, curvature_at)
    
    # if (max_speed < curr_velocity):
    #   added_distance = ((curr_velocity - max_speed) * time_step) # in inches
    #   curr_dist -= added_distance

    #   curr_velocity = max_speed

    #   total_time += added_distance / curr_velocity

    #   # need to factor in the other times here, not just total, as this makes velocity inaccurate

    #   if (elapsed_time > total_time):
    #     break;
    #   # accelerating
    #   elif (elapsed_time < acceleration_time):
    #     acceleration_time += added_distance / curr_velocity
    #   # cruising
    #   elif (cruise_time > 0 and elapsed_time < (acceleration_time + cruise_time)):
    #     cruise_time += added_distance / curr_velocity
    # # END

    # ACCEL CURVING 
    if (curr_velocity < max_speed):
      added_distance = ((max_speed - curr_velocity) * time_step) # in inches
      curr_dist -= added_distance

      total_time += added_distance / max_speed

      # need to factor in the other times here, not just total, as this makes velocity inaccurate

      if (elapsed_time > total_time):
        break;
      # accelerating
      elif (elapsed_time < acceleration_time):
        acceleration_time += added_distance / max_speed
      # cruising
      elif (cruise_time > 0 and elapsed_time < (acceleration_time + cruise_time)):
        cruise_time += added_distance / max_speed
    # END

    # Re calculate Angular Calculations with new total time
    x,y = path.position(elapsed_time / total_time)
    dx,dy = path.derivatives(elapsed_time / total_time)
    dx2,dy2 = path.second_derivatives(elapsed_time / total_time)

    theta = (np.arctan2(dy,dx))
    omega = ((dy2 * dx - dy * dx2) / (((dx) ** 2) * (1 + ((dy / dx)) ** 2) + 1e-6))

    dist_arr.append(curr_dist)
    vel_arr.append(curr_velocity)
    omega_arr.append(omega)
    theta_arr.append(theta)
    position_arr.append((x,y))
    times.append(elapsed_time)
        
    left_vel = curr_velocity - (omega * track_width / 2)
    right_vel = curr_velocity + (omega * track_width / 2)

    side_vel_arr.append((left_vel, right_vel))

    elapsed_time += time_step

  return [times, dist_arr, vel_arr, omega_arr, side_vel_arr, position_arr]

curr = [12, 12, np.radians(90), 0]


target = [-12, 60, np.radians(90), 0]

dist = np.sqrt(((target[0]-curr[0]) ** 2) + ((target[1]-curr[1]) ** 2))

curr_tangent = (np.cos(curr[2]) * dist, np.sin(curr[2]) * dist)
target_tangent = (np.cos(target[2]) * dist, np.sin(target[2]) * dist)

curr_pos = [curr[0], curr[1]]

curr_acceleration = dist * curr[3]
target_acceleration = dist * target[3]

path = QuinticPath((curr[0], curr[1]), (target[0], target[1]), curr_tangent, target_tangent, curr_acceleration, target_acceleration)

t = np.linspace(0, 1, 300)
x_vals,y_vals = path.position(t)
dx,dy = path.derivatives(t)
dx2,dy2 = path.second_derivatives(t)

total_dist = 0 
for val in t:
    x,y = path.position(val)
    dx,dy = path.derivatives(val)
    dx2,dy2 = path.second_derivatives(val)
    total_dist += dist_between(curr_pos[0], curr_pos[1], x, y)
    curr_pos = [x, y]

DESIRED_VOLTAGE = 80
MAX_VOLTAGE = 127
WHEEL_DIAMETER = 3.25
RPM = 450
TRACK_WIDTH = 15

max_acceleration = 70 # arbitrary constant
max_velocity = (DESIRED_VOLTAGE/MAX_VOLTAGE) * np.pi * WHEEL_DIAMETER * (RPM / 60.0)

t, dist_arr, vel_arr, omega_arr, side_vel_arr, position_arr = generate_motion_profile(max_acceleration, max_velocity, total_dist, TRACK_WIDTH, path)

figure, axis = plt.subplots(3, 2)

axis[0][1].plot(t, vel_arr)
axis[0][1].set_title("Velocity")

axis[0][0].plot(t, dist_arr)
axis[0][0].set_title("Distance Travelled")

axis[1][0].plot(x_vals, y_vals)
axis[1][0].set_title("Path")

axis[1][1].plot(t, omega_arr)
axis[1][1].set_title("Omega")

left_vels, right_vels = zip(*side_vel_arr)

axis[2][1].plot(t, left_vels, label="left")
axis[2][1].set_title("Side Velocities")
axis[2][1].plot(t, right_vels, label="right")

new_axis = plt.figure().add_subplot(projection='3d')

real_x, real_y = zip(*position_arr)

new_axis.plot(real_x, real_y, t, label="Actual Path")

plt.legend()
plt.show()