import matplotlib.pyplot as plt
import numpy as np

class HermiteSplinePath:
    def __init__(self, curr, target, curr_tangent, target_tangent):
        """
        Initializes the Hermite spline path.

        :param curr: (x, y) coordinates of the current point
        :param target: (x, y) coordinates of the target point
        :param curr_tangent: (dx, dy) tangent vector at the current point
        :param target_tangent: (dx, dy) tangent vector at the target point
        """
        self.curr = curr
        self.target = target
        self.curr_tangent = curr_tangent
        self.target_tangent = target_tangent

    def h1(self, t):
        return 2 * t**3 - 3 * t**2 + 1

    def h2(self, t):
        return -2 * t**3 + 3 * t**2

    def h3(self, t):
        return t**3 - 2 * t**2 + t

    def h4(self, t):
        return t**3 - t**2

    def h1_prime(self, t):
        return 6 * t**2 - 6 * t

    def h2_prime(self, t):
        return -6 * t**2 + 6 * t

    def h3_prime(self, t):
        return 3 * t**2 - 4 * t + 1

    def h4_prime(self, t):
        return 3 * t**2 - 2 * t

    def h1_prime2(self, t):
        return 12 * t - 6

    def h2_prime2(self, t):
        return -12 * t + 6

    def h3_prime2(self, t):
        return 6 * t - 4

    def h4_prime2(self, t):
        return 6 * t - 2

    def position(self, t):
        """
        Computes the (x, y) position on the spline at parameter t.
        :param t: Parameter in [0, 1]
        :return: (x, y) tuple
        """
        x = (
            self.h1(t) * self.curr[0] +
            self.h2(t) * self.target[0] +
            self.h3(t) * self.curr_tangent[0] +
            self.h4(t) * self.target_tangent[0]
        )
        y = (
            self.h1(t) * self.curr[1] +
            self.h2(t) * self.target[1] +
            self.h3(t) * self.curr_tangent[1] +
            self.h4(t) * self.target_tangent[1]
        )
        dx,dy,_ = self.derivatives(t)
        theta = (
          np.arctan2(dy, dx)
        )
        return x, y, theta

    def derivatives(self, t):
        """
        Computes the derivatives (dx/dt, dy/dt) at parameter t.
        :param t: Parameter in [0, 1]
        :return: (dx/dt, dy/dt) tuple
        """
        dx = (
            self.h1_prime(t) * self.curr[0] +
            self.h2_prime(t) * self.target[0] +
            self.h3_prime(t) * self.curr_tangent[0] +
            self.h4_prime(t) * self.target_tangent[0]
        )
        dy = (
            self.h1_prime(t) * self.curr[1] +
            self.h2_prime(t) * self.target[1] +
            self.h3_prime(t) * self.curr_tangent[1] +
            self.h4_prime(t) * self.target_tangent[1]
        )
        dx2, dy2 = self.second_derivatives(t)
        omega = (dy2 * dx - dy * dx2) / (1 + (dy/dx)**2)
        return dx, dy, omega

    def second_derivatives(self, t):
        """
        Computes the second derivatives (d2x/dt2, d2y/dt2) at parameter t.
        :param t: Parameter in [0, 1]
        :return: (d2x/dt2, d2y/dt2) tuple
        """
        dx2 = (
            self.h1_prime2(t) * self.curr[0] +
            self.h2_prime2(t) * self.target[0] +
            self.h3_prime2(t) * self.curr_tangent[0] +
            self.h4_prime2(t) * self.target_tangent[0]
        )
        dy2 = (
            self.h1_prime2(t) * self.curr[1] +
            self.h2_prime2(t) * self.target[1] +
            self.h3_prime2(t) * self.curr_tangent[1] +
            self.h4_prime2(t) * self.target_tangent[1]
        )
        return dx2, dy2

def dist_between(x1, y1, x2, y2):
  return np.hypot(x2-x1, y2-y1)

def lerp(start, end, step):
  return start + (end-start) * step

def generate_motion_profile(max_acceleration, max_velocity, distance, track_width, path):
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

  side_vel_arr = []
  omega_arr = []
  position_arr = []

  for elapsed_time in t:

    curr_dist = 0

    # Distance Calculations
    if (elapsed_time > entire_dt):
      curr_dist = distance
    # if we're accelerating
    elif (elapsed_time < acceleration_dt):
      # use the kinematic equation for acceleration
      curr_dist = 0.5 * max_acceleration * elapsed_time ** 2
    # if we're cruising
    elif (elapsed_time < deceleration_time):
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
      cruise_current_dt = elapsed_time - acceleration_dt
      # use the kinematic equation for constant velocity
      curr_dist = acceleration_distance + max_velocity * cruise_current_dt
    # if we're decelerating
    else:
      acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
      cruise_distance = max_velocity * cruise_dt
      # use the kinematic equations to calculate the instantaneous desired position
      curr_dist = acceleration_distance + cruise_distance + max_velocity * (elapsed_time - deceleration_time) - 0.5 * max_acceleration * (elapsed_time - deceleration_time) ** 2
    
    dist_arr.append(curr_dist)
    
    velocity = 0

    # Velocity Calculations
    if elapsed_time < acceleration_dt:
      velocity = lerp(0, max_velocity, elapsed_time/acceleration_dt)
    elif elapsed_time > acceleration_dt and elapsed_time < acceleration_dt + cruise_dt:
      velocity = max_velocity
    else:
      velocity = lerp(max_velocity, 0, (elapsed_time-(acceleration_dt + cruise_dt))/acceleration_dt)
    
    vel_arr.append(velocity)
    
    # Angular Calculations
    x,y,theta = path.position(curr_dist/total_dist)
    dx_ds, dy_ds, omega = path.derivatives(curr_dist/total_dist)
    # omega = np.arctan2(dy_ds, dx_ds) * velocity
    omega_arr.append(omega)
    position_arr.append((x,y,theta))

    left_vel = velocity - (omega * track_width / 2)
    right_vel = velocity + (omega * track_width / 2)

    side_vel_arr.append((left_vel, right_vel))

  
  return [t, dist_arr, vel_arr, omega_arr, side_vel_arr, position_arr]

curr = [0, 0, np.radians(90)]
target = [48, 24, np.radians(90)]

dist = np.hypot(target[0]-curr[0], target[1]-curr[1])

curr_tangent = (np.cos(curr[2]) * dist, np.sin(curr[2]) * dist)
target_tangent = (np.cos(target[2]) * dist, np.sin(target[2]) * dist)

path = HermiteSplinePath((curr[0], curr[1]), (target[0], target[1]), curr_tangent, target_tangent)

curr_pos = [curr[0], curr[1]]

t = np.linspace(0, 1, 300)
x_vals,y_vals,theta_vals = path.position(t)

total_dist = 0 
for val in t:
    x,y,theta = path.position(val)
    total_dist += dist_between(curr_pos[0], curr_pos[1], x, y)
    curr_pos = [x, y]

DESIRED_VOLTAGE = 120
MAX_VOLTAGE = 127
WHEEL_DIAMETER = 2.75
RPM = 450
TRACK_WIDTH = 0.381

max_acceleration = 300 # arbitrary constant
max_velocity = (DESIRED_VOLTAGE/MAX_VOLTAGE) * np.pi * WHEEL_DIAMETER * (RPM / 60.0)

print(max_velocity, max_acceleration)

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

axis[2][0].plot(t, left_vels)
axis[2][0].set_title("Left Velocities")

axis[2][1].plot(t, right_vels)
axis[2][1].set_title("Right Velocities")

plt.show()

