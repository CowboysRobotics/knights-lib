import sys
import numpy as np

class Differential:
    def __init__(self, wheel_diameter, wheel_base):
        self.wheel_radius = wheel_diameter/2  # Radius of the wheels
        self.wheel_base = wheel_base      # Distance between the wheels
        self.x = 0.0                      # Initial x position
        self.y = 0.0                      # Initial y position
        self.theta = np.pi/2                 # Initial orientation

    def update(self, v_l, v_r, dt):
        # Compute the linear and angular velocity
        v = self.wheel_radius * (v_r + v_l) / 2.0
        omega = self.wheel_radius * (v_r - v_l) / self.wheel_base

        # Update the position and orientation
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

    def get_position(self):
        return self.x, self.y, self.theta
    
route = []
done = False

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

robot = Differential(3.25, 16.0)


