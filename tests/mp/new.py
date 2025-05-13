import matplotlib.pyplot as plt
import numpy as np
import warnings

# Suppress RuntimeWarnings (like division by zero in curvature if needed)
warnings.filterwarnings("ignore", category=RuntimeWarning)

def dist_between(x1, y1, x2, y2):
    return np.hypot(x2 - x1, y2 - y1)

class QuinticHermiteSpline:
    """
    Represents a 2D Quintic Hermite Spline using Bézier control points.
    Ensures C2 continuity (position, velocity, acceleration) at endpoints.
    """
    def __init__(self, p0, v0, a0, p1, v1, a1):
        """
        Initializes the spline using endpoint conditions.

        :param p0: Start position (x, y) or [x, y]
        :param v0: Start velocity (dx/dt, dy/dt) or [dx/dt, dy/dt]
        :param a0: Start acceleration (d2x/dt2, d2y/dt2) or [d2x/dt2, d2y/dt2]
        :param p1: End position (x, y) or [x, y]
        :param v1: End velocity (dx/dt, dy/dt) or [dx/dt, dy/dt]
        :param a1: End acceleration (d2x/dt2, d2y/dt2) or [d2x/dt2, d2y/dt2]
        """
        p0, v0, a0 = np.array(p0), np.array(v0), np.array(a0)
        p1, v1, a1 = np.array(p1), np.array(v1), np.array(a1)

        # Calculate Bézier control points from Hermite conditions
        # P0 = p0
        # P1 = p0 + v0 / 5
        # P2 = p0 + (2/5)*v0 + (1/20)*a0
        # P3 = p1 - (2/5)*v1 + (1/20)*a1
        # P4 = p1 - v1 / 5
        # P5 = p1
        self.P = np.zeros((6, 2)) # 6 control points, 2 dimensions (x, y)
        self.P[0] = p0
        self.P[1] = p0 + v0 / 5.0
        self.P[2] = p0 + (2.0/5.0) * v0 + a0 / 20.0
        self.P[3] = p1 - (2.0/5.0) * v1 + a1 / 20.0
        self.P[4] = p1 - v1 / 5.0
        self.P[5] = p1

        # Precompute coefficients for position, velocity, acceleration
        # B(t) = P0*(1-t)^5 + P1*5*t(1-t)^4 + P2*10*t^2(1-t)^3 + ...
        # B'(t) = 5*(P1-P0)(1-t)^4 + 20*(P2-P1)t(1-t)^3 + ...
        # B''(t) = 20*(P2-2P1+P0)(1-t)^3 + 60*(P3-2P2+P1)t(1-t)^2 + ...
        self.dP = 5 * (self.P[1:] - self.P[:-1])
        self.ddP = 4 * (self.dP[1:] - self.dP[:-1]) # Note: This is 20*(Pi+2 - 2Pi+1 + Pi)


    def _bernstein(self, n, k, t):
        """Evaluates the k-th Bernstein polynomial of degree n at t."""
        return np.math.comb(n, k) * (t**k) * ((1 - t)**(n - k))

    def position(self, t):
        """
        Computes the (x, y) position on the spline at parameter t.
        Vectorized for t as a numpy array.
        :param t: Parameter in [0, 1], scalar or numpy array
        :return: (x, y) tuple or np.array of shape (len(t), 2)
        """
        t = np.asarray(t)
        if t.ndim == 0: # Scalar input
            coeffs = np.array([self._bernstein(5, k, t) for k in range(6)])
            return coeffs @ self.P
        else: # Vector input
            pos = np.zeros((len(t), 2))
            for k in range(6):
                coeffs = self._bernstein(5, k, t)
                pos += np.outer(coeffs, self.P[k])
            return pos

    def derivative(self, t):
        """
        Computes the first derivative (dx/dt, dy/dt) at parameter t.
        Vectorized for t as a numpy array.
        :param t: Parameter in [0, 1], scalar or numpy array
        :return: (dx/dt, dy/dt) tuple or np.array of shape (len(t), 2)
        """
        t = np.asarray(t)
        if t.ndim == 0: # Scalar input
             coeffs = np.array([self._bernstein(4, k, t) for k in range(5)])
             return coeffs @ self.dP
        else: # Vector input
            deriv = np.zeros((len(t), 2))
            for k in range(5):
                coeffs = self._bernstein(4, k, t)
                deriv += np.outer(coeffs, self.dP[k])
            return deriv


    def second_derivative(self, t):
        """
        Computes the second derivative (d2x/dt2, d2y/dt2) at parameter t.
        Vectorized for t as a numpy array.
        :param t: Parameter in [0, 1], scalar or numpy array
        :return: (d2x/dt2, d2y/dt2) tuple or np.array of shape (len(t), 2)
        """
        t = np.asarray(t)
        if t.ndim == 0: # Scalar input
             coeffs = np.array([self._bernstein(3, k, t) for k in range(4)])
             return coeffs @ self.ddP
        else: # Vector input
            sec_deriv = np.zeros((len(t), 2))
            for k in range(4):
                coeffs = self._bernstein(3, k, t)
                sec_deriv += np.outer(coeffs, self.ddP[k])
            return sec_deriv

    def curvature(self, t):
        """
        Computes the signed curvature k at parameter t.
        k = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
        Vectorized for t as a numpy array.
        :param t: Parameter in [0, 1], scalar or numpy array
        :return: Curvature k, scalar or numpy array
        """
        d1 = self.derivative(t) # shape (N, 2) or (2,)
        d2 = self.second_derivative(t) # shape (N, 2) or (2,)

        # Handle potential scalar input by ensuring 2D shape temporarily
        was_scalar = False
        if d1.ndim == 1:
            was_scalar = True
            d1 = d1[np.newaxis, :]
            d2 = d2[np.newaxis, :]

        xp, yp = d1[:, 0], d1[:, 1]
        xpp, ypp = d2[:, 0], d2[:, 1]

        numerator = xp * ypp - yp * xpp
        denominator = (xp**2 + yp**2)**1.5

        # Avoid division by zero/very small numbers (where curvature is ill-defined or zero)
        # Threshold can be adjusted based on required precision
        k = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator > 1e-9)

        return k[0] if was_scalar else k

    def arc_length(self, t_start=0, t_end=1, num_steps=100):
        """
        Approximate arc length using numerical integration (trapezoidal rule).
        :param t_start: Starting parameter t
        :param t_end: Ending parameter t
        :param num_steps: Number of steps for integration
        :return: Approximate arc length
        """
        t_vals = np.linspace(t_start, t_end, num_steps + 1)
        derivatives = self.derivative(t_vals) # shape (num_steps+1, 2)
        speeds = np.linalg.norm(derivatives, axis=1) # magnitude of velocity vector |dr/dt|
        
        # Trapezoidal rule for integration of speed dt
        arc_len = np.trapz(speeds, t_vals)
        return arc_len

# --- Motion Profile Generation ---

def generate_constrained_profile(path, num_samples, max_vel, max_accel, max_centripetal_accel=None):
    """
    Generates a velocity profile along a path, respecting kinematic constraints.

    Args:
        path (QuinticHermiteSpline): The path object.
        num_samples (int): Number of points to sample along the path.
        max_vel (float): Maximum allowable linear velocity.
        max_accel (float): Maximum allowable linear acceleration/deceleration.
        max_centripetal_accel (float, optional): Max centripetal acceleration.
                                                  If None, curvature constraint is ignored.

    Returns:
        tuple: (times, positions, velocities, curvatures, omegas)
               - times: np.array of timestamps for each point.
               - positions: np.array of (x, y) positions.
               - velocities: np.array of linear velocities (ds/d_tau).
               - curvatures: np.array of path curvatures.
               - omegas: np.array of angular velocities (d_theta/d_tau).
    """
    t_vals = np.linspace(0, 1, num_samples)
    
    # 1. Sample path properties
    positions = path.position(t_vals)            # shape (N, 2)
    derivatives = path.derivative(t_vals)      # shape (N, 2) dx/dt, dy/dt
    curvatures = path.curvature(t_vals)          # shape (N,)
    
    # 2. Calculate segment lengths (ds) - use distance between sampled points
    segment_lengths = np.zeros(num_samples)
    segment_lengths[1:] = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    total_dist = np.sum(segment_lengths)
    print(f"Total approximate path length: {total_dist:.3f}")

    # 3. Calculate velocity constraints
    constrained_vel = np.full(num_samples, max_vel)

    if max_centripetal_accel is not None and max_centripetal_accel > 1e-6:
        # Avoid division by zero for zero curvature (straight lines)
        # Set velocity constraint to infinity where curvature is near zero
        # Use np.abs because radius = 1/|k|
        curvature_limit_vel = np.sqrt(max_centripetal_accel / (np.abs(curvatures) + 1e-9)) # Add epsilon
        constrained_vel = np.minimum(constrained_vel, curvature_limit_vel)

    # Ensure start and end velocity are near zero (can be adjusted if needed)
    constrained_vel[0] = 1e-6
    constrained_vel[-1] = 1e-6 # Target zero velocity at the very end

    # 4. Forward pass (apply acceleration constraint)
    current_vel = 0.0
    profiled_vel_fwd = np.zeros(num_samples)
    profiled_vel_fwd[0] = current_vel # Start at 0 velocity
    for i in range(1, num_samples):
        ds = segment_lengths[i]
        max_reachable_vel = np.sqrt(profiled_vel_fwd[i-1]**2 + 2 * max_accel * ds)
        current_vel = min(constrained_vel[i], max_reachable_vel)
        profiled_vel_fwd[i] = current_vel
        
    # 5. Backward pass (apply deceleration constraint)
    current_vel = 0.0
    profiled_vel = np.zeros(num_samples)
    profiled_vel[-1] = current_vel # End at 0 velocity
    for i in range(num_samples - 2, -1, -1):
        ds = segment_lengths[i+1] # Look at the segment leading *to* the next point
        max_reachable_vel = np.sqrt(profiled_vel[i+1]**2 + 2 * max_accel * ds) # Using max_accel for deceleration limit too
        # Velocity at point i must be low enough to reach point i+1's velocity *and*
        # respect the forward pass/curvature constraints
        current_vel = min(profiled_vel_fwd[i], max_reachable_vel)
        profiled_vel[i] = current_vel

    # Correct the final velocity based on backward pass constraint (should be near zero)
    profiled_vel[-1] = 1e-6

    # 6. Calculate timestamps
    times = np.zeros(num_samples)
    for i in range(1, num_samples):
        ds = segment_lengths[i]
        v_avg = (profiled_vel[i] + profiled_vel[i-1]) / 2.0
        if v_avg < 1e-6: # Avoid division by zero if velocity is tiny
            dt = 0.0 # Assign zero time delta if segment effectively has zero velocity
        else:
            dt = ds / v_avg
        times[i] = times[i-1] + dt
        
    # 7. Calculate angular velocity (omega = v * k)
    omegas = profiled_vel * curvatures

    return times, positions, profiled_vel, curvatures, omegas


# --- Parameters ---
# Pose: [x, y, heading_radians, curvature(k=1/R) or 0] - Using 0 for acceleration for now
# Or: [x, y, heading_radians, velocity] # Let's use this for simplicity
# For Quintic, we ideally need Pos, Vel, Accel vectors at endpoints.
# Let's derive initial Accel=0 unless specified

# Start Pose: x=0, y=0, heading=0 deg
start_pos = [0, 0]
start_heading = np.radians(0)
start_vel_magnitude = 0 # Starting from rest
start_accel_magnitude = 0 # Starting with zero acceleration

# End Pose: x=24, y=24, heading=90 deg
target_pos = [24, 24]
target_heading = np.radians(90)
target_vel_magnitude = 0 # Ending at rest
target_accel_magnitude = 0 # Ending with zero acceleration

# Estimate a scale factor for the derivatives based on distance
# This helps shape the curve but isn't strictly required if accel is zero
dist_heuristic = np.hypot(target_pos[0] - start_pos[0], target_pos[1] - start_pos[1])
# If start/end velocity is non-zero, use it directly. If zero, use heuristic tangent.
start_vel_vec = [start_vel_magnitude * np.cos(start_heading) * dist_heuristic,
                 start_vel_magnitude * np.sin(start_heading) * dist_heuristic] if start_vel_magnitude > 1e-6 else [np.cos(start_heading) * dist_heuristic, np.sin(start_heading) * dist_heuristic]

target_vel_vec = [target_vel_magnitude * np.cos(target_heading) * dist_heuristic,
                  target_vel_magnitude * np.sin(target_heading) * dist_heuristic] if target_vel_magnitude > 1e-6 else [np.cos(target_heading) * dist_heuristic, np.sin(target_heading) * dist_heuristic]

# For simplicity, assume zero acceleration vectors if magnitude is zero
start_accel_vec = [start_accel_magnitude * np.cos(start_heading), start_accel_magnitude * np.sin(start_heading)] # Simplified assumption
target_accel_vec = [target_accel_magnitude * np.cos(target_heading), target_accel_magnitude * np.sin(target_heading)] # Simplified assumption

# Create the path
path = QuinticHermiteSpline(start_pos, start_vel_vec, start_accel_vec,
                            target_pos, target_vel_vec, target_accel_vec)

# --- Robot & Profile Parameters ---
DESIRED_VOLTAGE = 80 # Note: Voltage isn't directly used in this kinematic model
MAX_VOLTAGE = 127
WHEEL_DIAMETER = 3.25 # inches
RPM = 450 # Motor free speed RPM
GEAR_RATIO = 1.0 # Example: Add gear ratio if applicable
TRACK_WIDTH = 12.5970 # inches

# Estimate a realistic max velocity (e.g., 80% of theoretical free speed)
# inches/sec = inches/rev * rev/min * min/sec
max_theoretical_velocity = (np.pi * WHEEL_DIAMETER / GEAR_RATIO) * (RPM / 60.0)
MAX_VELOCITY = max_theoretical_velocity * 0.8 # inches/sec (Adjust factor as needed)

MAX_ACCELERATION = 50 # inches/sec^2 (Tune this value)
MAX_CENTRIPETAL_ACCEL = 60 # inches/sec^2 (Tune this - limits speed in turns)
NUM_SAMPLES = 300 # Number of points to sample the path

# --- Generate Profile ---
times, positions, velocities, curvatures, omegas = generate_constrained_profile(
    path, NUM_SAMPLES, MAX_VELOCITY, MAX_ACCELERATION, MAX_CENTRIPETAL_ACCEL
)

# Calculate Wheel Velocities
left_vels = velocities - (omegas * TRACK_WIDTH / 2.0)
right_vels = velocities + (omegas * TRACK_WIDTH / 2.0)

# Calculate Actual Distance Traveled from Profiled Points
actual_dist_arr = np.zeros_like(times)
actual_dist_arr[1:] = np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))

# Calculate Actual Velocity from Profiled Points/Times (using gradient for approximation)
actual_vel_arr = np.gradient(actual_dist_arr, times, edge_order=2)

# --- Plotting ---
fig, axs = plt.subplots(3, 3, figsize=(15, 12))
fig.suptitle('Improved Motion Profile Analysis')

# Plot Path Geometry
x_coords, y_coords = positions[:, 0], positions[:, 1]
axs[0, 0].plot(x_coords, y_coords, label='Profiled Path')
axs[0, 0].set_title("Path Geometry")
axs[0, 0].set_xlabel("X (inches)")
axs[0, 0].set_ylabel("Y (inches)")
axs[0, 0].axis('equal')
axs[0, 0].grid(True)
axs[0, 0].legend()

# Plot Profiled Velocity vs Time
axs[0, 1].plot(times, velocities, label='Center Velocity (Profiled)')
axs[0, 1].set_title("Linear Velocity vs Time")
axs[0, 1].set_xlabel("Time (s)")
axs[0, 1].set_ylabel("Velocity (in/s)")
axs[0, 1].grid(True)
axs[0, 1].legend()

# Plot Distance vs Time
axs[0, 2].plot(times, actual_dist_arr, label='Distance Travelled')
axs[0, 2].set_title("Distance vs Time")
axs[0, 2].set_xlabel("Time (s)")
axs[0, 2].set_ylabel("Distance (in)")
axs[0, 2].grid(True)
axs[0, 2].legend()


# Plot Angular Velocity vs Time
axs[1, 0].plot(times, omegas, label='Omega (Profiled)')
axs[1, 0].set_title("Angular Velocity vs Time")
axs[1, 0].set_xlabel("Time (s)")
axs[1, 0].set_ylabel("Omega (rad/s)")
axs[1, 0].grid(True)
axs[1, 0].legend()

# Plot Wheel Velocities vs Time
axs[1, 1].plot(times, left_vels, label='Left Wheel Velocity')
axs[1, 1].plot(times, right_vels, label='Right Wheel Velocity')
axs[1, 1].set_title("Wheel Velocities vs Time")
axs[1, 1].set_xlabel("Time (s)")
axs[1, 1].set_ylabel("Velocity (in/s)")
axs[1, 1].grid(True)
axs[1, 1].legend()

# Plot Curvature vs Distance (or Time)
axs[1, 2].plot(actual_dist_arr, curvatures, label='Curvature')
axs[1, 2].set_title("Path Curvature vs Distance")
axs[1, 2].set_xlabel("Distance (in)")
axs[1, 2].set_ylabel("Curvature (1/in)")
axs[1, 2].grid(True)
axs[1, 2].legend()

# Plot Calculated Actual Velocity vs Time
axs[2, 0].plot(times, actual_vel_arr, label='Actual Velocity (Calculated)')
axs[2, 0].set_title("Actual Velocity vs Time (from Pos/Time)")
axs[2, 0].set_xlabel("Time (s)")
axs[2, 0].set_ylabel("Velocity (in/s)")
axs[2, 0].grid(True)
axs[2, 0].legend()

# Plot Velocity vs Distance
axs[2, 1].plot(actual_dist_arr, velocities, label='Profiled Velocity')
axs[2, 1].set_title("Velocity vs Distance")
axs[2, 1].set_xlabel("Distance (in)")
axs[2, 1].set_ylabel("Velocity (in/s)")
axs[2, 1].grid(True)
axs[2, 1].legend()

# Time step analysis
time_diffs = np.diff(times, prepend=0)
axs[2, 2].plot(times, time_diffs, label='Time Step')
axs[2, 2].set_title("Time Between Points vs Time")
axs[2, 2].set_xlabel("Time (s)")
axs[2, 2].set_ylabel("Delta Time (s)")
axs[2, 2].grid(True)
axs[2, 2].legend()


plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout to prevent title overlap


# Optional 3D Plot (Path vs Time)
fig_3d = plt.figure()
ax_3d = fig_3d.add_subplot(111, projection='3d')
ax_3d.plot(x_coords, y_coords, times, label='Path over Time')
ax_3d.set_xlabel('X (inches)')
ax_3d.set_ylabel('Y (inches)')
ax_3d.set_zlabel('Time (s)')
ax_3d.set_title('3D Path vs Time')
plt.legend()

plt.show()