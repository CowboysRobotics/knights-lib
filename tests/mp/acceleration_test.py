import re
import matplotlib.pyplot as plt
import numpy as np

from scipy.ndimage import median_filter
from scipy.signal import savgol_filter

def parse_debug_log(log):
    pattern = (r"time:\s*(?P<time>[-\d.]+)\s*"
               r"pos:\s*(?P<x>[-\d.]+)\s+(?P<y>[-\d.]+)\s+(?P<theta>[-\d.]+)\s*"
               )
    
    match = re.match(pattern, log)
    if match:
        return {key: float(value) for key, value in match.groupdict().items()}
    else:
        return None

# Read log from file and process multiple lines into separate arrays
def read_log_file(filename):
    with open(filename, "r") as file:
        logs = file.readlines()
    
    parsed_data = {
        "x": [], "y": [], "theta": [], "time": []
    }

    for log in logs:
        parsed_log = parse_debug_log(log.strip())
        if parsed_log:
            for key in parsed_log:
                parsed_data[key].append(parsed_log[key])
    
    return parsed_data

# Example usage
filename = "tests\mp\odom_log.txt"
decoded_values = read_log_file(filename)

for i in range(len(decoded_values["time"])):
    decoded_values["time"][i] /= 1000

raw_velocity = np.gradient(decoded_values["y"], decoded_values["time"], axis=0)

velocity = savgol_filter(raw_velocity, window_length=11, polyorder=2)
velocity = np.clip(velocity, -85, 85)

raw_acceleration = np.gradient(velocity, decoded_values["time"], axis=0)

acceleration = savgol_filter(raw_acceleration, window_length=11, polyorder=2)
acceleration = np.clip(acceleration, -1000, 1000)

print(np.mean(acceleration))

figure, axis = plt.subplots(2, 2)

axis[0][1].plot(decoded_values["time"], decoded_values["y"])
axis[0][1].set_title("Position")

axis[0][0].plot(decoded_values["time"], decoded_values["theta"])
axis[0][0].set_title("Theta")

axis[1][0].plot(decoded_values["time"], velocity)
axis[1][0].set_title("Velocity")

axis[1][1].plot(decoded_values["time"], acceleration)
axis[1][1].set_title("Acceleration")

plt.legend()
plt.show()


