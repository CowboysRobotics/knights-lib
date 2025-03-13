import re
import matplotlib.pyplot as plt

def parse_debug_log(log):
    pattern = (r"time:\s*(?P<time>[-\d.]+)\s*"
               r"pos:\s*(?P<pos_x>[-\d.]+)\s+(?P<pos_y>[-\d.]+)\s+(?P<pos_theta>[-\d.]+)\s*"
               r"lin vel:\s*(?P<linear_velocity>[-\d.]+)\s*"
               r"angular vel:\s*(?P<angular_velocity>[-\d.]+)\s*"
               r"dist:\s*(?P<distance_travelled>[-\d.]+)\s*"
               r"side vels \(r,l\):\s*(?P<right_vel>[-\d.]+)\s+(?P<left_vel>[-\d.]+)"
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
        "time": [], "pos_x": [], "pos_y": [], "pos_theta": [],
        "linear_velocity": [], "angular_velocity": [], "distance_travelled": [], "right_vel": [],
        "left_vel": []
    }

    for log in logs:
        parsed_log = parse_debug_log(log.strip())
        if parsed_log:
            for key in parsed_log:
                parsed_data[key].append(parsed_log[key])
    
    return parsed_data

# Example usage
filename = "motion_output.txt"
decoded_values = read_log_file(filename)

figure, axis = plt.subplots(3, 2)

axis[0][1].plot(decoded_values["time"], decoded_values["linear_velocity"])
axis[0][1].set_title("Velocity (in/s)")

axis[0][0].plot(decoded_values["time"], decoded_values["distance_travelled"])
axis[0][0].set_title("Distance Travelled")

axis[1][0].plot(decoded_values["pos_x"], decoded_values["pos_y"])
axis[1][0].set_title("Path")

axis[1][1].plot(decoded_values["time"], decoded_values["angular_velocity"])
axis[1][1].set_title("Omega (rad/s)")

axis[2][1].plot(decoded_values["time"], decoded_values["right_vel"], label = "right")
axis[2][1].plot(decoded_values["time"], decoded_values["left_vel"], label = "left")

plt.legend()
plt.show()

