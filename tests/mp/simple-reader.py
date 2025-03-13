import re
import matplotlib.pyplot as plt

def parse_debug_log(log):
    pattern = (r"right/left vel (?P<right_vel>[-\d.]+) (?P<left_vel>[-\d.]+) "
               r"final l/a vel (?P<final_lat_vel>[-\d.]+) (?P<final_ang_vel>[-\d.]+) "
               r"curr l/a vel (?P<curr_lat_vel>[-\d.]+) (?P<curr_ang_vel>[-\d.]+) "
               r"curr pos (?P<curr_pos_x>[-\d.]+) (?P<curr_pos_y>[-\d.]+) (?P<curr_pos_theta>[-\d.]+) "
               r"des pos (?P<des_pos_x>[-\d.]+) (?P<des_pos_y>[-\d.]+) (?P<des_pos_theta>[-\d.]+) "
               r"time (?P<time>[-\d.]+)")
    
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
        "right_vel": [], "left_vel": [], "final_lat_vel": [], "final_ang_vel": [],
        "curr_lat_vel": [], "curr_ang_vel": [], "gain": [], "curr_pos_x": [],
        "curr_pos_y": [], "curr_pos_theta": [], "des_pos_x": [], "des_pos_y": [],
        "des_pos_theta": [], "global_err_x": [], "global_err_y": [], "global_err_theta": [],
        "local_err_x": [], "local_err_y": [], "time": []
    }

    for log in logs:
        parsed_log = parse_debug_log(log.strip())
        # cant parse past 1.066 secnds
        if parsed_log:
            for key in parsed_log:
                parsed_data[key].append(parsed_log[key])
    
    return parsed_data

# Example usage
filename = "simple_mp_output.txt"
decoded_values = read_log_file(filename)
# print(decoded_values)

figure, axis = plt.subplots(3, 2)

axis[0][1].plot(decoded_values["time"], decoded_values["final_lat_vel"])
axis[0][1].set_title("Velocity (rpm)")

axis[1][0].plot(decoded_values["curr_pos_x"], decoded_values["curr_pos_y"])
axis[1][0].set_title("Path")

axis[1][1].plot(decoded_values["time"], decoded_values["final_ang_vel"])
axis[1][1].set_title("Omega (rpm)")

axis[2][1].plot(decoded_values["time"], decoded_values["right_vel"], label = "right")
axis[2][1].plot(decoded_values["time"], decoded_values["left_vel"], label = "left")

plt.legend()
plt.show()

