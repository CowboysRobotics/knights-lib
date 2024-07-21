import matplotlib.pyplot as plt
import numpy as np

def dist(pt1, pt2):
    return np.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])

route = []

file = open("tests/mptestroute.txt", "r")

for line in file:
    route.append([float(line.split()[0]), float(line.split()[1])])

total_dist = 0

for i,point in enumerate(route):
    if i > 0:
        total_dist += dist(route[i-1], route[i])



