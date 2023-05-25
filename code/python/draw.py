from jointPub import joint_publisher
from invkine import invkine
import time
import numpy as np

OPEN_GRIPPER = 0.0
CLOSED_GRIPPER = np.radians(-120)
SLEEP_TIME = 4

def go_home():
    joint_publisher(0, 0, 0, 0, CLOSED_GRIPPER)
    time.sleep(SLEEP_TIME)

def draw_from_points(points):
    for point in points:
        print(point)
        angs = invkine(point[0], point[1])
        joint_publisher(angs[0], angs[1], angs[2], angs[3], CLOSED_GRIPPER)
        time.sleep(SLEEP_TIME)

def draw_sqare():
    points = [
        [-80, 275+40],
        [80, 275+40],
        [80, 275-40],
        [-80, 275-40],
        [-80, 275+40],
    ]
    draw_from_points(points)


if __name__ == "__main__":
    go_home()
    draw_sqare()
    go_home()
