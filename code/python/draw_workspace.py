# Draws the workspace of the robot
from jointPub import joint_publisher
import numpy as np
import time

CLOSED_GRIPPER = np.radians(-120)
Q1_LIMIT = np.radians(80)


def draw_min():
    # Draw the minimum workspace
    joint_publisher(-np.pi/2, np.radians(90-135), np.radians(20-180), np.radians(270-180), CLOSED_GRIPPER)
    time.sleep(5)
    joint_publisher(np.pi/2, np.radians(90-135), np.radians(20-180), np.radians(270-180), CLOSED_GRIPPER)

def draw_max():
    # Draw the maximum workspace
    joint_publisher(-Q1_LIMIT, np.radians(-135), np.radians(-15), np.radians(15), CLOSED_GRIPPER)
    time.sleep(5)
    joint_publisher(Q1_LIMIT, np.radians(-135), np.radians(-15), np.radians(15), CLOSED_GRIPPER)

def draw_workspace():
    # Draw the workspace
    draw_min()
    draw_max()

if __name__ == "__main__":
    joint_publisher(0, 0, 0, 0, CLOSED_GRIPPER)
    time.sleep(5)
    draw_max()
    time.sleep(5)
    joint_publisher(0, 0, 0, 0, CLOSED_GRIPPER)
    time.sleep(5)
    draw_min()
    time.sleep(5)
    joint_publisher(0, 0, 0, 0, CLOSED_GRIPPER)