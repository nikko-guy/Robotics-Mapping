# """csci3302_lab4 controller."""
# Copyright (2025) University of Colorado Boulder
# CSCI 3302: Introduction to Robotics

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import random
import copy
import numpy as np
from controller import Robot, Motor, DistanceSensor

state = "line_follower"  # Change this to anything else to stay in place to test coordinate transform functions

LIDAR_SENSOR_MAX_RANGE = 3  # Meters
LIDAR_ANGLE_BINS = 21  # 21 Bins to cover the angular range of the lidar, centered at 10
LIDAR_ANGLE_RANGE = 1.5708  # 90 degrees, 1.5708 radians

# These are your pose values that you will update by solving the odometry equations
start_pose_x = 0.21
start_pose_y = 0.678
pose_x = start_pose_x
pose_y = start_pose_y
pose_theta = -np.pi

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053  # ePuck's wheels are 53mm apart.
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [
    robot.getDevice("gs0"),
    robot.getDevice("gs1"),
    robot.getDevice("gs2"),
]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Initialize the Display
display = robot.getDevice("display")

# get and enable lidar
lidar = robot.getDevice("LDS-01")
lidar.enable(SIM_TIMESTEP)
lidar.enablePointCloud()

##### DO NOT MODIFY ANY CODE ABOVE THIS #####

##### Part 1: Setup Data structures
#
# Create an empty list for your lidar sensor readings here,
# as well as an array that contains the angles of each ray
# in radians. The total field of view is LIDAR_ANGLE_RANGE,
# and there are LIDAR_ANGLE_BINS. An easy way to generate the
# array that contains all the angles is to use linspace from
# the numpy package.

lidar_sensor_readings = []
lidar_offsets = np.linspace(
    LIDAR_ANGLE_RANGE / 2, -LIDAR_ANGLE_RANGE / 2, LIDAR_ANGLE_BINS
)

# print("Lidar Angles: ", lidar_offsets)


#### End of Part 1 #####

# initialise the map with white pixels

# Create a grid to store map information
# 0: unexplored, 1: free space (white), 2: obstacle (blue), 3: robot path (red)
grid = np.zeros((300, 300), dtype=np.uint8)
# Track pixels that changed to only update those
changed_pixels = set()


# Bresenham's line algorithm for drawing lines
def bresenham_line(x0, y0, x1, y1):
    """Return list of points in a line from (x0,y0) to (x1,y1)"""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points


# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    #####################################################
    #                 Sensing                           #
    #####################################################

    # Read ground sensors
    for beam_index, gs in enumerate(ground_sensors):
        gsr[beam_index] = gs.getValue()

    # Read Lidar
    lidar_sensor_readings = lidar.getRangeImage()  # rhos
    # print("Lidar Sensor Readings: ", lidar_sensor_readings)
    # print("length: ", len(lidar_sensor_readings))
    # print("length of angles: ", len(lidar_offsets))

    ##### Part 2: Turn world coordinates into map coordinates
    #
    # Come up with a way to turn the robot pose (in world coordinates)
    # into coordinates on the map. Draw a red dot using display.drawPixel()
    # where the robot moves.

    ##### Part 3: Convert Lidar data into world coordinates
    #
    # Each Lidar reading has a distance rho and an angle alpha.
    # First compute the corresponding rx and ry of where the lidar
    # hits the object in the robot coordinate system. Then convert
    # rx and ry into world coordinates wx and wy.
    # The arena is 1x1m2 and its origin is in the top left of the arena.

    # Clear changed pixels from previous iteration
    changed_pixels.clear()

    # Draw the robot's position on the grid
    robot_x, robot_y = int(pose_x * 300), int(pose_y * 300)
    if 0 <= robot_x < 300 and 0 <= robot_y < 300:
        if grid[robot_y, robot_x] != 3:  # Only update if changed
            grid[robot_y, robot_x] = 3  # Mark as robot path
            changed_pixels.add((robot_x, robot_y))

    # Filter out infinity readings
    valid_readings = np.array(lidar_sensor_readings) < LIDAR_SENSOR_MAX_RANGE
    valid_distances = np.array(lidar_sensor_readings)[valid_readings]
    valid_angles = lidar_offsets[valid_readings]

    # used claude to speed up the computations and rendering as our implementation was very slow
    if len(valid_distances) > 0:
        # Convert polar to Cartesian coordinates (robot frame) in one operation
        rx = valid_distances * np.cos(valid_angles)
        ry = valid_distances * np.sin(valid_angles)

        # Create transformation matrix once
        trans_mat = np.array(
            [
                [np.sin(pose_theta), np.cos(pose_theta), pose_x],
                [np.cos(pose_theta), -np.sin(pose_theta), pose_y],
                [0, 0, 1],
            ]
        )

        # Apply transformation to all points at once
        robot_coords_h = np.vstack((rx, ry, np.ones_like(rx)))
        world_coords = trans_mat @ robot_coords_h

        # Process every nth lidar reading to reduce computation
        for i in range(0, world_coords.shape[1]):
            wx, wy = world_coords[0, i], world_coords[1, i]
            
            obj_x, obj_y = int(wx * 300), int(wy * 300)

            if 0 <= obj_x < 300 and 0 <= obj_y < 300:
                # Mark obstacle point
                if grid[obj_y, obj_x] != 2 and grid[obj_y, obj_x] != 3:
                    grid[obj_y, obj_x] = 2
                    changed_pixels.add((obj_x, obj_y))

            # Draw free space line
            line_points = bresenham_line(robot_x, robot_y, obj_x, obj_y)

            # Mark points as free space only if they're new
            for px, py in line_points:
                if 0 <= px < 300 and 0 <= py < 300:
                    if grid[py, px] == 0:
                        grid[py, px] = 1
                        changed_pixels.add((px, py))
    
    # Update only the pixels that have changed
    for x, y in changed_pixels:
        if grid[y, x] == 1:  # Free space
            display.setColor(0xFFFFFF)
            display.drawPixel(x, y)
        elif grid[y, x] == 2:  # Obstacle
            display.setColor(0x0000FF)
            display.drawPixel(x, y)
        elif grid[y, x] == 3:  # Robot path
            display.setColor(0xFF0000)
            display.drawPixel(x, y)

    ##### Part 4: Draw the obstacle and free space pixels on the map

    # DO NOT CHANGE THE FOLLOWING CODE (You might only add code to display robot poses)
    #####################################################
    #                 Robot controller                  #
    #####################################################

    if state == "line_follower":
        if gsr[1] < 350 and gsr[0] > 400 and gsr[2] > 400:
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3
        # Checking for Start Line
        elif gsr[0] < 500 and gsr[1] < 500 and gsr[2] < 500:
            vL = MAX_SPEED * 0.3
            vR = MAX_SPEED * 0.3
            # print("Over the line!") # Feel free to uncomment this
            display.imageSave(None, "map.png")
        elif gsr[2] < 650:  # turn right
            vL = 0.2 * MAX_SPEED
            vR = -0.05 * MAX_SPEED
        elif gsr[0] < 650:  # turn left
            vL = -0.05 * MAX_SPEED
            vR = 0.2 * MAX_SPEED

    else:
        # Stationary State
        vL = 0
        vR = 0

    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)

    #####################################################
    #                    Odometry                       #
    #####################################################

    EPUCK_MAX_WHEEL_SPEED = 0.11695 * SIM_TIMESTEP / 1000.0
    dsr = vR / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED
    dsl = vL / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED
    ds = (dsr + dsl) / 2.0

    pose_y += ds * math.cos(pose_theta)
    pose_x += ds * math.sin(pose_theta)
    pose_theta += (dsr - dsl) / EPUCK_AXLE_DIAMETER

    # Feel free to uncomment this for debugging
    # print("X: %f Y: %f Theta: %f " % (pose_x, pose_y, pose_theta))