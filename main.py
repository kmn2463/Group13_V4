#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import copy

# Every half-foot has a node.
workspaceRows = 20
workspaceCols = 32
node_distance = 152.4  # mm

obstacle = [
    (0.61, 2.743), (0.915, 2.743), (1.219, 2.743),
    (1.829, 1.219), (1.829, 1.524), (1.829, 1.829),
    (1.829, 2.134), (2.743, 0.305), (2.743, 0.61),
    (2.743, 0.915), (2.743, 2.743), (3.048, 2.743),
    (3.353, 2.743), (-1, -1), (-1, -1),
    (-1, -1), (-1, -1), (-1, -1),
    (-1, -1), (-1, -1), (-1, -1),
    (-1, -1), (-1, -1), (-1, -1),
    (-1, -1)]
start = (0.305, 1.219)
goal = (3.658, 1.829)

# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
ev3.speaker.beep()

# Because the diameter of rubber tires is around 56mm, the circumference is 56 * pi.
pi = 3.14159265359
ev3_wheel_diameter = 56  # mm
ev3_wheel_circumference = ev3_wheel_diameter*pi


def move_fw(x):
    left_motor.run_angle(200, x*360/ev3_wheel_circumference, Stop.HOLD, False)
    right_motor.run_angle(200, x*360/ev3_wheel_circumference, Stop.HOLD, True)


def turn_left():
    left_motor.run_angle(100, -179.68, Stop.HOLD, False)
    right_motor.run_angle(100, 179.68, Stop.HOLD, True)


def turn_right():
    left_motor.run_angle(100, 179.68, Stop.HOLD, False)
    right_motor.run_angle(100, -179.68, Stop.HOLD, True)


def move_forward_one_node():
    left_motor.run_angle(200, node_distance*360 /
                         ev3_wheel_circumference, Stop.HOLD, False)
    right_motor.run_angle(200, node_distance*360 /
                          ev3_wheel_circumference, Stop.HOLD, True)

#######################################


def metersToFeet(length):
    return length*3.28084
# Algorithm for minimum manhattan distance, Dijkstra


class Node:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.manhattan_distance = None
        self.is_start = False
        self.is_goal = False
        self.is_obstacle = False
        self.visited = False
        
    def coordinates(self, x, y):
        self.x = x
        self.y = y

    def mark_start_node(self):
        self.is_start = True
        self.visited = True

    def mark_goal_node(self):
        self.is_goal = True
        self.manhattan_distance = 0

    def mark_visited_node(self):
        self.visited = True

    def mark_obstacle_node(self):
        self.is_obstacle = True
        self.manhattan_distance = -1


workspace_east = 'e'
workspace_north = 'n'
workspace_west = 'w'
workspace_south = 's'


class Workspace:
    def __init__(self):
        self.map = []
        self.start = None
        self.goal = None
        self.goal_in_workspace = None
        self.robot_location = None
        self.robot_direction = workspace_east
        for i in range(workspaceRows):
            empty_row = []
            for j in range(workspaceCols):
                node = Node()
                x = .5 * j
                y = .5 * (workspaceRows - i)
                node.coordinates(x, y)
                empty_row.append(node)
            self.map.append(empty_row)

    def mark_Start_Goal(self):
        self.start = (round(metersToFeet(start[0]), 1), round(
            metersToFeet(start[1]), 1))
        self.goal = (round(metersToFeet(goal[0]), 1), round(
            metersToFeet(goal[1]), 1))
        for i in range(workspaceRows):
            for j in range(workspaceCols):
                node = self.map[i][j]
                if((node.x, node.y) == self.start):
                    node.mark_start_node()
                    # maintain track of our location on the worksite
                    self.robot_location = (i, j)
                elif((node.x, node.y) == self.goal):
                    node.mark_goal_node()
                    # utilized in our algorithm to see if it has been "visited" goal yet
                    self.goal_in_workspace = (i, j)

    def mark_obstacles_center(self):
        # for each obstacle
        for obs in obstacle:
            if obs[0] == -1:
                continue
            obs_x = round(metersToFeet(obs[0]), 1)
            obs_y = round(metersToFeet(obs[1]), 1)
            for i in range(workspaceRows):
                for j in range(workspaceCols):
                    node = self.map[i][j]
                    if(node.x == obs_x and node.y == obs_y):
                        node.mark_obstacle_node()
                        self.obstacles_in_real_size(i, j)

                    elif(node.y == workspaceRows/2):
                        node.mark_obstacle_node()
                    elif(node.x == 0):
                        node.mark_obstacle_node()

    # Because we are using half-foot nodes, the nodes towards the center are obstacles.
    def obstacles_in_real_size(self, i, j):
        if(i+1 != workspaceRows):
            self.map[i+1][j].mark_obstacle_node()
        if(j+1 != workspaceCols):
            self.map[i][j+1].mark_obstacle_node()
        if(i-1 != -1):
            self.map[i-1][j].mark_obstacle_node()
        if(j-1 != -1):
            self.map[i][j-1].mark_obstacle_node()
        if(i+1 != workspaceRows and j+1 != workspaceCols):
            self.map[i+1][j+1].mark_obstacle_node()
        if(i+1 != workspaceRows and j-1 != -1):
            self.map[i+1][j-1].mark_obstacle_node()
        if(i-1 != -1 and j+1 != workspaceCols):
            self.map[i-1][j+1].mark_obstacle_node()
        if(i-1 != -1 and j-1 != -1):
            self.map[i-1][j-1].mark_obstacle_node()
        if(i+1 != workspaceRows and j+2 != workspaceCols):
            self.map[i+1][j+2].mark_obstacle_node()
        if(i-1 != -1 and j-2 >= 0):
            self.map[i-1][j-2].mark_obstacle_node()
        if(i-2 >= 0 and j-1 >= 0):
            self.map[i-2][j-1].mark_obstacle_node()
        if(i-2 >= 0 and j-2 >= 0):
            self.map[i-2][j-2].mark_obstacle_node()
        if(i+2 < workspaceRows and j-2 >= 0):
            self.map[i+2][j-2].mark_obstacle_node()


    def coppy_workspace(self):
        temp = []
        for row in self.map:
            temp_row = []
            for node in row:
                temp_row.append(node)
            temp.append(temp_row)
        return temp
        
    def set_manhattan_distance(self):
        for m in range(workspaceRows * workspaceCols):

            temp = self.coppy_workspace()

            for i in range(workspaceRows):
                for j in range(workspaceCols):
                    if(self.map[i][j].manhattan_distance != None):
                        continue

                    if j+1 != workspaceCols and temp[i][j+1].manhattan_distance != None and temp[i][j+1].manhattan_distance != -1:
                        self.map[i][j].manhattan_distance = temp[i][j +
                                                                    1].manhattan_distance + 1

                    elif j-1 != -1 and temp[i][j-1].manhattan_distance != None and temp[i][j-1].manhattan_distance != -1:
                        self.map[i][j].manhattan_distance = temp[i][j -
                                                                    1].manhattan_distance + 1

                    elif i+1 != workspaceRows and temp[i+1][j].manhattan_distance != None and temp[i+1][j].manhattan_distance != -1:
                        self.map[i][j].manhattan_distance = temp[i +
                                                                 1][j].manhattan_distance + 1

                    elif i-1 != -1 and temp[i-1][j].manhattan_distance != None and temp[i-1][j].manhattan_distance != -1:
                        self.map[i][j].manhattan_distance = temp[i -
                                                                 1][j].manhattan_distance + 1

    def visited_goal(self):
        i, j = self.goal_in_workspace
        return self.map[i][j].visited

    def get_turning_commands(self, desired_dir):
        if(self.robot_direction == desired_dir):
            return []
        elif((self.robot_direction == workspace_east and desired_dir == workspace_south) or
             (self.robot_direction == workspace_south and desired_dir == workspace_west) or
             (self.robot_direction == workspace_west and desired_dir == workspace_north) or
             (self.robot_direction == workspace_north and desired_dir == workspace_east)):
            return ['turn_right']
        elif((self.robot_direction == workspace_east and desired_dir == workspace_north) or
             (self.robot_direction == workspace_south and desired_dir == workspace_east) or
             (self.robot_direction == workspace_west and desired_dir == workspace_south) or
             (self.robot_direction == workspace_north and desired_dir == workspace_west)):
            return ['turn_left']
        else:
            return ['turn_right', 'turn_right']

    def get_neighbor_nodes(self):
        i, j = self.robot_location

        # Each neighbor in the list will be represented by I j, neighbor node.
        neighbors = []

        # If statements ensure that we do not grab something that is out of bounds or that is an obstacle.
        if(j + 1 != workspaceCols and not self.map[i][j+1].is_obstacle):
            neighbors.append((i, j+1, self.map[i][j+1]))

        if(i + 1 != workspaceRows and not self.map[i+1][j].is_obstacle):
            neighbors.append((i+1, j, self.map[i+1][j]))

        if(j - 1 != -1 and not self.map[i][j-1].is_obstacle):
            neighbors.append((i, j-1, self.map[i][j-1]))

        if(i - 1 != -1 and not self.map[i-1][j].is_obstacle):
            neighbors.append((i-1, j, self.map[i-1][j]))

        return neighbors

    def next_node(self):
        neighbor_nodes = self.get_neighbor_nodes()
        # default neighbor to contact
        min_ij = neighbor_nodes[0][0], neighbor_nodes[0][1]
        # default neighbor with the least distance
        min_manhattan_distance = neighbor_nodes[0][2].manhattan_distance

        for neighbor in neighbor_nodes:
            if neighbor[2].manhattan_distance < min_manhattan_distance:
                min_ij = neighbor[0], neighbor[1]
                min_manhattan_distance = neighbor[2].manhattan_distance

        i, j = self.robot_location  # where the robot is right now
        next_direction = workspace_east
        if((i+1, j) == min_ij):
            next_direction = workspace_south
        elif((i, j-1) == min_ij):
            next_direction = workspace_west
        elif((i-1, j) == min_ij):
            next_direction = workspace_north

        turning_commands = self.get_turning_commands(next_direction)
        full_commands = turning_commands + ['move']

        # assign "current" vals to the node to which we are going
        self.robot_location = min_ij
        self.robot_direction = next_direction
        i, j = self.robot_location
        self.map[i][j].visited = True

        return full_commands


def perform_commands(commands):
    for command in commands:
        if(command == "turn_right"):
            turn_right()
        elif(command == "turn_left"):
            turn_left()
        elif(command == "move"):
            move_forward_one_node()


# Build Workspace
myWorkspace = Workspace()
myWorkspace.mark_obstacles_center()
myWorkspace.mark_Start_Goal()
myWorkspace.set_manhattan_distance()

# To indicate that the map has been completed, play a sound.
ev3.speaker.beep()

while(not myWorkspace.visited_goal()):
    commands = myWorkspace.next_node()
    perform_commands(commands)

# When done moving, play another 2 beeps.
ev3.speaker.beep()
ev3.speaker.beep()
