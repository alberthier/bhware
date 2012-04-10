#!/usr/bin/env python
# encoding: utf-8

import sys
import os
import subprocess
import math
import bisect

import tools
import packets
import logger
from definitions import *




class Pose(object):

    def __init__(self, x = 0.0, y = 0.0, angle = None):
        self.x = x
        self.y = y
        self.angle = angle


    def look_at(self, pose):
        return math.atan2(pose.y - self.y, pose.x - self.x)


    def __repr__(self):
        return "Pose({}, {}, {})".format(self.x, self.y, self.angle)


    def __eq__(self, other):
        if other is None:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle




class MapCell(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g_score = 0
        self.h_score = 0
        self.f_score = 0
        self.came_from = None
        self.key = MapCell.make_key(x, y)


    @staticmethod
    def make_key(x, y):
        return (x << 16) + y


    def __lt__(self, other):
        return self.f_score < other.f_score




class Map(object):

    MAP_X_SIZE = int(FIELD_X_SIZE / MAP_CELL_RESOLUTION)
    MAP_Y_SIZE = int(FIELD_Y_SIZE / MAP_CELL_RESOLUTION)
    MAIN_OPPONENT_AVOIDANCE_CELLS = int(MAIN_OPPONENT_AVOIDANCE_RANGE / MAP_CELL_RESOLUTION)
    SECONDARY_OPPONENT_AVOIDANCE_CELLS = int(SECONDARY_OPPONENT_AVOIDANCE_RANGE / MAP_CELL_RESOLUTION)


    def __init__(self, eventloop):
        self.eventloop = eventloop

        self.build_module()

        import pathfinding

        wall_cost = float(Map.MAP_X_SIZE * Map.MAP_Y_SIZE)
        main_opponent_collision_cost = wall_cost
        secondary_opponent_collision_cost = main_opponent_collision_cost

        self.pathfinder = pathfinding.PathFinder(Map.MAP_X_SIZE,
                                                 Map.MAP_Y_SIZE,
                                                 ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF,
                                                 Map.MAIN_OPPONENT_AVOIDANCE_CELLS,
                                                 Map.SECONDARY_OPPONENT_AVOIDANCE_CELLS,
                                                 main_opponent_collision_cost,
                                                 secondary_opponent_collision_cost)

        self.walls = []
        self.walls.append([0.5 - MAP_WALLS_DISTANCE, 0.0, 0.52 + MAP_WALLS_DISTANCE, 0.4 + MAP_WALLS_DISTANCE])
        self.walls.append([0.5 - MAP_WALLS_DISTANCE, 2.6 - MAP_WALLS_DISTANCE, 0.52 + MAP_WALLS_DISTANCE, 3.0])
        self.walls.append([1.25 - MAP_WALLS_DISTANCE, 0.0, 2.0, 0.380 + MAP_WALLS_DISTANCE])
        self.walls.append([1.25 - MAP_WALLS_DISTANCE, 2.62 - MAP_WALLS_DISTANCE, 2.0, 3.0])
        self.walls.append([0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE, 1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE])
        for wall in self.walls:
            for i in xrange(len(wall)):
                wall[i] = int(wall[i] / MAP_CELL_RESOLUTION)
            self.pathfinder.add_penalized_zone(wall[0], wall[1], wall[2], wall[3], wall_cost)


    def route(self, start_x, start_y, goal_x, goal_y, reference_team = TEAM_PURPLE):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorResetRoutePath())

        real_start_y = start_y #self.eventloop.robot.convert_y(start_y, reference_team)
        real_goal_y = self.eventloop.robot.convert_y(goal_y, reference_team)

        logger.log("Compute route from ({}, {}) to ({}, {})".format(start_x, real_start_y, goal_x, real_goal_y))

        start_cell_x = int(start_x / MAP_CELL_RESOLUTION)
        start_cell_y = int(real_start_y / MAP_CELL_RESOLUTION)
        goal_cell_x = int(goal_x / MAP_CELL_RESOLUTION)
        goal_cell_y = int(real_goal_y / MAP_CELL_RESOLUTION)

        path = self.pathfinder.find(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)

        logger.log("Found route length: {}".format(len(path)))

        simplified_path = []

        if len(path) > 1:
            cleaned_path = [ Pose(path[0][0], path[0][1]) ]
            for i in xrange(1, len(path)):
                (prev_x, prev_y) = path[i - 1]
                (x, y) = path[i]
                if x != prev_x and y != prev_y:
                    cleaned_path.append(Pose(x, y))
            if cleaned_path[-1].x != path[-1][0] or cleaned_path[-1].y != path[-1][1]:
                cleaned_path.append(Pose(path[-1][0], path[-1][1]))
            p1 = cleaned_path[0]
            p2 = None
            p3 = None
            simplified_path.append(p1)
            for i in xrange(2, len(cleaned_path)):
                p2 = cleaned_path[i - 1]
                p3 = cleaned_path[i]
                a1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
                a2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
                if abs(a1 - a2) > 0.1:
                   simplified_path.append(p2)
                   p1 = p2
            if p3 is not None:
                simplified_path.append(p3)

        if IS_HOST_DEVICE_PC:
            self.send_route_to_simulator(path, False)
            self.send_route_to_simulator(simplified_path[1:], True)

        if len(simplified_path) > 0:
            for i in xrange(1, len(simplified_path) - 1):
                simplified_path[i].x = simplified_path[i].x * MAP_CELL_RESOLUTION + MAP_CELL_RESOLUTION / 2.0
                y = self.eventloop.robot.convert_y(simplified_path[i].y * MAP_CELL_RESOLUTION + MAP_CELL_RESOLUTION / 2.0, reference_team)
                simplified_path[i].y = y
            simplified_path[-1].x = goal_x
            simplified_path[-1].y = goal_y

        return simplified_path[1:]


    def opponent_detected(self, opponent, x, y):
        if opponent == OPPONENT_ROBOT_MAIN:
            self.pathfinder.set_main_opponent_position(int(x / MAP_CELL_RESOLUTION), int(y / MAP_CELL_RESOLUTION))
        else:
            self.pathfinder.set_secondary_opponent_position(int(x / MAP_CELL_RESOLUTION), int(y / MAP_CELL_RESOLUTION))


    def opponent_disapeared(self, opponent):
        if opponent == OPPONENT_ROBOT_MAIN:
            self.pathfinder.clear_main_opponent_position()
        else:
            self.pathfinder.clear_secondary_opponent_position()


    def send_route_to_simulator(self, path, is_simplified):
        max_elements = packets.SimulatorRoutePath.DEFINITION[0].max_elements
        if is_simplified:
            packet = packets.SimulatorSimplifiedRoutePath()
        else:
            packet = packets.SimulatorRoutePath()
        for cell in path:
            if len(packet.points) == max_elements:
                self.eventloop.send_packet(packet)
                packet.points = []
            if isinstance(cell, Pose):
                packet.points.append((cell.x, cell.y))
            else:
                packet.points.append(cell)
        if len(packet.points) != 0:
            self.eventloop.send_packet(packet)


    def send_walls_to_simulator(self):
        packet = packets.SimulatorRouteWalls()
        for wall in self.walls:
            packet.walls.append(wall)
        self.eventloop.send_packet(packet)


    def build_module(self):
        pyversion = "python{}.{}".format(sys.version_info.major, sys.version_info.minor)
        include_dir = sys.exec_prefix + "/include/" + pyversion
        lib_dir = sys.exec_prefix + "/lib"
        working_dir = os.path.dirname(__file__)
        cmd = ["gcc", "-O2", "-shared", "-fPIC", "-o", "pathfinding.so", "-I" + include_dir, "-L" + lib_dir, "pathfinding.c", "-l" + pyversion]
        gcc = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, cwd=working_dir)
        (out, err) = gcc.communicate()
        if gcc.returncode == 0:
            logger.log("Pathfinding C module compiled successfully")
        for l in out.splitlines():
            logger.log(l)
        for l in err.splitlines():
            logger.log(l)
