#!/usr/bin/env python
# encoding: utf-8

import math
import bisect

import tools
import packets
import logger
import pathfinding
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
        if other == None:
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

    MAP_WIDTH  = int(FIELD_WIDTH  / MAP_CELL_RESOLUTION)
    MAP_HEIGHT = int(FIELD_HEIGHT / MAP_CELL_RESOLUTION)
    MAP_WALLS_CELLS = 9 # int(MAP_WALLS_DISTANCE / MAP_CELL_RESOLUTION)
    MAIN_OPPONENT_AVOIDANCE_CELLS = int(MAIN_OPPONENT_AVOIDANCE_RANGE / MAP_CELL_RESOLUTION)
    SECONDARY_OPPONENT_AVOIDANCE_CELLS = int(SECONDARY_OPPONENT_AVOIDANCE_RANGE / MAP_CELL_RESOLUTION)


    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.pathfinder = pathfinding.PathFinder(Map.MAP_HEIGHT,
                                                 Map.MAP_WIDTH,
                                                 ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF,
                                                 Map.MAIN_OPPONENT_AVOIDANCE_CELLS,
                                                 Map.SECONDARY_OPPONENT_AVOIDANCE_CELLS)

        self.walls = []
        self.walls.append([0.5 - MAP_WALLS_DISTANCE, 0.0, 0.52 + MAP_WALLS_DISTANCE, 0.5 + MAP_WALLS_DISTANCE])
        self.walls.append([0.5 - MAP_WALLS_DISTANCE, 2.5 - MAP_WALLS_DISTANCE, 0.52 + MAP_WALLS_DISTANCE, 3.0])
        self.walls.append([1.25 - MAP_WALLS_DISTANCE, 0.0, 2.0, 0.380 + MAP_WALLS_DISTANCE])
        self.walls.append([1.25 - MAP_WALLS_DISTANCE, 2.62 - MAP_WALLS_DISTANCE, 2.0, 3.0])
        self.walls.append([0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE, 1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE])
        for wall in self.walls:
            for i in xrange(len(wall)):
                wall[i] = int(wall[i] / MAP_CELL_RESOLUTION)
            self.pathfinder.add_wall(wall[0], wall[1], wall[2], wall[3])


    def route(self, start_x, start_y, goal_x, goal_y):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorResetRoutePath())

        logger.log("Compute route from ({}, {}) to ({}, {})".format(start_x, start_y, goal_x, goal_y))

        start_cell_x = int(start_x / MAP_CELL_RESOLUTION)
        start_cell_y = int(start_y / MAP_CELL_RESOLUTION)
        goal_cell_x = int(goal_x / MAP_CELL_RESOLUTION)
        goal_cell_y = int(goal_y / MAP_CELL_RESOLUTION)

        path = self.pathfinder.find(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)

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
            if p3 != None:
                simplified_path.append(p3)

        if IS_HOST_DEVICE_PC:
            self.send_route_to_simulator(path, False)
            self.send_route_to_simulator(simplified_path, True)

        return simplified_path


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
