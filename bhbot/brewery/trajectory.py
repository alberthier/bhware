#!/usr/bin/env python
# encoding: utf-8

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
        self.key = (self.x << 16) + self.y


    def __lt__(self, other):
        return self.f_score < other.f_score




class Map(object):

    MAP_WIDTH  = int(FIELD_WIDTH  / MAP_CELL_RESOLUTION)
    MAP_HEIGHT = int(FIELD_HEIGHT / MAP_CELL_RESOLUTION)
    DISTANCE_MAP = None

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.walls = []
        self.opponents = []
        if Map.DISTANCE_MAP == None:
            Map.DISTANCE_MAP = []
            for x in xrange(Map.MAP_HEIGHT + 1):
                row = []
                x2 = x ** 2
                for y in xrange(Map.MAP_WIDTH + 1):
                    row.append(math.sqrt(x2 + y ** 2))
                Map.DISTANCE_MAP.append(row)


    def heuristic_cost_estimate(self, x1, y1, x2, y2):
        return Map.DISTANCE_MAP[abs(x1 - x2)][abs(y1 - y2)] * ASTAR_HEURISTIC_VS_EFFECTIVE_TRADEOFF


    def effective_cost(self, x1, y1, x2, y2):
        return Map.DISTANCE_MAP[abs(x1 - x2)][abs(y1 - y2)]


    def route(self, start_x, start_y, goal_x, goal_y):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorResetRoutePath())

        logger.log("Compute route from ({}, {}) to ({}, {})".format(start_x, start_y, goal_x, goal_y))

        closedset_keys = set()
        openset = []
        openset_keys = set()
        max_openset_size = 1


        start_cell_x = int(start_x / MAP_CELL_RESOLUTION)
        start_cell_y = int(start_y / MAP_CELL_RESOLUTION)
        goal_cell_x = int(goal_x / MAP_CELL_RESOLUTION)
        goal_cell_y = int(goal_y / MAP_CELL_RESOLUTION)

        start = MapCell(start_cell_x, start_cell_y)
        start.g_score = 0
        start.h_score = self.heuristic_cost_estimate(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)
        start.f_score = start.g_score + start.f_score

        openset.append(start)
        openset_keys.add(start.key)

        while len(openset) != 0:
            current = openset[0]
            if current.x == goal_cell_x and current.y == goal_cell_y:
                path = []
                p = current
                while p.came_from != None:
                    path.append(Pose(p.x, p.y))
                    p = p.came_from
                if IS_HOST_DEVICE_PC:
                    self.send_route_to_simulator(path)
                logger.log("Route computed. Length: {}. Max openset size: {}".format(len(path), max_openset_size))
                return path

            del openset[0]
            openset_keys.remove(current.key)
            closedset_keys.add(current.key)

            for neighbor in self.neighbor_nodes(current):
                if not neighbor.key in closedset_keys:
                    tentative_g_score = current.g_score + self.effective_cost(current.x, current.y, neighbor.x, neighbor.y)
                    if not neighbor.key in openset_keys:
                        neighbor.h_score = self.heuristic_cost_estimate(neighbor.x, neighbor.y, goal_cell_x, goal_cell_y)
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current
                        bisect.insort_left(openset, neighbor)
                        openset_keys.add(neighbor.key)
                        if (len(openset) > max_openset_size):
                            max_openset_size = len(openset)
                    elif tentative_g_score < neighbor.g_score:
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current

        self.send_route_to_simulator([])
        return []


    def neighbor_nodes(self, node):
        n = []
        prev_x = node.x - 1
        next_x = node.x + 1
        prev_y = node.y - 1
        next_y = node.y + 1

        if prev_x >= 0:
            if prev_y >= 0:
                n.append(MapCell(prev_x, prev_y))
            n.append(MapCell(prev_x, node.y))
            if next_y < Map.MAP_WIDTH:
                n.append(MapCell(prev_x, next_y))
        if prev_y >= 0:
            n.append(MapCell(node.x, prev_y))
        if next_y < Map.MAP_WIDTH:
            n.append(MapCell(node.x, next_y))
        if next_x < Map.MAP_HEIGHT:
            if prev_y >= 0:
                n.append(MapCell(next_x, prev_y))
            n.append(MapCell(next_x, node.y))
            if next_y < Map.MAP_WIDTH:
                n.append(MapCell(next_x, next_y))

        return n


    def send_route_to_simulator(self, path):
        max_elements = packets.SimulatorRoutePath.DEFINITION[0].max_elements
        packet = packets.SimulatorRoutePath()
        for cell in path:
            if len(packet.points) == max_elements:
                self.eventloop.send_packet(packet)
                packet.points = []
            packet.points.append((cell.x, cell.y))
        if len(packet.points) != 0:
            self.eventloop.send_packet(packet)
