#!/usr/bin/env python
# encoding: utf-8

import math
import bisect

import tools
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

    MAP_WIDTH  = int(FIELD_WIDTH  / ROUTER_MAP_RESOLUTION)
    MAP_HEIGHT = int(FIELD_HEIGHT / ROUTER_MAP_RESOLUTION)

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

    def __init__(self):
        walls = []


    def heuristic_cost_estimate(self, x1, y1, x2, y2):
        return tools.manathan_distance(x1, y1, x2, y2)



    def route(start_x, start_y, goal_x, goal_y)
        closedset = set()
        openset = [ start ]
        openset_keys = set()

        start = MapCell(start_x, start_y)
        start.g_score = 0
        start.h_score = self.heuristic_cost_estimate(start_x, start_y, goal_x, goal_y)
        start.f_score = start.g_score + start.f_score

        while len(openset) != 0:
            current = openset[0]
            if current.x == goal_x and current.y == goal_y:
                path = []
                p = current
                while p.came_from != None:
                    path.append(Pose(p.x, p.y))
                    p = p.came_from
                return path

            del openset[0]
            closedset.add(current)

            for neighbor in self.neighbor_nodes(current):
                if not neighbor in closedset:
                    tentative_g_score = current.g_score + tools.manathan_distance(current.x, current.y, neighbor.x, neighbor.y)
                    if not neighbor.key in openset_keys:
                        neighbor.h_score = self.heuristic_cost_estimate(neighbor.x, neighbor.y, goal_x, goal_y)
                        tentative_is_better = True
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current
                        bisect.insort_left(openset, neighbor)
                        openset_keys.add(neighbor.key)
                    elif tentative_g_score < neighbor.g_score:
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current

        return []


    def node_neighbors(self, node):
        n = []
        prev_x = node.x - 1
        next_x = node.x + 1
        prev_y = node.y - 1
        next_y = node.y + 1

        if prev_x >= 0:
            if prev_y >= 0:
                n.append(MapCell(prev_x, prev_y)
            n.append(MapCell(prev_x, node.y))
            if next_y < MAP_WIDTH:
                n.append(MapCell(prev_x, next_y))
        if prev_y >= 0:
            n.append(MapCell(node.x, prev_y)
        n.append(MapCell(node.x, self.y))
        if next_y < MAP_WIDTH:
            n.append(MapCell(node.x, next_y))
        if next_x < MAP_HEIGHT:
            if prev_y >= 0:
                n.append(MapCell(next_x, prev_y)
            n.append(MapCell(next_x, node.y))
            if next_y < MAP_WIDTH:
                n.append(MapCell(next_x, next_y))

    return n
 
