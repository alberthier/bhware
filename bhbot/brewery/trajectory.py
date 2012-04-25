#!/usr/bin/env python
# encoding: utf-8

import sys
import os
import subprocess
import math

import tools
import packets
import logger
import eventloop
from definitions import *




class Pose(object):

    match_team = TEAM_UNKNOWN

    def __init__(self, x = 0.0, y = 0.0, angle = None, virtual=False):
        self.virt = VirtualPose(self)
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        if not virtual :
            self.x = x
            self.y = y
            self.angle = angle
        else :
            self.virt.x = x
            self.virt.y = y
            self.virt.angle = angle


    def look_at(self, pose):
        return math.atan2(pose.y - self.y, pose.x - self.x)


    def __repr__(self):
        return "Pose({}, {}, {})".format(self.x, self.y, self.angle)


    def __eq__(self, other):
        if other is None:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle




class VirtualPose(object):

    def __init__(self, virt_pose):

        """
        Initialize the VirtualPose with a real Pose

        @type virt_pose: Pose
        """
        self.real_pose = virt_pose


    def set_x(self, x):
        self.real_pose.x = x


    def get_x(self):
        return self.real_pose.x


    def set_y(self, y):
        if Pose.match_team == TEAM_RED :
            self.real_pose.y = FIELD_Y_SIZE - y
        else :
            self.real_pose.y = y

    def get_y(self):
        if Pose.match_team == TEAM_RED :
            return FIELD_Y_SIZE - self.real_pose.y
        else :
            return self.real_pose.y


    def set_angle(self, angle):
        if angle :
            angle = tools.normalize_angle(angle)
        if angle and Pose.match_team == TEAM_RED :
            self.real_pose.angle = -angle
        else :
            self.real_pose.angle = angle


    def get_angle(self):
        if self.real_pose.angle and Pose.match_team == TEAM_RED :
            return -self.real_pose.angle
        else :
            return self.real_pose.angle


    x = property(get_x, set_x)
    y = property(get_y, set_y)
    angle = property(get_angle, set_angle)




class Map(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.blocked_zones = []

        self.build_module()

        (self.pathfinder, self.forbidden_zones, self.penalized_zones) = self.create_pathfinder(ROUTING_MAP_RESOLUTION)
        (self.evaluator, forbidden_zones, penalized_zones) = self.create_pathfinder(EVALUATOR_MAP_RESOLUTION)


    def create_pathfinder(self, map_resolution):
        import pathfinding

        map_x_size = int(math.ceil(FIELD_X_SIZE / map_resolution))
        map_y_size = int(math.ceil(FIELD_Y_SIZE / map_resolution))
        main_opponent_avoidance_cells      = int(round(MAIN_OPPONENT_AVOIDANCE_RANGE      / map_resolution))
        secondary_opponent_avoidance_cells = int(round(SECONDARY_OPPONENT_AVOIDANCE_RANGE / map_resolution))

        wall_cost = float(map_x_size * map_y_size)
        main_opponent_collision_cost = wall_cost
        secondary_opponent_collision_cost = main_opponent_collision_cost

        pathfinder = pathfinding.PathFinder(map_x_size,
                                            map_y_size,
                                            ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF,
                                            main_opponent_avoidance_cells,
                                            secondary_opponent_avoidance_cells,
                                            main_opponent_collision_cost,
                                            secondary_opponent_collision_cost)

        lateral_distance = ROBOT_X_SIZE - ROBOT_CENTER_X - map_resolution
        captain_room_reduction = 0.01

        forbidden_zones_def = []
        # Captain room
        forbidden_zones_def.append(['rect', 0.5 - MAP_WALLS_DISTANCE + captain_room_reduction, 0.0, 0.52 + MAP_WALLS_DISTANCE - captain_room_reduction, 0.4])
        forbidden_zones_def.append(['circle', 0.51, 0.4, MAP_WALLS_DISTANCE])
        # Bridge
        forbidden_zones_def.append(['rect', 1.25 - MAP_WALLS_DISTANCE, 0.0, 2.0, 0.380 + MAP_WALLS_DISTANCE])
        # Island
        forbidden_zones_def.append(['rect', 0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE, 1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE])
        # Edges
        forbidden_zones_def.append(['rect', 0.0, 0.0, MAP_WALLS_DISTANCE, FIELD_Y_SIZE / 2.0]) # Top edge
        forbidden_zones_def.append(['rect', FIELD_X_SIZE - MAP_WALLS_DISTANCE, 0.0, FIELD_X_SIZE, FIELD_Y_SIZE / 2.0]) # Bottom edge
        forbidden_zones_def.append(['rect', 0.0, 0.0, FIELD_X_SIZE, lateral_distance]) # Bridge edge

        forbidden_zones = []
        for zone in forbidden_zones_def:
            if zone[0] == 'rect':
                purple_shape = [ 'rect',
                                int(round(zone[1] / map_resolution)),
                                int(round(zone[2] / map_resolution)),
                                int(round(zone[3] / map_resolution)),
                                int(round(zone[4] / map_resolution)) ]
                red_shape = [ 'rect',
                             int(round(zone[1] / map_resolution)),
                             int(round((FIELD_Y_SIZE - zone[4]) / map_resolution)),
                             int(round(zone[3] / map_resolution)),
                             int(round((FIELD_Y_SIZE - zone[2]) / map_resolution)) ]

                pathfinder.add_forbidden_rect(purple_shape[1], purple_shape[2], purple_shape[3], purple_shape[4], wall_cost * 2.0)
                pathfinder.add_forbidden_rect(red_shape[1], red_shape[2], red_shape[3], red_shape[4], wall_cost * 2.0)
            else:
                purple_shape = [ 'circle',
                                 int(round(zone[1] / map_resolution)),
                                 int(round(zone[2] / map_resolution)),
                                 int(round(zone[3] / map_resolution)) ]
                red_shape = [ 'circle',
                             int(round(zone[1] / map_resolution)),
                             int(round((FIELD_Y_SIZE - zone[2]) / map_resolution)),
                             int(round(zone[3] / map_resolution)) ]

                pathfinder.add_forbidden_circle(purple_shape[1], purple_shape[2], purple_shape[3], wall_cost * 2.0)
                pathfinder.add_forbidden_circle(red_shape[1], red_shape[2], red_shape[3], wall_cost * 2.0)
            forbidden_zones.append(purple_shape)
            forbidden_zones.append(red_shape)

        penalized_zones = []

        return (pathfinder, forbidden_zones, penalized_zones)


    def route(self, start, goal):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorResetRoutePath())

        logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, goal.x, goal.y))

        start_cell_x = int(round(start.x / ROUTING_MAP_RESOLUTION))
        start_cell_y = int(round(start.y / ROUTING_MAP_RESOLUTION))
        goal_cell_x = int(round(goal.x / ROUTING_MAP_RESOLUTION))
        goal_cell_y = int(round(goal.y / ROUTING_MAP_RESOLUTION))

        pathfinding_result = self.pathfinder.find(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)

        if pathfinding_result is None:
            logger.log("No route found")
            return []

        path = pathfinding_result[1]

        logger.log("Found route: length={} cost={}".format(len(path), pathfinding_result[0]))

        simplified_paths = []

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
            if len(cleaned_path) >= 2:
                p3 = cleaned_path[1]
            else:
                p3 = None
            simplified_paths.append([])
            for i in xrange(2, len(cleaned_path)):
                p2 = p3
                p3 = cleaned_path[i]
                a1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
                a2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
                if abs(a1 - a2) > 0.1:
                    simplified_paths[-1].append(p2)
                    if abs(a1 - a2) > ROUTE_SPLIT_ANGLE:
                        simplified_paths.append([])
                    p1 = p2
            if p3 is not None:
                simplified_paths[-1].append(p3)

        if IS_HOST_DEVICE_PC:
            self.send_route_to_simulator(path, False)
            for simplified_path in simplified_paths:
                self.send_route_to_simulator(simplified_path, True)

        for i in xrange(len(simplified_paths)):
            simplified_path = simplified_paths[i]
            if len(simplified_path) == 0:
                del simplified_paths[i]
            else:
                for j in xrange(len(simplified_path)):
                    simplified_path[j].x = simplified_path[j].x * ROUTING_MAP_RESOLUTION + ROUTING_MAP_RESOLUTION / 2.0
                    simplified_path[j].y = simplified_path[j].y * ROUTING_MAP_RESOLUTION + ROUTING_MAP_RESOLUTION / 2.0
        if len(simplified_paths) > 0 and len(simplified_paths[-1]) > 0:
            simplified_paths[-1][-1].x = goal.x
            simplified_paths[-1][-1].y = goal.y

        return simplified_paths


    def evaluate(self, start, goal):
        start_cell_x = int(round(start.x / EVALUATOR_MAP_RESOLUTION))
        start_cell_y = int(round(start.y / EVALUATOR_MAP_RESOLUTION))
        goal_cell_x = int(round(goal.x / EVALUATOR_MAP_RESOLUTION))
        goal_cell_y = int(round(goal.y / EVALUATOR_MAP_RESOLUTION))

        pathfinding_result = self.evaluator.find(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)
        if pathfinding_result is None:
            max_cost = (FIELD_X_SIZE / EVALUATOR_MAP_RESOLUTION) * (FIELD_Y_SIZE / EVALUATOR_MAP_RESOLUTION)
            max_cost = max_cost ** 2
            return max_cost
        else:
            return pathfinding_result[0]


    def on_goto_finished(self, packet):
        angle = None
        if packet.reason == REASON_BLOCKED_FRONT:
            angle = self.eventloop.robot.pose.angle
        elif packet.reason == REASON_BLOCKED_BACK:
            angle = self.eventloop.robot.pose.angle + math.pi

        if angle is not None:
            distance = ROBOT_X_SIZE - ROBOT_CENTER_X
            x1 = math.cos(angle) * distance + self.eventloop.robot.pose.x - BLOCKED_ZONE_SIZE / 2.0
            y1 = math.sin(angle) * distance + self.eventloop.robot.pose.y - BLOCKED_ZONE_SIZE / 2.0
            x2 = x1 + BLOCKED_ZONE_SIZE
            y2 = y1 + BLOCKED_ZONE_SIZE
            cost = FIELD_X_SIZE * FIELD_Y_SIZE
            rz = self.pathfinder.add_penalized_zone(int(x1 / ROUTING_MAP_RESOLUTION),
                                                    int(y1 / ROUTING_MAP_RESOLUTION),
                                                    int(x2 / ROUTING_MAP_RESOLUTION),
                                                    int(y2 / ROUTING_MAP_RESOLUTION),
                                                    cost / ROUTING_MAP_RESOLUTION)
            ez = self.evaluator.add_penalized_zone(int(x1 / EVALUATOR_MAP_RESOLUTION),
                                                   int(y1 / EVALUATOR_MAP_RESOLUTION),
                                                   int(x2 / EVALUATOR_MAP_RESOLUTION),
                                                   int(y2 / EVALUATOR_MAP_RESOLUTION),
                                                   cost / EVALUATOR_MAP_RESOLUTION)
            self.blocked_zones.append((rz, ez))
            timer = eventloop.Timer(self.eventloop, BLOCKED_ZONE_DISAPEARING_MS, self.pop_blocked_zone)
            timer.start()


    def pop_blocked_zone(self):
        if len(self.blocked_zones) > 0:
            (rz, ez) = self.blocked_zones[0]
            del self.blocked_zones[0]
            self.pathfinder.remove_penalized_zone(rz)
            self.evaluator.remove_penalized_zone(ez)


    def opponent_detected(self, opponent, x, y):
        if opponent == OPPONENT_ROBOT_MAIN:
            self.pathfinder.set_main_opponent_position(int(round(x / ROUTING_MAP_RESOLUTION)), int(round(y / ROUTING_MAP_RESOLUTION)))
            self.evaluator.set_main_opponent_position(int(round(x / EVALUATOR_MAP_RESOLUTION)), int(round(y / EVALUATOR_MAP_RESOLUTION)))
        else:
            self.pathfinder.set_secondary_opponent_position(int(round(x / ROUTING_MAP_RESOLUTION)), int(round(y / ROUTING_MAP_RESOLUTION)))
            self.evaluator.set_secondary_opponent_position(int(round(x / EVALUATOR_MAP_RESOLUTION)), int(round(y / EVALUATOR_MAP_RESOLUTION)))


    def opponent_disapeared(self, opponent):
        if opponent == OPPONENT_ROBOT_MAIN:
            self.pathfinder.clear_main_opponent_position()
            self.evaluator.clear_main_opponent_position()
        else:
            self.pathfinder.clear_secondary_opponent_position()
            self.evaluator.clear_secondary_opponent_position()


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


    def send_zones_to_simulator(self):
        forbidden_rects_packet = packets.SimulatorRouteRects()
        forbidden_circles_packet = packets.SimulatorRouteCircles()
        penalized_rects_packet = packets.SimulatorRouteRects()
        penalized_circles_packet = packets.SimulatorRouteCircles()

        for zone in self.forbidden_zones:
            if zone[0] == 'rect':
                forbidden_rects_packet.shapes.append(zone[1:])
            else:
                forbidden_circles_packet.shapes.append(zone[1:])
        for zone in self.penalized_zones:
            if zone[0] == 'rect':
                penalized_rects_packet.shapes.append(zone[1:])
            else:
                penalized_circles_packet.shapes.append(zone[1:])

        self.eventloop.send_packet(packets.SimulatorRouteResetZones())
        if len(forbidden_rects_packet.shapes) != 0:
            self.eventloop.send_packet(forbidden_rects_packet)
        if len(forbidden_circles_packet.shapes) != 0:
            self.eventloop.send_packet(forbidden_circles_packet)
        if len(penalized_rects_packet.shapes) != 0:
            self.eventloop.send_packet(penalized_rects_packet)
        if len(penalized_circles_packet.shapes) != 0:
            self.eventloop.send_packet(penalized_circles_packet)


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
