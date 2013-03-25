# encoding: utf-8

import sys
import os
import subprocess
import math

import tools
import packets
import logger
import eventloop
import binarizer
import builder
from definitions import *
from position import Pose, VirtualPose




class Map(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.blocked_zones = []

        self.build_module()

        self.simulator_circle_shapes = []
        self.simulator_rect_shapes = []

        (self.pathfinder, routing_zone_cost)  = self.create_pathfinder(ROUTING_MAP_RESOLUTION)
        (self.evaluator, evaluator_zone_cost) = self.create_pathfinder(EVALUATOR_MAP_RESOLUTION)

        self.zone_cost = max(routing_zone_cost, evaluator_zone_cost)
        lateral_distance = ROBOT_X_SIZE - ROBOT_CENTER_X - max(ROUTING_MAP_RESOLUTION, EVALUATOR_MAP_RESOLUTION)
        captain_room_reduction = 0.05

        only_rects = False

        if only_rects:
            # Captain room
            self.add_rect(0.5 - MAP_WALLS_DISTANCE + captain_room_reduction, 0.0, 0.52 + MAP_WALLS_DISTANCE - captain_room_reduction, 0.4 + MAP_WALLS_DISTANCE, self.zone_cost * 2.0, True, True)
            # Bridge
            self.add_rect(1.25 - MAP_WALLS_DISTANCE, 0.0, 2.0, 0.380 + MAP_WALLS_DISTANCE, self.zone_cost * 2.0, True, True)
            # Island
            self.add_rect(0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE, 1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE, self.zone_cost * 2.0, True, True)
            # Edges
            self.add_rect(0.0, 0.0, MAP_WALLS_DISTANCE, FIELD_Y_SIZE / 2.0, self.zone_cost * 2.0, True, True) # Top edge
            self.add_rect(FIELD_X_SIZE - MAP_WALLS_DISTANCE, 0.0, FIELD_X_SIZE, FIELD_Y_SIZE / 2.0, self.zone_cost * 2.0, True, True) # Bottom edge
            self.add_rect(0.0, 0.0, FIELD_X_SIZE, lateral_distance, self.zone_cost * 2.0, True, True) # Bridge edge
        else:
            self.add_circle(0.0, 1.5, 0.5 + ROBOT_Y_SIZE / 2.0, self.zone_cost * 2.0, True, False)
            # Edges
            self.add_rect(0.0, 0.0, MAP_WALLS_DISTANCE, FIELD_Y_SIZE / 2.0, self.zone_cost * 2.0, True, True) # Top edge
            self.add_rect(FIELD_X_SIZE - MAP_WALLS_DISTANCE, 0.0, FIELD_X_SIZE, FIELD_Y_SIZE / 2.0, self.zone_cost * 2.0, True, True) # Bottom edge
            self.add_rect(0.0, 0.0, FIELD_X_SIZE, MAP_WALLS_DISTANCE, self.zone_cost * 2.0, True, True) # Bridge edge


    def create_pathfinder(self, map_resolution):
        import pathfinding

        map_x_size = int(math.ceil(FIELD_X_SIZE / map_resolution))
        map_y_size = int(math.ceil(FIELD_Y_SIZE / map_resolution))
        main_opponent_avoidance_cells      = int(round(MAIN_OPPONENT_AVOIDANCE_RANGE      / map_resolution))
        secondary_opponent_avoidance_cells = int(round(SECONDARY_OPPONENT_AVOIDANCE_RANGE / map_resolution))

        zone_cost = float(map_x_size * map_y_size)
        main_opponent_collision_cost = zone_cost
        secondary_opponent_collision_cost = main_opponent_collision_cost

        pathfinder = pathfinding.PathFinder(map_x_size,
                                            map_y_size,
                                            ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF,
                                            main_opponent_avoidance_cells,
                                            secondary_opponent_avoidance_cells,
                                            main_opponent_collision_cost,
                                            secondary_opponent_collision_cost)

        return (pathfinder, zone_cost)


    def process_rect(self, x1, y1, x2, y2, cost, is_forbidden):
        for (pathfinder, map_resolution, send_to_simulator) in [(self.pathfinder, ROUTING_MAP_RESOLUTION, IS_HOST_DEVICE_PC), (self.evaluator, EVALUATOR_MAP_RESOLUTION, False)]:
            px1 = int(round(x1 / map_resolution))
            px2 = int(round(x2 / map_resolution))
            py1 = int(round(y1 / map_resolution))
            py2 = int(round(y2 / map_resolution))
            if is_forbidden:
                pathfinder.add_forbidden_rect(px1, py1, px2, py2, cost)
            else:
                pathfinder.add_penalized_rect(px1, py1, px2, py2, cost)
            if send_to_simulator:
                self.simulator_rect_shapes.append((px1, py1, px2, py2, is_forbidden))


    def process_circle(self, x, y, radius, cost, is_forbidden):
        for (pathfinder, map_resolution, send_to_simulator) in [(self.pathfinder, ROUTING_MAP_RESOLUTION, IS_HOST_DEVICE_PC), (self.evaluator, EVALUATOR_MAP_RESOLUTION, False)]:
            px = int(round(x / map_resolution))
            py = int(round(y / map_resolution))
            pradius = int(round(radius / map_resolution))
            if is_forbidden:
                pathfinder.add_forbidden_circle(px, py, pradius, cost)
            else:
                pathfinder.add_penalized_circle(px, py, pradius, cost)
            if send_to_simulator:
                self.simulator_circle_shapes.append((px, py, pradius, is_forbidden))


    def add_rect(self, x1, y1, x2, y2, cost, is_forbidden, is_symetric):
        self.process_rect(x1, y1, x2, y2, cost, is_forbidden)
        if is_symetric:
            self.process_rect(x1, FIELD_Y_SIZE - y2, x2, FIELD_Y_SIZE - y1, cost, is_forbidden)


    def add_circle(self, x, y, radius, cost, is_forbidden, is_symetric):
        self.process_circle(x, y, radius, cost, is_forbidden)
        if is_symetric:
            self.process_circle(x, FIELD_Y_SIZE - y, radius, cost, is_forbidden)


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

        if len(path) == 0:
            return []
        elif len(path) < 3:
            simplified_paths.append([goal_cell_y, goal_cell_x])
        else:
            # 1. Filter X or Y lines
            lines_filtered_path = [ Pose(path[-1][0], path[-1][1]) ]
            for i in reversed(range(len(path) - 1)):
                (next_x, next_y) = path[i + 1]
                (x, y) = path[i]
                if x != next_x and y != next_y:
                    lines_filtered_path.insert(0, Pose(x, y))

            # 2. Filter aligned points
            p1 = lines_filtered_path[0]
            p2 = None
            if len(lines_filtered_path) > 1:
                p3 = lines_filtered_path[1]
            else:
                p3 = None
            alignment_filtered_path = []
            if p1.x != start_cell_x or p1.y != start_cell_y:
                alignment_filtered_path.append(p1)
            for i in range(2, len(lines_filtered_path)):
                p2 = p3
                p3 = lines_filtered_path[i]
                a1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
                a2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
                if abs(a1 - a2) > 0.1:
                    alignment_filtered_path.append(p2)
                    p1 = p2
            if p3 is not None:
                alignment_filtered_path.append(p3)

            # 3. Filter neighbor points
            neighbor_filtered_path = [ alignment_filtered_path[0] ]
            for i in range(1, len(alignment_filtered_path)):
                prev = neighbor_filtered_path[-1]
                curr = alignment_filtered_path[i]
                if abs(prev.x - curr.x) > 1 or abs(prev.y - curr.y) > 1:
                    neighbor_filtered_path.append(curr)

            # 4. Split the path if the angle is too sharp
            p1 = neighbor_filtered_path[0]
            p2 = None
            if len(neighbor_filtered_path) > 1:
                p3 = neighbor_filtered_path[1]
            else:
                p3 = None
            simplified_paths.append([ p1 ])
            for i in range(2, len(neighbor_filtered_path)):
                p2 = p3
                p3 = neighbor_filtered_path[i]
                a1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
                a2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
                simplified_paths[-1].append(p2)
                if abs(a1 - a2) > ROUTE_SPLIT_ANGLE:
                    simplified_paths.append([])
            if p3 is not None:
                simplified_paths[-1].append(p3)

        if IS_HOST_DEVICE_PC:
            self.send_route_to_simulator(path, False)
            for simplified_path in simplified_paths:
                self.send_route_to_simulator(simplified_path, True)

        for i in range(len(simplified_paths)):
            simplified_path = simplified_paths[i]
            if len(simplified_path) == 0:
                del simplified_paths[i]
            else:
                for j in range(len(simplified_path)):
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
            max_cost**=2
            return max_cost
        else:
            return pathfinding_result[0]


    def on_device_ready(self, packet):
        x1 = 0.70
        y1 = 2.20
        x2 = 2.0
        y2 = 3.0
        if packet.team == TEAM_RED:
            tmp = y2
            y2 = FIELD_Y_SIZE - y1
            y1 = FIELD_Y_SIZE - tmp
        # Secondary robot rect
        #self.add_rect(x1, y1, x2, y2, self.zone_cost, False, False)
        self.send_zones_to_simulator()


    def on_goto_finished(self, packet):
        return # Disable this feature for the cup
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
            rz = self.pathfinder.add_penalized_rect(int(x1 / ROUTING_MAP_RESOLUTION),
                                                    int(y1 / ROUTING_MAP_RESOLUTION),
                                                    int(x2 / ROUTING_MAP_RESOLUTION),
                                                    int(y2 / ROUTING_MAP_RESOLUTION),
                                                    cost / ROUTING_MAP_RESOLUTION)
            ez = self.evaluator.add_penalized_rect(int(x1 / EVALUATOR_MAP_RESOLUTION),
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


    def opponent_detected(self, packet, x, y):
        if packet.robot == OPPONENT_ROBOT_MAIN:
            if packet.distance == 0:
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
        max_elements = packets.SimulatorRoutePath.DEFINITION[0][1].max_count
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
        if IS_HOST_DEVICE_PC:
            forbidden_rects_packet = packets.SimulatorRouteRects(is_forbidden_zone = True)
            forbidden_circles_packet = packets.SimulatorRouteCircles(is_forbidden_zone = True)
            penalized_rects_packet = packets.SimulatorRouteRects(is_forbidden_zone = False)
            penalized_circles_packet = packets.SimulatorRouteCircles(is_forbidden_zone = False)

            for rect in self.simulator_rect_shapes:
                s = binarizer.StructInstance(x1 = rect[0], y1 = rect[1], x2 = rect[2], y2 = rect[3])
                if rect[-1]:
                    forbidden_rects_packet.shapes.append(s)
                else:
                    penalized_rects_packet.shapes.append(s)
            self.simulator_rect_shapes = []
            for circle in self.simulator_circle_shapes:
                s = binarizer.StructInstance(x = circle[0], y = circle[1], radius = circle[2])
                if circle[-1]:
                    forbidden_circles_packet.shapes.append(s)
                else:
                    penalized_circles_packet.shapes.append(s)
            self.simulator_circle_shapes = []

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
        pyversion = "python{}.{}{}".format(sys.version_info.major, sys.version_info.minor, sys.abiflags)
        include_dir = sys.exec_prefix + "/include/" + pyversion
        lib_dir = sys.exec_prefix + "/lib"

        working_dir = os.path.dirname(__file__)
        source_file = "pathfinding.c"
        output_file = "pathfinding.so"
        exe = "gcc"

        if sys.platform == "darwin" :
            lib_dir = "/usr/local/Cellar/python3/3.3.0/Frameworks/Python.framework/Versions/3.3/lib"

        params = ["-O2", "-shared", "-fPIC", "-o", output_file, "-I" + include_dir, "-L" + lib_dir,
                    source_file, "-l" + pyversion]

        if sys.platform == "darwin" :
            params = ["-dynamiclib"] + params

        commands = [exe] + params

        bld = builder.Builder(source_file, output_file, commands, working_dir)
        bld.build()
