#!/usr/bin/env python
# encoding: utf-8

import sys
import os
import subprocess
import math

import tools
import packets
import logger
from definitions import *


class Pose(object):
    match_team = TEAM_UNKNOWN
    def __init__(self, x = 0.0, y = 0.0, angle = None, virtual=False):
#        self.real = RealPose(self)
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

#class RealPose(object):
#    def __init__(self, virt_pose):
#
#        """
#        Initialize the RealPose with a virtual Pose
#
#        @type virt_pose: Pose
#        """
#        self.virt_pose = virt_pose
#
#    def set_x(self, x):
#        self.virt_pose.x = x
#
#    def get_x(self):
#        return self.virt_pose.x
#
#    def set_y(self, y):
#        if Pose.match_team == TEAM_RED :
#            self.virt_pose.y = FIELD_Y_SIZE - y
#        else :
#            self.virt_pose.y = y
#
#    def get_y(self):
#        if Pose.match_team == TEAM_RED :
#            return FIELD_Y_SIZE - self.virt_pose.y
#        else :
#            return self.virt_pose.y
#
#
#    def set_angle(self, angle):
#        angle = tools.normalize_angle(angle)
#        if Pose.match_team == TEAM_RED :
#            self.virt_pose.angle = -angle
#        else :
#            self.virt_pose.y = angle
#
#    def get_angle(self):
#        if Pose.match_team == TEAM_RED :
#            return -self.virt_pose.angle
#        else :
#            return self.virt_pose.angle
#
#
#    x = property(get_x, set_x)
#    y = property(get_y, set_y)
#    angle = property(get_angle, set_angle)

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

        self.build_module()

        (self.pathfinder, self.walls) = self.create_pathfinder(ROUTING_MAP_RESOLUTION)
        (self.evaluator, walls) = self.create_pathfinder(EVALUATOR_MAP_RESOLUTION)


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
        walls = []
        walls.append([0.5 - MAP_WALLS_DISTANCE + captain_room_reduction, 0.0, 0.52 + MAP_WALLS_DISTANCE - captain_room_reduction, 0.4 + MAP_WALLS_DISTANCE]) # Purple captain room
        walls.append([0.5 - MAP_WALLS_DISTANCE + captain_room_reduction, 2.6 - MAP_WALLS_DISTANCE, 0.52 + MAP_WALLS_DISTANCE - captain_room_reduction, 3.0]) # Red captain room
        walls.append([1.25 - MAP_WALLS_DISTANCE, 0.0, 2.0, 0.380 + MAP_WALLS_DISTANCE]) # Purple bridge
        walls.append([1.25 - MAP_WALLS_DISTANCE, 2.62 - MAP_WALLS_DISTANCE, 2.0, 3.0])  # Red bridge
        walls.append([0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE, 1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE]) # Island
        walls.append([0.0, 0.0, MAP_WALLS_DISTANCE, FIELD_Y_SIZE]) # Top edge
        walls.append([FIELD_X_SIZE - MAP_WALLS_DISTANCE, 0.0, FIELD_X_SIZE, FIELD_Y_SIZE]) # Bottom edge
        walls.append([0.0, 0.0, FIELD_X_SIZE, lateral_distance]) # Purple edge
        walls.append([0.0, FIELD_Y_SIZE - lateral_distance, FIELD_X_SIZE, FIELD_Y_SIZE]) # Red edge
        for wall in walls:
            for i in xrange(len(wall)):
                wall[i] = int(round(wall[i] / map_resolution))
            pathfinder.add_wall(wall[0], wall[1], wall[2], wall[3], wall_cost * 2.0)

        return (pathfinder, walls)


    def route(self, start_x, start_y, goal_x, goal_y, reference_team = TEAM_PURPLE):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorResetRoutePath())

        real_start_y = start_y #self.eventloop.robot.convert_y(start_y, reference_team)
        real_goal_y = self.eventloop.robot.convert_y(goal_y, reference_team)

        logger.log("Compute route from ({}, {}) to ({}, {})".format(start_x, real_start_y, goal_x, real_goal_y))

        start_cell_x = int(round(start_x / ROUTING_MAP_RESOLUTION))
        start_cell_y = int(round(real_start_y / ROUTING_MAP_RESOLUTION))
        goal_cell_x = int(round(goal_x / ROUTING_MAP_RESOLUTION))
        goal_cell_y = int(round(real_goal_y / ROUTING_MAP_RESOLUTION))

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
                simplified_path[i].x = simplified_path[i].x * ROUTING_MAP_RESOLUTION + ROUTING_MAP_RESOLUTION / 2.0
                y = self.eventloop.robot.convert_y(simplified_path[i].y * ROUTING_MAP_RESOLUTION + ROUTING_MAP_RESOLUTION / 2.0, reference_team)
                simplified_path[i].y = y
            simplified_path[-1].x = goal_x
            simplified_path[-1].y = goal_y

        return simplified_path[1:]


    def evaluate(self, start_x, start_y, goal_x, goal_y, reference_team = TEAM_PURPLE):
        real_start_y = start_y #self.eventloop.robot.convert_y(start_y, reference_team)
        real_goal_y = self.eventloop.robot.convert_y(goal_y, reference_team)

        start_cell_x = int(round(start_x / EVALUATOR_MAP_RESOLUTION))
        start_cell_y = int(round(real_start_y / EVALUATOR_MAP_RESOLUTION))
        goal_cell_x = int(round(goal_x / EVALUATOR_MAP_RESOLUTION))
        goal_cell_y = int(round(real_goal_y / EVALUATOR_MAP_RESOLUTION))

        path = self.evaluator.find(start_cell_x, start_cell_y, goal_cell_x, goal_cell_y)
        return len(path)


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
