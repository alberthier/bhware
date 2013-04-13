# encoding: utf-8

import sys

import packets
import builder
import logger
import position
import math
import datetime

from definitions import *




class ZoneData:

    def __init__(self, id):
        self.id = id
        self.x = 0.0
        self.y = 0.0




class Map:

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.build_module()
        import graphpathfinding
        self.pathfinder = graphpathfinding.PathFinder(ROBOT_GYRATION_RADIUS, ROBOT_GYRATION_RADIUS, 2.0 - ROBOT_GYRATION_RADIUS, 3.0 - ROBOT_GYRATION_RADIUS)

        # Half cake
        coords = []
        radius = 0.5 + ROBOT_GYRATION_RADIUS
        npoints = 6
        angles = []
        angles.append(-math.pi / 2.0)
        angles.append(-13 * math.pi / 32.0)
        for i in range(npoints - 2):
            angles.append(-math.pi / 2.0 + float(i + 1) * math.pi / float(npoints - 1))
        angles.append(13 * math.pi / 32.0)
        angles.append(math.pi / 2.0)
        for a in angles:
            x = math.cos(a) * radius
            y = 1.5 + math.sin(a) * radius
            coords.append((x, y))
        self.pathfinder.add_zone(coords)

        self.main_opponent_zone = self.add_circular_zone(0.130 + ROBOT_GYRATION_RADIUS)
        self.secondary_opponent_zone = self.add_circular_zone(0.080 + ROBOT_GYRATION_RADIUS)
        if IS_MAIN_ROBOT:
            self.teammate_zone = self.add_circular_zone(0.080 + ROBOT_GYRATION_RADIUS)
        else:
            self.teammate_zone = self.add_circular_zone(0.130 + ROBOT_GYRATION_RADIUS)

        self.pathfinder.field_config_done()

        self.pathfinder.enable_zone(self.main_opponent_zone.id, False)
        self.pathfinder.enable_zone(self.secondary_opponent_zone.id, False)
        self.pathfinder.enable_zone(self.teammate_zone.id, False)


    def add_circular_zone(self, radius):
        coords = []
        npoints = 8
        for i in range(npoints):
            a = float(i) * 2.0 * math.pi / float(npoints)
            x = math.cos(a) * radius
            y = math.sin(a) * radius
            coords.append((x, y))
        return ZoneData(self.pathfinder.add_zone(coords))


    def route(self, start, end):
        logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, end.x, end.y))
        start_date = datetime.datetime.now()
        (cost, path) = self.pathfinder.find_path(start.x, start.y, end.x, end.y)
        delta = datetime.datetime.now() - start_date
        if len(path) == 0:
            logger.log("No route found.")
        else:
            logger.log("Route computed. Cost: {}. compuation time: {}".format(cost, delta.total_seconds()))
        pose_path = []
        # remove start node and convert to poses
        for (x, y) in path[1:]:
            pose_path.append(position.Pose(x, y))
        self.send_to_simulator(pose_path)
        return (cost, pose_path)


    def send_to_simulator(self, path):
        logger.log("sending ...")
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorClearGraphMapEdges())
            packet = packets.SimulatorGraphMapEdges()
            for v in self.pathfinder.get_edges():
                packet.points.append(v)
                if len(packet.points) == 60:
                    self.event_loop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.event_loop.send_packet(packet)
            if path is not None:
                points = []
                for p in path:
                    points.append(p.x)
                    points.append(p.y)
                packet = packets.SimulatorGraphMapRoute(points = points)
                self.event_loop.send_packet(packet)


    def on_opponent_detected(self, packet, opponent_direction, x, y):
        if packet.robot == OPPONENT_ROBOT_MAIN:
            opponent = self.main_opponent_zone
        else:
            opponent = self.secondary_opponent_zone
        self.pathfinder.enable_zone(opponent.id, True)
        dx = x - opponent.x
        dy = y - opponent.y
        opponent.x = x
        opponent.y = y
        self.pathfinder.move_zone(opponent.id, dx, dy)


    def on_opponent_disappeared(self, opponent, opponent_direction):
        if opponent == OPPONENT_ROBOT_MAIN:
            opponent = self.main_opponent_zone
        else:
            opponent = self.secondary_opponent_zone
        self.pathfinder.enable_zone(opponent.id, False)


    def build_module(self):
        pyversion = "python{}.{}{}".format(sys.version_info.major, sys.version_info.minor, sys.abiflags)
        include_dir = sys.exec_prefix + "/include/" + pyversion
        lib_dir = sys.exec_prefix + "/lib"

        working_dir = os.path.dirname(__file__)
        source_file = "graphpathfinding.c"
        output_file = "graphpathfinding.so"
        exe = "gcc"

        params = ["-O2", "-shared", "-Wall", "-fPIC", "-o", output_file, "-I" + include_dir, "-L" + lib_dir, source_file, "-l" + pyversion]

        if sys.platform == "darwin" :
            params = ["-dynamiclib"] + params

        commands = [exe] + params

        bld = builder.Builder(source_file, output_file, commands, working_dir)
        bld.build()


    def on_interbot_position(self, packet):
        """
        :type packet: packets.InterbotPosition
        """
        teammate = self.teammate_zone
        x = packet.current_pose.x
        y = packet.current_pose.y
        self.pathfinder.enable_zone(teammate.id, True)
        dx = x - teammate.x
        dy = y - teammate.y
        teammate.x = x
        teammate.y = y
        if abs(dx)<0.01 and abs(dy)<0.01:
            logger.log("Movement too small, not updating zone (dx={} dy={})".format(dx,dy))
        else :
            logger.log("Move team mate zone dx={} dy={}".format(dx,dy))
            self.pathfinder.move_zone(teammate.id, dx, dy)
