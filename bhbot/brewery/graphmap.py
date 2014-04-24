# encoding: utf-8

import sys
import itertools

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
        self.pathfinder = graphpathfinding.PathFinder(ROBOT_GYRATION_RADIUS, ROBOT_GYRATION_RADIUS, FIELD_X_SIZE - ROBOT_GYRATION_RADIUS, FIELD_Y_SIZE - ROBOT_GYRATION_RADIUS)


    def on_device_ready(self, packet):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorClearGraphMapZones())

        self.main_opponent_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.130 + ROBOT_GYRATION_RADIUS))
        self.secondary_opponent_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.080 + ROBOT_GYRATION_RADIUS))
        if IS_MAIN_ROBOT:
            self.teammate_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.080 + ROBOT_GYRATION_RADIUS))
        else:
            self.teammate_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.130 + ROBOT_GYRATION_RADIUS))

        # Add Field obstacles
        offset = ROBOT_GYRATION_RADIUS
        symetrical_zones = []
        # Fruit baskets
        basket_offset = math.sin(math.pi / 8.0) * offset
        symetrical_zones.append([(0.0, 0.4 - offset),
                                 (0.3 + basket_offset, 0.4 - offset),
                                 (0.3 + offset, 0.4 - basket_offset),
                                 (0.3 + offset, 1.1 + basket_offset),
                                 (0.3 + basket_offset, 1.1 + offset),
                                 (0.0, 1.1 + offset)])
        #symetrical_zones.append(self.create_rect_coords(0.0, 0.4 - offset, 0.3 + offset, 0.7 + 2 * offset))
        # Bottom hearts of fire
        symetrical_zones.append(self.create_quarter_coords(FIELD_X_SIZE, 0.0, 0.25 + offset))
        # Add symetrical zones
        for zone in symetrical_zones:
            self.add_zone(zone)
            symetrical = []
            for x, y in zone:
                symetrical.append((x, FIELD_Y_SIZE - y))
            self.add_zone(symetrical)
        # Add central heart of fire
        self.add_zone(self.create_circular_coords(FIELD_X_SIZE / 2.0, FIELD_Y_SIZE / 2.0, 0.15 + offset))

        self.pathfinder.field_config_done()

        self.enable_zone(self.main_opponent_zone.id, False)
        self.enable_zone(self.secondary_opponent_zone.id, False)
        self.enable_zone(self.teammate_zone.id, False)


    def create_quarter_coords(self, x, y, radius):
        coords = [(x, y)]
        npoints = 4
        for i in range(npoints):
            a = float(i) * (math.pi / 2.0) / float(npoints - 1)
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x - cx, y + cy))
        return coords


    def create_circular_coords(self, x, y, radius):
        coords = []
        npoints = 8
        for i in range(npoints):
            a = float(i) * 2.0 * math.pi / float(npoints)
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x + cx, y + cy))
        return coords


    def create_rect_coords(self, x, y, width, height):
        return [(x, y),
                (x + width, y),
                (x + width, y + height),
                (x, y + height)]


    def add_zone(self, coords):
        id = self.pathfinder.add_zone(coords)
        if IS_HOST_DEVICE_PC:
            flattened_coords = list(itertools.chain.from_iterable(coords))
            self.event_loop.send_packet(packets.SimulatorAddGraphMapZone(id = id, points = flattened_coords))
        return ZoneData(id)


    def enable_zone(self, id, enabled):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorEnableGraphMapZone(id = id, enabled = enabled))
        self.pathfinder.enable_zone(id, enabled)


    def move_zone(self, id, dx, dy):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorMoveGraphMapZone(id = id, dx = dx, dy = dy))
        self.pathfinder.move_zone(id, dx, dy)


    def route(self, start, end):
        logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, end.x, end.y))
        start_date = datetime.datetime.now()
        (cost, path) = self.pathfinder.find_path(start.x, start.y, end.x, end.y)
        delta = datetime.datetime.now() - start_date
        if len(path) == 0:
            logger.log("No route found. Cost: {}. compuation time: {}".format(cost, delta.total_seconds()))
            return (None, [])
        else:
            logger.log("Route computed. Cost: {}. compuation time: {}".format(cost, delta.total_seconds()))
        pose_path = []
        # remove start node and convert to poses
        for (x, y) in path[1:]:
            pose_path.append(position.Pose(x, y))
        self.send_to_simulator(pose_path)
        return (cost, pose_path)


    def send_to_simulator(self, path):
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


    def on_opponent_position(self, packet):
        if packet.robot == OPPONENT_ROBOT_MAIN:
            zone = self.main_opponent_zone
        else:
            zone = self.secondary_opponent_zone
        if packet.x is not None and packet.y is not None:
            self.enable_zone(zone.id, True)
            dx = packet.x - zone.x
            dy = packet.y - zone.y
            zone.x = packet.x
            zone.y = packet.y
            self.move_zone(zone.id, dx, dy)
        else:
            self.enable_zone(zone.id, False)


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
        if TEAMMATE_POSITION_IN_MAP:
            self.enable_zone(self.teammate_zone.id, True)
            dx = packet.pose.x - self.teammate_zone.x
            dy = packet.pose.y - self.teammate_zone.y
            self.teammate_zone.x = packet.pose.x
            self.teammate_zone.y = packet.pose.y
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                # logger.log("Move team mate zone dx={} dy={}".format(dx,dy))
                self.move_zone(self.teammate_zone.id, dx, dy)
