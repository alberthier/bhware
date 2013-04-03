# encoding: utf-8

import sys

import packets
import builder
import logger
import position
import math

from definitions import *




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

        self.pathfinder.move_zone(self.main_opponent_zone, 1.4, 0.3)
        self.pathfinder.move_zone(self.secondary_opponent_zone, 1.4, 0.6)
        self.pathfinder.move_zone(self.teammate_zone, 1.5, 2.4)

        self.pathfinder.field_config_done()


    def add_circular_zone(self, radius):
        coords = []
        npoints = 8
        for i in range(npoints):
            a = float(i) * 2.0 * math.pi / float(npoints)
            x = math.cos(a) * radius
            y = math.sin(a) * radius
            coords.append((x, y))
        return self.pathfinder.add_zone(coords)


    def route(self, start, end):
        (cost, path) = self.pathfinder.find_path(start.x, start.y, end.x, end.y)
        logger.log("Route computed. Cost: {}".format(cost))
        logger.log(str(path))
        pose_path = []
        for (x, y) in path:
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
        pass


    def on_opponent_disapeared(self, opponent, opponent_direction):
        pass


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
