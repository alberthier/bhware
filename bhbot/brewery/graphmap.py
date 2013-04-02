# encoding: utf-8

import sys

import packets
import builder
import logger

from definitions import *




class Map:

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.build_module()
        import graphpathfinding
        self.pathfinder = graphpathfinding.PathFinder(0.1, 0.1, 1.9, 2.9)


    def send_to_simulator(self):
        logger.log("sending ...")
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorClearGraphMapEdges())
            packet = packets.SimulatorGraphMapEdges()
            for v in self.pathfinder.get_edges():
                logger.log(v)
                packet.points.append(v)
                if len(packet.points) == 60:
                    self.event_loop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.event_loop.send_packet(packet)


    def on_device_ready(self, packet):
        self.pathfinder.add_zone([(0.5, 0.8), (1.3, 1.0), (0.8, 1.2)])
        self.pathfinder.field_config_done()
        self.send_to_simulator()


    def build_module(self):
        pyversion = "python{}.{}{}".format(sys.version_info.major, sys.version_info.minor, sys.abiflags)
        include_dir = sys.exec_prefix + "/include/" + pyversion
        lib_dir = sys.exec_prefix + "/lib"

        working_dir = os.path.dirname(__file__)
        source_file = "graphpathfinding.c"
        output_file = "graphpathfinding.so"
        exe = "gcc"

        #params = ["-O2", "-shared", "-fPIC", "-o", output_file, "-I" + include_dir, "-L" + lib_dir, source_file, "-l" + pyversion]
        params = ["-g", "-shared", "-fPIC", "-o", output_file, "-I" + include_dir, "-L" + lib_dir, source_file, "-l" + pyversion]

        if sys.platform == "darwin" :
            params = ["-dynamiclib"] + params

        commands = [exe] + params

        bld = builder.Builder(source_file, output_file, commands, working_dir)
        bld.build()
