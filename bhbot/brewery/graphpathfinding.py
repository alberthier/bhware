#!/usr/bin/env python
# encoding: utf-8

import math
import tools

import packets
import logger

from definitions import *




class Node(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.edges = []




class Edge(object):

    def __init__(self, src, dst):
        self.src = src
        self.src.edges.append(self)
        self.dst = dst
        self.dst.edges.append(self)
        self.static_penality = 0.0
        self.opponent_penality = { OPPONENT_ROBOT_MAIN: 0.0, OPPONENT_ROBOT_SECONDARY: 0.0 }
        self.main_opponent_penality = 0.0
        self.secondary_opponent_penality = 0.0
        self.x1 = min(self.src.x, self.dst.x)
        self.x2 = max(self.src.x, self.dst.x)
        self.y1 = min(self.src.y, self.dst.y)
        self.y2 = max(self.src.y, self.dst.y)
        self.distance = tools.distance(self.src.x, self.src.y, self.dst.x, self.dst.y)
        if tools.quasi_equal(self.src.x, self.dst.x):
            self.a = None
            self.b = None
        else:
            self.a = (dst.y - src.y) / (dst.x - src.x)
            self.b = self.src.y - self.a * self.src.x


    def contains(self, x, y):
        if self.a == None:
            ok = tools.quasi_equal(x, self.x1) and (y > self.y1 and y < self.y2)
        else:
            ok = tools.quasi_equal(self.a * x + self.b, y)
            if tools.quasi_equal(self.y1, self.y2):
                ok = ok and tools.quasi_equal(self.y1, y)
            else:
                ok = ok and (y > self.y1 and y < self.y2)
            ok = ok and (x > self.x1 and x < self.x2)
        return ok


    def intersects(self, edge):
        if self.a is None:
            e2 = self
            e1 = edge
        else:
            e1 = self
            e2 = edge

        if e1.a == None and e2.a == None:
            return False
        if e2.a == None:
            cross_x = e2.src.x
        elif tools.quasi_equal(e1.a, e2.a):
            return False
        else:
            cross_x = (e2.b - e1.b) / (e1.a - e2.a)
        cross_y = e1.a * cross_x + e1.b

        return e1.contains(cross_x, cross_y) and e2.contains(cross_x, cross_y)


    def unlink(self):
        self.src.edges.remove(self)
        self.dst.edges.remove(self)




class Zone(object):

    def __init__(self, close_path, *nodes):
        self.nodes = list(nodes)
        self.edges = []
        for i in xrange(1, len(self.nodes)):
            edge = Edge(self.nodes[i - 1], self.nodes[i])
            self.nodes[i - 1].edges.append(edge)
            self.nodes[i].edges.append(edge)
            self.edges.append(edge)
        if close_path:
            edge = Edge(self.nodes[0], self.nodes[-1])
            self.nodes[0].edges.append(edge)
            self.nodes[-1].edges.append(edge)
            self.edges.append(edge)


    def intersects(self, edge):
        for e in self.edges:
            if e.intersects(edge):
                return True
        return False




class Map(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.zones = []
        self.zone_links_edges = []
        self.opponent_zones = { OPPONENT_ROBOT_MAIN: None, OPPONENT_ROBOT_SECONDARY: None }
        self.opponent_avoidance_ranges = { OPPONENT_ROBOT_MAIN: MAIN_OPPONENT_AVOIDANCE_RANGE, OPPONENT_ROBOT_SECONDARY: SECONDARY_OPPONENT_AVOIDANCE_RANGE }

        # Captain room
        self.add_zone(True, False, Node(0.5 - MAP_WALLS_DISTANCE, 0.4 + MAP_WALLS_DISTANCE),
                                   Node(0.52 + MAP_WALLS_DISTANCE, 0.4 + MAP_WALLS_DISTANCE),
                                   Node(0.52 + MAP_WALLS_DISTANCE, MAP_WALLS_DISTANCE))
        # Bridge
        self.add_zone(True, False, Node(1.25 - MAP_WALLS_DISTANCE, MAP_WALLS_DISTANCE),
                                   Node(1.25 - MAP_WALLS_DISTANCE, 0.380 + MAP_WALLS_DISTANCE),
                                   Node(2.0 - MAP_WALLS_DISTANCE, 0.380 + MAP_WALLS_DISTANCE))
        # Island
        self.add_zone(False, True, Node(0.875 - MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE),
                                   Node(0.875 - MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE),
                                   Node(1.125 + MAP_WALLS_DISTANCE, 2.025 + MAP_WALLS_DISTANCE),
                                   Node(1.125 + MAP_WALLS_DISTANCE, 0.975 - MAP_WALLS_DISTANCE))

        self.initial_setup()
        #self.add_opponent(2.0, 1.6, OPPONENT_ROBOT_MAIN)


    def add_zone(self, symetric, close_path, *nodes):
        self.zones.append(Zone(close_path, *nodes))
        if symetric:
            symetric_nodes = []
            for node in nodes:
                symetric_nodes.append(Node(node.x, FIELD_Y_SIZE - node.y))
            self.zones.append(Zone(close_path, *symetric_nodes))


    def initial_setup(self):
        handled_node_pairs = set()
        for zone in self.zones:
            for node1 in zone.nodes:
                for other_zone in self.zones:
                    if not zone is other_zone:
                        for node2 in other_zone.nodes:
                            if not node2 in zone.nodes:
                                if ((node1, node2) not in handled_node_pairs) or ((node2, node1) not in handled_node_pairs):
                                    edge = self.create_edge(node1, node2)
                                    if edge is not None:
                                        self.zone_links_edges.append(edge)
                                        node1.edges.append(edge)
                                        node2.edges.append(edge)
                                    handled_node_pairs.add((node1, node2))
                                    handled_node_pairs.add((node2, node1))


    def create_edge(self, node1, node2):
        edge = Edge(node1, node2)
        for zone in self.zones:
            if zone.intersects(edge):
                return None
        return edge


    def add_opponent(self, x, y, opponent):
        distance = self.opponent_avoidance_ranges[opponent]
        nodes = []
        nb_edges = 6
        for i in xrange(nb_edges):
            nodes.append(Node(x + distance * math.cos(float(i) * math.pi / float(nb_edges)), y + distance * math.sin(float(i) * math.pi / float(nb_edges))))
        zone = Zone(True, *nodes)
        self.zones.append(zone)
        self.link_zone(zone, opponent)
        self.opponent_zones[opponent] = zone


    def link_zone(self, zone, opponent):
        for node1 in zone.nodes:
            for other_zone in self.zones:
                if not other_zone is zone:
                    for node2 in zone.nodes:
                        edge = self.create_edge(node1, node2)
                        if edge is not None:
                            self.zone_links_edges.append(edge)
                            node1.edges.append(edge)
                            node2.edges.append(edge)

        edges_to_remove = []
        for edge1 in zone.edges:
            for other_zone in self.zones:
                if not other_zone is zone:
                    for edge2 in zone.edges:
                        if edge1.intersects(edge2):
                            edge2.opponent_penality[opponent] = FIELD_X_SIZE * FIELD_Y_SIZE
                            edges_to_remove.append(edge1)

        for edge in edges_to_remove:
            edge1.unlink()
            zone.edges.remove(edge)

        nodes_to_remove = []
        for node in zone.nodes:
            if len(node.edges) == 0:
                nodes_to_remove.append(node)

        for node in nodes_to_remove:
            zone.nodes.remove(node)


    def remove_opponent(self, opponent):
        for edge in self.zone_links_edges:
            edge.opponent_penality[opponent] = 0.0
        for edge in self.opponent_zones[opponent].edges:
            edge.unlink()
        self.opponent_zones[opponent] = None


    def send_to_simulator(self):
        if IS_HOST_DEVICE_PC:
            edges = list(self.zone_links_edges)
            for zone in self.zones:
                edges += zone.edges

            packet = packets.SimulatorGraphMapEdges()
            for edge in edges:
                packet.points.append(edge.src.x)
                packet.points.append(edge.src.y)
                packet.points.append(edge.dst.x)
                packet.points.append(edge.dst.y)
                if len(packet.points) == 60:
                    self.eventloop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.eventloop.send_packet(packet)


    def on_device_ready(self, packet):
        self.send_to_simulator()
