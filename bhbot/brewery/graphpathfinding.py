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


    def is_in_field(self):
        return self.x > 0.0 and self.x < FIELD_X_SIZE and self.y > 0.0 and self.y < FIELD_Y_SIZE




class Edge(object):

    def __init__(self, node1, node2):
        self.node1 = node1
        self.node1.edges.append(self)
        self.node2 = node2
        self.node2.edges.append(self)
        self.penality = 0.0


    def links(self, node1, node2):
        return (self.node1 is node1 and self.node2 is node2) or (self.node1 is node2 and self.node2 is node1)


    def unlink(self):
        self.node1.edges.remove(self)
        self.node2.edges.remove(self)




class AbstractZone(object):

    def __init__(self, forbidden, penality):
        self.forbidden = forbidden
        self.penality = penality
        self.nodes = []


    def create_symetric_zone(self):
        return NotImplementedError()


    def create_nodes(self, map):
        return NotImplementedError()


    def contains(self, node):
        return NotImplementedError()


    def intersects(self, node1, node2):
        return NotImplementedError()




class RectZone(AbstractZone):

    def __init__(self, x1, y1, x2, y2, forbidden, penality):
        AbstractZone.__init__(self, forbidden, penality)
        self.x1 = min(x1, x2)
        self.y1 = min(y1, y2)
        self.x2 = max(x1, x2)
        self.y2 = max(y1, y2)


    def create_symetric_zone(self):
        return RectZone(self.x1, FIELD_Y_SIZE - self.y1, self.x2, FIELD_Y_SIZE - self.y2, self.forbidden, self.penality)


    def create_nodes(self, map):
        self.nodes = [ map.get_node(self.x1, self.y1), map.get_node(self.x1, self.y2), map.get_node(self.x2, self.y1), map.get_node(self.x2, self.y2) ]
        return self.nodes


    def contains(self, node):
        return self.x1 < node.x and self.x2 > node.x and self.y1 < node.y and self.y2 > node.y


    def intersects(self, node1, node2):
        if self.contains(node1) or self.contains(node2):
            return True
        if self.x1 == min(node1.x, node2.x) and self.y1 == min(node1.y, node2.y) and self.x2 == max(node1.x, node2.x) and self.y2 == max(node1.y, node2.y):
            return True
        if tools.segment_intersects_segment(self.x1, self.y1, self.x1, self.y2, node1.x, node1.y, node2.x, node2.y):
            return True
        if tools.segment_intersects_segment(self.x1, self.y2, self.x2, self.y2, node1.x, node1.y, node2.x, node2.y):
            return True
        if tools.segment_intersects_segment(self.x2, self.y2, self.x2, self.y1, node1.x, node1.y, node2.x, node2.y):
            return True
        if tools.segment_intersects_segment(self.x2, self.y1, self.x1, self.y1, node1.x, node1.y, node2.x, node2.y):
            return True
        return False




class CircleZone(AbstractZone):

    RESOLUTION = 6

    def __init__(self, x, y, radius, forbidden, penality):
        AbstractZone.__init__(self, forbidden, penality)
        self.x = x
        self.y = y
        self.radius = radius

    def create_symetric_zone(self):
        return CircleZone(self.x, FIELD_Y_SIZE - self.y, self.radius, self.forbidden, self.penality)


    def create_nodes(self, map):
        self.nodes = []
        fresolution = float(self.RESOLUTION)
        for i in xrange(self.RESOLUTION):
            fi = float(i)
            node = map.get_node(self.x + self.radius * math.cos(fi * 2.0 * math.pi / fresolution), self.y + self.radius * math.sin(fi * 2.0 * math.pi / fresolution))
            nodes.append(node)
        return self.nodes


    def contains(self, node):
        return tools.distance(self.x, self.y, node.x, node.y) < self.radius


    def intersects(self, node1, node2):
        return NotImplementedError()




class Map(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.zones = []
        self.nodes = []
        self.edges = []
        self.penality = FIELD_X_SIZE * FIELD_Y_SIZE
        self.opponent_zones = { OPPONENT_ROBOT_MAIN: None, OPPONENT_ROBOT_SECONDARY: None }
        self.opponent_positions = { OPPONENT_ROBOT_MAIN: None, OPPONENT_ROBOT_SECONDARY: None }
        self.opponent_edges = { OPPONENT_ROBOT_MAIN: [], OPPONENT_ROBOT_SECONDARY: [] }
        self.changed_opponents = []
        self.start_node = None
        self.start_node_edges = []
        self.start_node_changed = False
        self.destination_node = None
        self.destination_node_edges = []
        self.destination_node_changed = False


    def get_node(self, x, y):
        for node in self.nodes:
            if tools.quasi_equal(x, node.x) and tools.quasi_equal(y, node.y):
                return node
        node = Node(x, y)
        self.nodes.append(node)
        return node


    def add_zone(self, symetric, create_nodes, zone):
        self.zones.append(zone)
        if create_nodes:
            zone.create_nodes(self)
        if symetric:
            symetric_zone = zone.create_symetric_zone()
            self.zones.append(symetric_zone)
            if create_nodes:
                symetric_zone.create_nodes(self)


    def link_nodes(self):
        for node in self.nodes:
            self.edges += self.link_node(node)


    def link_node(self, node):
        new_edges = []
        if node.is_in_field():
            for other_node in self.nodes:
                if other_node is not node and other_node.is_in_field():
                    already_exists = False
                    for existing_edge in node.edges:
                        if existing_edge.links(node, other_node):
                            already_exists = True
                            break
                    if not already_exists:
                        ok = True
                        penality = 0.0
                        for zone in self.zones:
                            intersects = zone.intersects(node, other_node)
                            if intersects:
                                if zone.forbidden:
                                    ok = False
                                    break
                                else:
                                    if zone.penality > penality:
                                        penality = zone.penality
                        if ok:
                            edge = Edge(node, other_node)
                            new_edges.append(edge)
        return new_edges


    def set_opponent(self, x, y, opponent):
        self.opponent_positions[opponent] = (x, y)
        self.changed_opponents.append(opponent)


    def clear_opponent(self, opponent):
        self.opponent_positions[opponent] = None
        self.changed_opponents.append(opponent)


    def synchronize(self):
        for opponent in self.changed_opponents:
            # Clear previous positions
            nodes = set()
            for edge in self.opponent_edges[opponent]:
                edge.unlink()
                nodes.add(edge.node1)
                nodes.add(edge.node2)
            self.opponent_edges[opponent] = []
            for node in nodes:
                if len(node.edges) == 0:
                    self.nodes.remove(node)
            self.opponent_zones[opponent] = None

            # Set new positions
            coords = self.opponent_positions[opponent]
            if coords is not None:
                (x, y) = coords
                opponent_zone = RectZone(x - MAP_WALLS_DISTANCE, y - MAP_WALLS_DISTANCE, x + MAP_WALLS_DISTANCE, y + MAP_WALLS_DISTANCE, False, self.penality)
                self.opponent_zones[opponent] = opponent_zone
                self.add_zone(False, True, opponent_zone)
                opponent_edges = []
                for node in opponent_zone.nodes:
                    opponent_edges += self.link_node(node)
                self.opponent_edges[opponent] = opponent_edges
        self.changed_opponents = []

        if self.start_node_changed:
            for edge in self.start_node_edges:
                edge.unlink()
            self.start_node_edges = self.link_node(self.start_node)
            self.start_node_changed = False
        if self.destination_node_changed:
            for edge in self.destination_node_edges:
                edge.unlink()
            self.destination_node_edges = self.link_node(self.destination_node)
            self.destination_node_changed = False


    def send_to_simulator(self):
        if IS_HOST_DEVICE_PC:
            packet = packets.SimulatorGraphMapEdges()
            edges = list(self.edges)
            for edgelist in self.opponent_edges.values():
                edges += edgelist
            edges += self.start_node_edges
            edges += self.destination_node_edges
            for edge in edges:
                packet.points.append(edge.node1.x)
                packet.points.append(edge.node1.y)
                packet.points.append(edge.node2.x)
                packet.points.append(edge.node2.y)
                if len(packet.points) == 60:
                    self.eventloop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.eventloop.send_packet(packet)


    def on_device_ready(self, packet):
        # Captain room
        self.add_zone(True, True, RectZone(0.5 - MAP_WALLS_DISTANCE,
                                           0.0,
                                           0.52 + MAP_WALLS_DISTANCE,
                                           0.4 + MAP_WALLS_DISTANCE,
                                           True, self.penality))
        # Bridge
        self.add_zone(True, True, RectZone(1.25 - MAP_WALLS_DISTANCE,
                                           0.0,
                                           2.0,
                                           0.380 + MAP_WALLS_DISTANCE,
                                           True, self.penality))
        # Island
        self.add_zone(False, True, RectZone(0.875 - MAP_WALLS_DISTANCE,
                                             0.975 - MAP_WALLS_DISTANCE,
                                             1.125 + MAP_WALLS_DISTANCE,
                                             2.025 + MAP_WALLS_DISTANCE,
                                             True, self.penality))
        # BH Secondary robot zone
        x1 = 0.70
        y1 = 2.20
        x2 = 2.0
        y2 = 3.0
        if packet.team == TEAM_RED:
            y1 = FIELD_Y_SIZE - y1
            y2 = FIELD_Y_SIZE - y2
        self.add_zone(False, False, RectZone(x1, y1, x2, y2, False, self.penality))

        self.link_nodes()

        self.set_opponent(1.5, 2.0, OPPONENT_ROBOT_MAIN)
        self.set_opponent(2.0, 2.0, OPPONENT_ROBOT_SECONDARY)
        #self.start_node = self.get_node(0.15, 1.0)
        #self.start_node_changed = True
        #self.destination_node = self.get_node(1.5, 1.0)
        #self.destination_node_changed = True
        self.synchronize()
        self.clear_opponent(OPPONENT_ROBOT_MAIN)
        #self.clear_opponent(OPPONENT_ROBOT_SECONDARY)
        self.synchronize()

        self.send_to_simulator()
