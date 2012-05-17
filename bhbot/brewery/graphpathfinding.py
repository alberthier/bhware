#!/usr/bin/env python
# encoding: utf-8

import math
import tools

import packets
import logger
import trajectory
import geometry

from definitions import *




class Node(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.refcount = 1
        self.edges = []
        self.g_score = 0.0
        self.h_score = 0.0
        self.f_score = 0.0
        self.path_edge = None
        self.is_in_openset = False
        self.is_in_closedset = False


    def is_in_field(self):
        return self.x > MAP_WALLS_DISTANCE and self.x < FIELD_X_SIZE - MAP_WALLS_DISTANCE and self.y > MAP_WALLS_DISTANCE and self.y < FIELD_Y_SIZE - MAP_WALLS_DISTANCE


    def neighbor_nodes(self):
        neighbors = []
        for edge in self.edges:
            if self is edge.node1:
                neighbors.append((edge, edge.node2))
            else:
                neighbors.append((edge, edge.node1))
        return neighbors


    def __repr__(self):
        return "Node({}, {})".format(self.x, self.y)




class Edge(object):

    def __init__(self, node1, node2, segment = None):
        self.node1 = node1
        self.node1.edges.append(self)
        self.node2 = node2
        self.node2.edges.append(self)
        if segment is not None:
            self.segment = segment
        else:
            self.segment = geometry.Segment(node1.x, node1.y, node2.x, node2.y)
        self.penality = 0.0
        self.distance = tools.distance(self.node1.x, self.node1.y, self.node2.x, self.node2.y)


    def links(self, node1, node2):
        return (self.node1 is node1 and self.node2 is node2) or (self.node1 is node2 and self.node2 is node1)


    def unlink(self):
        self.node1.edges.remove(self)
        self.node1 = None
        self.node2.edges.remove(self)
        self.node2 = None


    def other_node(self, node):
        if node is self.node1:
            return self.node2
        return self.node1


    def __repr__(self):
        return "Edge(({}, {}), ({}, {}), {})".format(self.node1.x, self.node1.y, self.node2.x, self.node2.y, self.penality)




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

    def __init__(self, x, y, x_size, y_size, forbidden, penality):
        AbstractZone.__init__(self, forbidden, penality)
        self.x = x
        self.y = y
        self.x_size = x_size
        self.y_size = y_size
        ZONE_DELTA = 0.005
        self.shape = geometry.Rectangle(x + ZONE_DELTA, y + ZONE_DELTA, x_size - 2.0 * ZONE_DELTA, y_size - 2.0 * ZONE_DELTA)


    def create_symetric_zone(self):
        return RectZone(self.x, FIELD_Y_SIZE - self.y - self.y_size, self.x_size, self.y_size, self.forbidden, self.penality)


    def create_nodes(self, map):
        self.nodes = [ map.get_node(self.x              , self.y              ),
                       map.get_node(self.x              , self.y + self.y_size),
                       map.get_node(self.x + self.x_size, self.y              ),
                       map.get_node(self.x + self.x_size, self.y + self.y_size) ]
        return self.nodes


    def contains(self, node):
        return self.shape.contains(node.x, node.y)


    def intersects(self, segment):
        return self.shape.intersects(segment)




class DynamicPoint(object):

    def __init__(self):
        self.node = None
        self.next_coords = None
        self.dynamic_edges = []




class Map(object):

    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.zones = []
        self.nodes = []
        self.base_edges = []
        self.penality = FIELD_X_SIZE * FIELD_Y_SIZE * 4.0
        self.opponent_zones = { OPPONENT_ROBOT_MAIN: None, OPPONENT_ROBOT_SECONDARY: None }
        self.opponent_positions = { OPPONENT_ROBOT_MAIN: None, OPPONENT_ROBOT_SECONDARY: None }
        self.changed_opponents = set()
        self.points = [ DynamicPoint(),  # Start Point
                        DynamicPoint() ] # Destination Point


    def all_edges(self):
        edges = list(self.base_edges)
        for zone in self.opponent_zones.values():
            if zone != None:
                for node in zone.nodes:
                    edges += node.edges
        for point in self.points:
            edges += point.dynamic_edges
        return edges


    def get_node(self, x, y):
        for node in self.nodes:
            if tools.quasi_equal(x, node.x) and tools.quasi_equal(y, node.y):
                node.refcount += 1
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
            self.base_edges += self.link_node(node)


    def link_node(self, node, allow_escape = False):
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
                        segment = geometry.Segment(node.x, node.y, other_node.x, other_node.y)
                        for zone in self.zones:
                            if zone.intersects(segment) and zone.forbidden:
                                if not (allow_escape and zone.contains(node)):
                                    ok = False
                                    break
                        if ok:
                            edge = Edge(node, other_node, segment)
                            new_edges.append(edge)

        return new_edges


    def set_opponent(self, opponent, x, y):
        self.opponent_positions[opponent] = (x, y)
        self.changed_opponents.add(opponent)


    def clear_opponent(self, opponent):
        self.opponent_positions[opponent] = None
        self.changed_opponents.add(opponent)


    def set_points(self, x1, y1, x2, y2):
        self.points[0].next_coords = (x1, y1)
        self.points[1].next_coords = (x2, y2)


    def synchronize(self):
        for opponent in self.changed_opponents:
            # Clear previous positions
            if self.opponent_zones[opponent] != None:
                for node in self.opponent_zones[opponent].nodes:
                    node.refcount -= 1
                    if node.refcount == 0:
                        for edge in list(node.edges):
                            edge.unlink()
                            for point in self.points:
                                if edge in point.dynamic_edges:
                                    point.dynamic_edges.remove(edge)
                        self.nodes.remove(node)
                self.zones.remove(self.opponent_zones[opponent])
                self.opponent_zones[opponent] = None

            # Set new positions
            coords = self.opponent_positions[opponent]
            if coords is not None:
                (x, y) = coords
                if opponent == OPPONENT_ROBOT_MAIN:
                    radius = MAIN_OPPONENT_AVOIDANCE_RANGE
                else:
                    radius = SECONDARY_OPPONENT_AVOIDANCE_RANGE
                opponent_zone = RectZone(x - radius, y - radius, x + radius * 2.0, y + radius * 2.0, False, self.penality * 2.0)
                self.opponent_zones[opponent] = opponent_zone
                logger.log("Add opponent zone at {} {}".format(x, y))
                self.add_zone(False, True, opponent_zone)
                for node in opponent_zone.nodes:
                    self.link_node(node)
        self.changed_opponents = set()

        # Update start and destination points
        for idx, point in enumerate(self.points):
            if point.next_coords is not None:
                for edge in point.dynamic_edges:
                    edge.unlink()
                if point.node is not None:
                    point.node.refcount -= 1
                    if len(point.node.edges) == 0 and point.node.refcount == 0:
                        self.nodes.remove(point.node)
                point.node = self.get_node(*point.next_coords)
                point.dynamic_edges = self.link_node(point.node, idx == 0)
                point.next_coords = None

        # Update edge penalities:
        for edge in self.all_edges():
            edge.penality = 0.0
            for zone in self.zones:
                if zone.intersects(edge.segment):
                    if zone.penality > edge.penality:
                        edge.penality = zone.penality

        self.send_to_simulator()


    def send_to_simulator(self):
        if IS_HOST_DEVICE_PC:
            self.eventloop.send_packet(packets.SimulatorClearGraphMapEdges())
            packet = packets.SimulatorGraphMapEdges()
            for edge in self.all_edges():
                packet.points.append(edge.node1.x)
                packet.points.append(edge.node1.y)
                packet.points.append(edge.node2.x)
                packet.points.append(edge.node2.y)
                packet.points.append(edge.penality)
                if len(packet.points) == 60:
                    self.eventloop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.eventloop.send_packet(packet)


    def on_device_ready(self, packet):
        # Captain room
        self.add_zone(True, True, RectZone(max(0.5 - MAP_WALLS_DISTANCE, PURPLE_START_X + 0.005),
                                           0.000,
                                           0.020 + MAP_WALLS_DISTANCE * 2.0,
                                           0.400 + MAP_WALLS_DISTANCE,
                                           True, self.penality))
        # Bridge
        self.add_zone(True, True, RectZone(1.250 - MAP_WALLS_DISTANCE,
                                           0.000,
                                           0.750 + MAP_WALLS_DISTANCE * 2.0,
                                           0.380 + MAP_WALLS_DISTANCE,
                                           True, self.penality))
        # Island
        self.add_zone(False, True, RectZone(0.845 - MAP_WALLS_DISTANCE,
                                            0.945 - MAP_WALLS_DISTANCE,
                                            0.310 + MAP_WALLS_DISTANCE * 2.0,
                                            1.110 + MAP_WALLS_DISTANCE * 2.0,

        # BH Secondary robot zone
        x = 0.700
        y = 2.200
        x_size = 1.300
        y_size = 0.800
        if packet.team == TEAM_RED:
            y = FIELD_Y_SIZE - y - y_size
        self.add_zone(False, False, RectZone(x, y, x_size, y_size, False, self.penality))

        self.link_nodes()

        self.synchronize()

        self.send_to_simulator()


    def route(self, start, goal):
        return [ self.find_path(start, goal)[1] ]


    def evaluate(self, start, goal):
        cost = self.find_path(start, goal)[0]
        if cost is None:
            cost = (FIELD_X_SIZE * FIELD_Y_SIZE) ** 2
        return cost


    def find_path(self, start, goal):
        logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, goal.x, goal.y))

        self.set_points(start.x, start.y, goal.x, goal.y)
        self.synchronize()

        goal_node = self.points[1].node
        start_node = self.points[0].node
        start_node.g_score = 0.0
        start_node.h_score = self.heuristic_cost_estimate(start_node, goal_node)
        start_node.f_score = start_node.g_score + start_node.h_score
        start_node.path_edge = None

        for node in self.nodes:
            node.is_in_openset = False
            node.is_in_closedset = False

        openset = [ start_node ]
        start_node.is_in_openset = True

        while len(openset) != 0:
            current = openset[0]
            if tools.quasi_equal(current.x, goal_node.x) and tools.quasi_equal(current.y, goal_node.y):
                path = []
                path_length = 0.0
                path_cost = 0.0
                n = current
                while n.path_edge != None:
                    path.insert(0, trajectory.Pose(n.x, n.y))
                    path_length += n.path_edge.distance
                    path_cost += n.path_edge.distance + n.path_edge.penality
                    n = n.path_edge.other_node(n)
                logger.log("Route computed. Length: {}, Cost: {}".format(path_length, path_cost))
                if IS_HOST_DEVICE_PC:
                    packet = packets.SimulatorGraphMapRoute()
                    for path_element in path:
                        packet.points.append(path_element.x)
                        packet.points.append(path_element.y)
                    self.eventloop.send_packet(packet)
                return (path_cost, path)

            del openset[0]
            current.is_in_openset = False
            current.is_in_closedset = True

            for edge, neighbor in current.neighbor_nodes():
                if not neighbor.is_in_closedset:
                    tentative_g_score = current.g_score + self.effective_cost(edge)
                    if not neighbor.is_in_openset:
                        neighbor.h_score = self.heuristic_cost_estimate(neighbor, goal_node)
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.path_edge = edge
                        self.insert_sorted(openset, neighbor)
                        neighbor.is_in_openset = True
                    elif tentative_g_score < neighbor.g_score:
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.path_edge = edge

        logger.log("No route found")
        return (None, [])


    def effective_cost(self, edge):
        return edge.distance + edge.penality


    def heuristic_cost_estimate(self, node1, node2):
        return tools.distance(node1.x, node1.y, node2.x, node2.y)


    def insert_sorted(self, openset, neighbor):
        for i in xrange(len(openset)):
            if openset[i].f_score >= neighbor.f_score:
                openset.insert(i, neighbor)
                return
        openset.append(neighbor)


    def on_goto_finished(self, packet):
        pass


    def pop_blocked_zone(self):
        pass


    def opponent_detected(self, packet, x, y):
        self.set_opponent(packet.robot, x, y)


    def opponent_disapeared(self, opponent):
        self.clear_opponent(opponent)
