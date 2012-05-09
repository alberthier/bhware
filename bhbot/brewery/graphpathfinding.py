#!/usr/bin/env python
# encoding: utf-8

import math
import tools

import packets
import logger
import trajectory

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
        self.came_from = None


    def is_in_field(self):
        return self.x > 0.0 and self.x < FIELD_X_SIZE and self.y > 0.0 and self.y < FIELD_Y_SIZE


    def neighbor_nodes(self):
        neighbors = []
        for edge in self.edges:
            if self is edge.node1:
                neighbors.append((edge, edge.node2))
            else:
                neighbors.append((edge, edge.node1))
        return neighbors




class Edge(object):

    def __init__(self, node1, node2):
        self.node1 = node1
        self.node1.edges.append(self)
        self.node2 = node2
        self.node2.edges.append(self)
        self.penality = 0.0
        self.distance = tools.distance(self.node1.x, self.node1.y, self.node2.x, self.node2.y)


    def links(self, node1, node2):
        return (self.node1 is node1 and self.node2 is node2) or (self.node1 is node2 and self.node2 is node1)


    def unlink(self):
        self.node1.edges.remove(self)
        self.node1 = None
        self.node2.edges.remove(self)
        self.node2 = None




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
        self.x_squared = x ** 2
        self.y = y
        self.y_squared = y ** 2
        self.radius = radius
        self.radius_squared = radius ** 2

    def create_symetric_zone(self):
        return CircleZone(self.x, FIELD_Y_SIZE - self.y, self.radius, self.forbidden, self.penality)


    def create_nodes(self, map):
        self.nodes = []
        fresolution = float(self.RESOLUTION)
        for i in xrange(self.RESOLUTION):
            fi = float(i)
            node = map.get_node(self.x + self.radius * math.cos(fi * 2.0 * math.pi / fresolution), self.y + self.radius * math.sin(fi * 2.0 * math.pi / fresolution))
            self.nodes.append(node)
        return self.nodes


    def contains(self, node):
        return tools.distance(self.x, self.y, node.x, node.y) < self.radius


    def intersects(self, node1, node2):
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        a = dx ** 2 + dy ** 2
        b = 2.0 * (dx * (node1.x - self.x) + dy * (node1.y - self.y))
        c = node1.x ** 2 + node1.y ** 2 + self.x_squared + self.y_squared - 2.0 * (node1.x * self.x + node1.y * self.y) - self.radius_squared
        d = b ** 2 - 4 * a * c

        if d < 0.0:
            return False

        u = ((self.x - node1.x) * dx + (self.y - node1.y) * dy) / a

        return 0.0 <= u and u <= 1.0




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
        self.penality = FIELD_X_SIZE * FIELD_Y_SIZE
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
                        for zone in self.zones:
                            if zone.intersects(node, other_node) and zone.forbidden:
                                ok = False
                                break
                        if ok:
                            edge = Edge(node, other_node)
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
        logger.log("Synchronize")
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
                opponent_zone = CircleZone(x, y, radius, False, self.penality)
                self.opponent_zones[opponent] = opponent_zone
                logger.log("Add opponent zone at {} {}".format(x, y))
                self.add_zone(False, True, opponent_zone)
                for node in opponent_zone.nodes:
                    self.link_node(node)
        self.changed_opponents = set()

        # Update start and destination points
        for point in self.points:
            if point.next_coords is not None:
                for edge in point.dynamic_edges:
                    edge.unlink()
                if point.node is not None:
                    point.node.refcount -= 1
                    if len(point.node.edges) == 0 and point.node.refcount == 0:
                        self.nodes.remove(point.node)
                point.node = self.get_node(*point.next_coords)
                point.dynamic_edges = self.link_node(point.node)
                point.next_coords = None

        # Update edge penalities:
        for edge in self.all_edges():
            edge.penality = 0.0
            for zone in self.zones:
                if zone.intersects(edge.node1, edge.node2):
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
        start_node.came_from = None

        closedset = set()
        openset = [ start_node ]

        while len(openset) != 0:
            current = openset[0]
            if tools.quasi_equal(current.x, goal_node.x) and tools.quasi_equal(current.y, goal_node.y):
                path = []
                path_length = 0.0
                n = current
                while n.came_from != None:
                    path.insert(0, trajectory.Pose(n.x, n.y))
                    if n.came_from != None:
                        path_length += tools.distance(n.x, n.y, n.came_from.x, n.came_from.y)
                    n = n.came_from
                logger.log("Route computed. Length: {}.".format(path_length))
                if IS_HOST_DEVICE_PC:
                    packet = packets.SimulatorGraphMapRoute()
                    for path_element in path:
                        packet.points.append(path_element.x)
                        packet.points.append(path_element.y)
                    self.eventloop.send_packet(packet)
                return (path_length, path)

            del openset[0]
            closedset.add(current)

            for edge, neighbor in current.neighbor_nodes():
                if not neighbor in closedset:
                    tentative_g_score = current.g_score + self.effective_cost(edge)
                    if not neighbor in openset:
                        neighbor.h_score = self.heuristic_cost_estimate(neighbor, goal_node)
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current
                        self.insert_sorted(openset, neighbor)
                    elif tentative_g_score < neighbor.g_score:
                        neighbor.g_score = tentative_g_score
                        neighbor.f_score = neighbor.g_score + neighbor.h_score
                        neighbor.came_from = current

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


    def opponent_detected(self, opponent, x, y):
        self.set_opponent(opponent, x, y)


    def opponent_disapeared(self, opponent):
        self.clear_opponent(opponent)
