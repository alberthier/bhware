#!/usr/bin/env python
# encoding: utf-8


from definitions import *


class Edge(object):

    def __init__(self, node1, node2):
        self.blocked = False
        self.node1 = node1
        self.node2 = node2
        node1.edges.append(self)
        node2.edges.append(self)


class Node(object):

    def __init__(self, node_id, x, y, node_type = NODE_UNKNOWN, bonus = False, safe = False):
        self.id = node_id
        self.x = x
        self.y = y
        self.type = node_type
        self.is_bonus = bonus
        self.is_safe = safe
        self.edges = []


class HomologationWorld(object):

    def __init__(self):
        self.nodes = []
        self.edges = []
        # Tenir compte de la position du palet par rapport au centre du robot.
        self.nodes.append(Node("0", 200.0, 450.0))
        self.nodes.append(Node("1", 350.0, 800.0))
        self.nodes.append(Node("2", 1750.0, 800.0))
        self.nodes.append(Node("3", 1950.0, 600.0))
        self.nodes.append(Node("4", 150.0, 975.0))

        self.edges.append(Edge(self.nodes[0], self.nodes[1]))
        self.edges.append(Edge(self.nodes[1], self.nodes[2]))
        self.edges.append(Edge(self.nodes[1], self.nodes[4]))
        self.edges.append(Edge(self.nodes[2], self.nodes[3]))


    def get_trajectory(self):
        traj = []
        traj.append((self.nodes[0].x, self.nodes[0].y))
        traj.append((self.nodes[1].x, self.nodes[1].y))
        traj.append((self.nodes[2].x, self.nodes[2].y))
        traj.append((self.nodes[3].x, self.nodes[3].y))
        traj.append((self.nodes[2].x, self.nodes[2].y))
        traj.append((self.nodes[1].x, self.nodes[1].y))
        traj.append((self.nodes[4].x, self.nodes[4].y))
        return traj

world = HomologationWorld()
