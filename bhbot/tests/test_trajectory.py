import unittest

import run_tests
run_tests.patch_pythonpath()

import math

import trajectory
from definitions import *
import position




class TestPoseBase(unittest.TestCase):

    def setUp(self):
        self.pose = position.Pose(1.0,1.0,math.pi)


    def test_base(self):
        self.assertEquals(self.pose.x,1.0)
        self.assertEquals(self.pose.y,1.0)
        self.assertEquals(self.pose.angle,math.pi)




class TestPoseYellow(unittest.TestCase):

    def setUp(self):
        position.Pose.match_team = TEAM_YELLOW
        self.pose = position.Pose(1.0,1.0,math.pi)


    def test_base(self):
        self.assertEquals(self.pose.x,1.0)
        self.assertEquals(self.pose.y,1.0)
        self.assertEquals(self.pose.angle,math.pi)


    def test_real(self):
        self.assertEquals(self.pose.virt.x,1.0)
        self.assertEquals(self.pose.virt.y,1.0)
        self.assertEquals(self.pose.virt.angle,math.pi)




class TestPoseYellow(unittest.TestCase):

    def setUp(self):
        position.Pose.match_team = TEAM_YELLOW
        self.pose = position.Pose(1.0,1.0,math.pi)


    def test_virt(self):
        self.assertEquals(self.pose.virt.x,1.0)
        self.assertEquals(self.pose.virt.y,1.0)
        self.assertEquals(self.pose.virt.angle,math.pi)




class TestPoseRed(unittest.TestCase):

    def setUp(self):
        position.Pose.match_team = TEAM_RED
        self.pose = position.Pose(1.0,1.0,math.pi)

    def test_virt(self):
        self.assertEquals(self.pose.virt.x, 1.0)
        self.assertEquals(self.pose.virt.y, FIELD_Y_SIZE - 1.0)
        self.assertEquals(self.pose.virt.angle,-math.pi)




class TestMap(unittest.TestCase):

    def setUp(self):
        self.map = trajectory.Map(self)


    def send_packet(self, packet):
        # Dummy path through send_packet method for map
        pass


    def test_strait_route(self):
        route = self.map.route(position.Pose(YELLOW_START_X, YELLOW_START_Y), position.Pose(YELLOW_START_X, 1.5))
        self.assertEquals(len(route), 1)
        self.assertTrue(len(route[0]) > 0)
        p = route[0][0]
        self.assertEquals(p.x, YELLOW_START_X)
        self.assertEquals(p.y, 1.5)


    def test_unroutable(self):
        route = self.map.route(position.Pose(YELLOW_START_X, YELLOW_START_Y), position.Pose(1.99, 2.99))
        self.assertEquals(len(route), 0)


    def test_escape_from_forbidden_zone(self):
        route = self.map.route(position.Pose(YELLOW_START_X + MAP_WALLS_DISTANCE, YELLOW_START_Y), position.Pose(1.6, 2.0))
        self.assertNotEquals(len(route), 0)




if __name__ == '__main__':
    unittest.main()
