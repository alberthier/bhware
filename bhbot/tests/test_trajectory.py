import unittest

import run_tests
run_tests.patch_pythonpath()

import math
import definitions

import trajectory

class TestPoseBase(unittest.TestCase):
    def setUp(self):
        self.pose = trajectory.Pose(1.0,1.0,math.pi)
    def test_base(self):
        self.assertEquals(self.pose.x,1.0)
        self.assertEquals(self.pose.y,1.0)
        self.assertEquals(self.pose.angle,math.pi)


class TestPosePurple(unittest.TestCase):
    def setUp(self):
        trajectory.Pose.match_team = definitions.TEAM_PURPLE
        self.pose = trajectory.Pose(1.0,1.0,math.pi)
    def test_base(self):
        self.assertEquals(self.pose.x,1.0)
        self.assertEquals(self.pose.y,1.0)
        self.assertEquals(self.pose.angle,math.pi)
    def test_real(self):
        self.assertEquals(self.pose.virt.x,1.0)
        self.assertEquals(self.pose.virt.y,1.0)
        self.assertEquals(self.pose.virt.angle,math.pi)

class TestPosePurple(unittest.TestCase):
    def setUp(self):
        trajectory.Pose.match_team = definitions.TEAM_PURPLE
        self.pose = trajectory.Pose(1.0,1.0,math.pi)
    def test_virt(self):
        self.assertEquals(self.pose.virt.x,1.0)
        self.assertEquals(self.pose.virt.y,1.0)
        self.assertEquals(self.pose.virt.angle,math.pi)

class TestPoseRed(unittest.TestCase):
    def setUp(self):
        trajectory.Pose.match_team = definitions.TEAM_RED
        self.pose = trajectory.Pose(1.0,1.0,math.pi)
    def test_virt(self):
        self.assertEquals(self.pose.virt.x, 1.0)
        self.assertEquals(self.pose.virt.y, definitions.FIELD_Y_SIZE - 1.0)
        self.assertEquals(self.pose.virt.angle,-math.pi)


if __name__ == '__main__':
    unittest.main()
