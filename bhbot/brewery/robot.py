#!/usr/bin/env python
# encoding: utf-8

import trajectory

from definitions import *




class Robot(object):

    def __init__(self):
        self.pose = trajectory.Pose(0.0, 0.0, 0.0)
        self.team = TEAM_UNKNOWN
