# encoding: utf-8

import tools

from definitions import *




class Pose(object):

    match_team = TEAM_UNKNOWN

    def __init__(self, x = 0.0, y = 0.0, angle = None, virtual=False):
        self.virt = VirtualPose(self)
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        if not virtual :
            self.x = x
            self.y = y
            self.angle = angle
        else :
            self.virt.x = x
            self.virt.y = y
            self.virt.angle = angle

    def __sub__(self, other):
        return tools.distance(self.x, self.y, other.x, other.y)


    def __repr__(self):
        return "Pose({}, {}, {})".format(self.x, self.y, self.angle)


    def __eq__(self, other):
        if other is None:
            return False
        return self.x == other.x and self.y == other.y and self.angle == other.angle




class VirtualPose(object):

    def __init__(self, virt_pose):

        """
        Initialize the VirtualPose with a real Pose

        @type virt_pose: Pose
        """
        self.real_pose = virt_pose


    def set_x(self, x):
        self.real_pose.x = x


    def get_x(self):
        return self.real_pose.x


    def set_y(self, y):
        if Pose.match_team == TEAM_YELLOW:
            self.real_pose.y = FIELD_Y_SIZE - y
        else :
            self.real_pose.y = y

    def get_y(self):
        if Pose.match_team == TEAM_YELLOW:
            return FIELD_Y_SIZE - self.real_pose.y
        else :
            return self.real_pose.y


    def set_angle(self, angle):
        if angle :
            angle = tools.normalize_angle(angle)
        if angle and Pose.match_team == TEAM_YELLOW:
            self.real_pose.angle = -angle
        else :
            self.real_pose.angle = angle


    def get_angle(self):
        if self.real_pose.angle and Pose.match_team == TEAM_YELLOW:
            return -self.real_pose.angle
        else :
            return self.real_pose.angle


    x = property(get_x, set_x)
    y = property(get_y, set_y)
    angle = property(get_angle, set_angle)
