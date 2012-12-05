# encoding: utf-8

import sys
import trajectory
import logger
import tools

from definitions import *




class Goal(object):

    def __init__(self, identifier, weight, x, y, direction, handler_state, ctor_parameters = None):
        self.identifier = identifier
        self.weight = weight
        self.x = x
        self.y = y
        self.direction = direction
        self.handler_state = handler_state
        self.navigation_cost = 0.0
        self.ctor_parameters = ctor_parameters
        self.score = 0.0
        self.penality = 0.0




class GoalManager(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.harvesting_goals = []
        self.emptying_goals = []
        self.count = 0

    def next_goal(self):
        if self.event_loop.robot.tank_full:
            return self.get_best_goal(self.emptying_goals)
        else:
            return self.get_best_goal(self.harvesting_goals)


    def get_best_goal(self, goals):
        self.count += 1
        if self.count == 2:
            self.event_loop.opponent_detector.enable()

        if not goals :
            return None

        for goal in goals:
            pose = trajectory.Pose(goal.x, goal.y, virtual=True)
            logger.log("Evaluate goal {}".format(goal.identifier))
            if GOAL_EVALUATION_USES_PATHFINDING:
                goal.navigation_cost = self.event_loop.eval_map.evaluate(self.event_loop.robot.pose, pose)
            else:
                goal.navigation_cost = tools.distance(self.event_loop.robot.pose.x, self.event_loop.robot.pose.y, pose.x, pose.y)
            goal.score = goal.penality
            goal.penality = 0.0

        for order, goal in enumerate(sorted(goals, key=lambda x:x.navigation_cost)):
            logger.log("Goal {} nav cost = {}".format(goal.identifier, goal.navigation_cost))
            goal.score += order * 2

        for order, goal in enumerate(sorted(goals, key=lambda x:x.weight, reverse=True)):
            goal.score += order

        logger.log("Goals by score : {}".format( ["{}:{}".format(g.identifier, g.score) for g in goals ] ))

        best_goal = min(goals, key=lambda g : g.score)

        logger.log("Best goal is {} with score {}".format(best_goal.identifier, best_goal.score))

        return best_goal


    def penalize_goal(self, goal):
        for g in self.harvesting_goals + self.emptying_goals:
            if g.identifier == goal.identifier:
                g.penality = 100.0


    def goal_done(self, goal):
        logger.log("Goal done : "+goal.identifier)
        self.harvesting_goals = [ g for g in self.harvesting_goals if g.identifier != goal.identifier ]
        self.emptying_goals = [ g for g in self.emptying_goals if g.identifier != goal.identifier ]
