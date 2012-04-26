    #!/usr/bin/env python
# encoding: utf-8

import sys
import trajectory
import logger




class Goal(object):

    def __init__(self, identifier, weight, x, y, direction, handler_state, ctor_parameters = None):
        self.identifier = identifier
#        self.count = count
        self.weight = weight
        self.x = x
        self.y = y
        self.direction = direction
        self.handler_state = handler_state
#        self.intrinsic_cost = 0.0
        self.navigation_cost = 0.0
#        self.navigation_weight = 0.0
#        self.normalized_weight = 0.0
        self.ctor_parameters = ctor_parameters
        self.score = 0.0




class GoalManager(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.harvesting_goals = []
        self.emptying_goals = []

    def next_goal(self):
        if self.event_loop.robot.tank_full:
            return self.get_best_goal(self.emptying_goals)
        else:
            return self.get_best_goal(self.harvesting_goals)


#    def get_best_goal(self, goals):
#        #TODO : sort by distance cost, order -> note1, order by weight -> note2 then cost = note1+note2
#
#        robot_x = self.event_loop.robot.pose.x
#        robot_y = self.event_loop.robot.pose.y
#
#        # normalize intrinsic cost
#        intrinsic_max_weight = float((max(goals, key = lambda g: g.weight)).weight)
#        for goal in goals:
#            goal.intrinsic_cost = 1.0 - float(goal.weight) / intrinsic_max_weight
#
#        # normalize navigation cost
#        max_navigation_cost = 0
#        for goal in goals:
#            pose = trajectory.Pose(goal.x, goal.y, virtual=True)
#            goal.navigation_cost = self.event_loop.map.evaluate(self.event_loop.robot.pose, pose)
#            if goal.navigation_cost > max_navigation_cost:
#                max_navigation_cost = goal.navigation_cost
#        for goal in goals:
##            goal.navigation_weight = 1.0 - float(goal.navigation_weight) / float(max_navigation_cost)
#            goal.navigation_weight = goal.navigation_cost
#
#        best_goal = None
#        best_cost = float(sys.maxint)
#        for goal in goals:
#            goal_cost = goal.normalized_weight + goal.navigation_weight
#            if goal_cost < best_cost:
#                best_cost = goal_cost
#                best_goal = goal
#
#        return best_goal

    def get_best_goal(self, goals):

        if not goals :
            return None

        for goal in goals:
            pose = trajectory.Pose(goal.x, goal.y, virtual=True)
            goal.navigation_cost = self.event_loop.map.evaluate(self.event_loop.robot.pose, pose)

        for order, goal in enumerate(sorted(goals, key=lambda x:x.navigation_cost)):
            goal.score = order

        for order, goal in enumerate(sorted(goals, key=lambda x:x.weight, reverse=True)):
            goal.score += order

        logger.log("Goals by score : {}".format( ["{}:{}".format(g.identifier, g.score) for g in goals ] ))

        best_goal = min(goals, key=lambda g : g.score)

        logger.log("Best goal is {} with score {}".format(best_goal.identifier, best_goal.score))

        return best_goal


    def goal_done(self, goal):
        logger.log("Goal done : "+goal.identifier)
        self.harvesting_goals = [ g for g in self.harvesting_goals if g.identifier != goal.identifier ]
        self.emptying_goals = [ g for g in self.emptying_goals if g.identifier != goal.identifier ]
