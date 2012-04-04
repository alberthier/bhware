    #!/usr/bin/env python
# encoding: utf-8

import sys




class Goal(object):

    def __init__(self, count, weight, x, y, handler_state):
        self.count = count
        self.weight = weight
        self.x = x
        self.y = y
        self.handler_state = handler_state
        self.intrinsic_cost = 0.0
        self.navigation_cost = 0.0




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


    def get_best_goal(self, goals):
        robot_x = self.event_loop.robot.pose.x
        robot_y = self.event_loop.robot.pose.y

        # normalize intrinsic cost
        intrinsic_max_weight = float(max(goals, key = lambda g: g.weight))
        for goal in goals:
            goal.intrinsic_cost = 1.0 - float(goal.weight) / intrinsic_max_weight

        # normalize navigation cost
        max_navigation_cost = 0
        for goal in goals:
            goal.navigation_cost = self.event_loop.map.evaluate_cost(robot_x, robot_y, goal.x, goal.y)
            if goal.navigation_cost > max_navigation_cost:
                max_navigation_cost = goal.navigation_cost
        for goal in goals:
            goal.navigation_weight = 1.0 - float(goal.navigation_weight) / float(max_navigation_cost)

        best_goal = None
        best_cost = float(sys.maxint)
        for goal in goals:
            goal_cost = goal.normalized_weight + goal.navigation_weight
            if goal_cost < best_cost:
                best_cost = goal_cost
                best_goal = goal

        return best_goal


    def goal_done(self, goal):
        if goal in self.harvesting_goals:
            self.harvesting_goals.remove(goal)
        if goal in self.emptying_goals:
            self.emptying_goals.remove(goal)
