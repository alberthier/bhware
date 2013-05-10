# encoding: utf-8

import sys
import position
import logger
import tools
import statemachine
import signals

from definitions import *

import itertools


class Goal(object):

    def __init__(self, identifier, weight, x, y, direction, handler_state, ctor_parameters = None, shared = True,
                 navigate = True):
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
        self.status = GOAL_AVAILABLE
        self.shared = shared
        self.navigate = navigate
        self.trial_count = 0

    def increment_trials(self):
        self.trial_count+=1
        logger.log('Goal {} : increment trials : {}'.format(self.identifier, self.trial_count))

    def get_state(self):
        if isinstance(self.handler_state, statemachine.State):
            return self.handler_state
        else :
            if self.ctor_parameters :
                return self.handler_state(self, *self.ctor_parameters)
            else:
                return self.handler_state(self)

    def available(self):
        self.status = GOAL_AVAILABLE

    def doing(self):
        self.status = GOAL_DOING

    def done(self):
        self.status = GOAL_DONE

    def is_available(self):
        return self.status == GOAL_AVAILABLE


class GlassDepositGoal(Goal):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.barmen = []

    def is_available(self):
        # logger.log('barmen : {} {}'.format(self.barmen, [b.glasses_count for b in self.barmen]))
        return any(b.glasses_count > 0 for b in self.barmen)


class GoalManager(object):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.harvesting_goals = []
        self.emptying_goals = []
        self.on_goal_state_change = signals.SafeSignal()
        self.last_goal = None


    @property
    def all_goals(self):
        return itertools.chain(self.harvesting_goals, self.emptying_goals)

    def next_goal(self):
        if self.event_loop.robot.tank_full:
            return self.get_best_goal(self.emptying_goals)
        else:
            return self.get_best_goal(self.harvesting_goals)

    def is_goal_available(self, identifier):
        return any((g.is_available() for g in self.all_goals if g.identifier == identifier))


    def get_best_goal(self, goals):
        """

        :type goals:  list of Goal
        """

        available_goals = [ g for g in goals if g.is_available() ]

        if len(available_goals) > 1 and self.last_goal in available_goals:
            available_goals.remove(self.last_goal)

        logger.log('available goals : {}'.format([g.identifier for g in available_goals]))

        if not available_goals :
            return None

        for goal in available_goals:
            pose = position.Pose(goal.x, goal.y, virtual=True)
            logger.log("Evaluate goal {}".format(goal.identifier))
            if GOAL_EVALUATION_USES_PATHFINDING:
                goal.navigation_cost = self.event_loop.eval_map.evaluate(self.event_loop.robot.pose, pose)
            else:
                goal.navigation_cost = tools.distance(self.event_loop.robot.pose.x, self.event_loop.robot.pose.y, pose.x, pose.y)
            goal.score = goal.penality
            goal.penality = 0.0

        for order, goal in enumerate(sorted(available_goals, key=lambda x:x.navigation_cost)):
            logger.log("Goal {} nav cost = {}".format(goal.identifier, goal.navigation_cost))
            goal.score += order * 2

        for order, goal in enumerate(sorted(available_goals, key=lambda x:x.weight, reverse=True)):
            goal.score += order

        for order, goal in enumerate(sorted(available_goals, key=lambda x:x.trial_count, reverse=True)):
            goal.score += order

        logger.log("available_goals by score : {}".format( ["{}:{}".format(g.identifier, g.score) for g in available_goals ] ))

        best_goal = min(available_goals, key=lambda g : g.score)

        logger.log("Best goal is {} with score {}".format(best_goal.identifier, best_goal.score))

        self.last_goal = best_goal

        return best_goal


    def penalize_goal(self, goal):
        for g in self.harvesting_goals + self.emptying_goals:
            if g.identifier == goal.identifier:
                g.penality = 100.0

    def goal_done(self, goal):
        self.update_goals_status(goal, "done", GOAL_DONE)

    def goal_doing(self, goal):
        self.update_goals_status(goal, "doing", GOAL_DOING)

    def goal_available(self, goal):
        self.update_goals_status(goal, "available", GOAL_AVAILABLE)

    def update_goals_status(self, goal, status_string, new_status):
        logger.log("Goal {} : {}".format(status_string, goal.identifier))

        self.internal_goal_update(goal.identifier, new_status)

        self.on_goal_state_change.send(goal)

    def internal_goal_update(self, identifier, status):
        for g in self.all_goals:
            if g.identifier == identifier:
                old = g.status
                g.status = status
                logger.log('updated goal {} status : {} -> {}'.format(identifier, old, status))
