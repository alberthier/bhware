# encoding: utf-8

import datetime
import itertools
import random
import sys

import logger
import packets
import position
import statemachine
import tools

from definitions import *




class Goal:

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
        self.last_try = None


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
        self.last_try = datetime.datetime.now()


    def done(self):
        self.status = GOAL_DONE


    def is_available(self):
        return self.status == GOAL_AVAILABLE




class GlassDepositGoal(Goal):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.barmen = []
        self.used = False


    def is_available(self):
        # logger.log('barmen : {} {}'.format(self.barmen, [b.glasses_count for b in self.barmen]))
        return not self.used and any(b.glasses_count > 0 for b in self.barmen)




class GoalManager:

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.harvesting_goals = []
        self.emptying_goals = []
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


    def get_least_recent_tried_goal(self):
        available_goals = [ g for g in self.all_goals if g.is_available() ]

        if not available_goals :
            return None

        goals = None

        never_tried = [ g for g in available_goals if g.last_try is None ]

        if never_tried :
            goals = never_tried

        else :
            max_date = max([g.last_try for g in available_goals if g.last_try])

            goals = [ g for g in available_goals if g.last_try == max_date ]

        if len(goals) == 1 :
            return  goals[0]
        else :
            return random.choice(goals)


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

        if goal.shared :
            logger.log('A shared goal status changed, notifying my buddy : {} -> {}'.format(goal.identifier, goal.status))
            packet = packets.InterbotGoalStatus(goal_identifier = goal.identifier, goal_status = goal.status)
            self.event_loop.send_packet(packet)


    def internal_goal_update(self, identifier, status):
        for g in self.all_goals:
            if g.identifier == identifier:
                old = g.status
                g.status = status
                logger.log('updated goal {} status : {} -> {}'.format(identifier, old, status))


    def on_interbot_goal_status(self, packet):
        logger.log('Got goal status : {} = {}'.format(packet.goal_identifier, packet.goal_status))
        self.internal_goal_update(packet.goal_identifier, packet.goal_status)
