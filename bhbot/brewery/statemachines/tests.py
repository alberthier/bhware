#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import commonstates

from definitions import *




class Main(statemachine.State):

    def on_enter(self):
        logger.log("Waiting for the start signal")

    def on_start(self, packet):
        seq = commonstates.Sequence()

        trigo = True
        start_poses = [
                       trajectory.Pose(1.0, 1.0,  0.0),
                       trajectory.Pose(1.0, 1.0,  math.pi / 2.0),
                       trajectory.Pose(1.0, 1.0,  math.pi),
                       trajectory.Pose(1.0, 1.0, -math.pi / 2)
                      ]

        for start_pose in start_poses:
            # Line moves
            #self.add_goto_test(TestMove(DIRECTION_FORWARD , 1.0, start_pose), seq, start_pose)
            #self.add_goto_test(TestMove(DIRECTION_BACKWARD, 1.0, start_pose), seq, start_pose)

            self.add_goto_test(TestForwardBackward(1.0, start_pose), seq, start_pose)

            # Rotation tests
            #self.add_goto_test(TestRotation(      math.pi / 2.0 , trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestRotation(      math.pi       , trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestRotation(3.0 * math.pi / 2.0 , trigo, start_pose), seq, start_pose)

            #self.add_goto_test(TestCircleRotation(8, trigo, start_pose), seq, start_pose)

            # Go forward - rotate - go forward - rotate
            #self.add_goto_test(TestMoveAndReturn(DIRECTION_FORWARD, 1.0, start_pose), seq, start_pose)

            # Square
            #self.add_goto_test(TestSquare(DIRECTION_BACKWARD, 1.0,     trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestSquare(DIRECTION_BACKWARD, 1.0, not trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestSquare(DIRECTION_FORWARD , 1.0,     trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestSquare(DIRECTION_FORWARD , 1.0, not trigo, start_pose), seq, start_pose)

            # Multipoint 'M' shape
            #self.add_goto_test(TestMultipointMShape(DIRECTION_FORWARD , 1.0, start_pose), seq, start_pose)
            #self.add_goto_test(TestMultipointMShape(DIRECTION_BACKWARD, 1.0, start_pose), seq, start_pose)

            # Multipoint Square
            #self.add_goto_test(TestMultipointSquare(DIRECTION_FORWARD , 1.0, trigo, start_pose), seq, start_pose)
            #self.add_goto_test(TestMultipointSquare(DIRECTION_BACKWARD, 1.0, trigo, start_pose), seq, start_pose)

        # Actuators
        #seq.add(TestGripper(GRIPPER_SIDE_LEFT))
        #seq.add(TestGripper(GRIPPER_SIDE_RIGHT))
        #seq.add(TestGripper(GRIPPER_SIDE_BOTH))

        #seq.add(TestSweeper())

        #seq.add(TestMapArm())

        #seq.add(TestGoldBarDetection())

        #seq.add(TestOpponentDetection())

        self.switch_to_substate(seq)


    def add_goto_test(self, test, seq, start_pose):
        seq.add(commonstates.DefinePosition(start_pose.x, start_pose.y, start_pose.angle))
        seq.add(test)


    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())



##############################################################################################################




def get_destination_pose(direction, distance, delta_angle, trigo, start_pose):
    result = trajectory.Pose(start_pose.x, start_pose.y, start_pose.angle)
    if trigo:
        result.angle += delta_angle
    else:
        result.angle -= delta_angle
    result.x += math.cos(result.angle) * direction * distance
    result.y += math.sin(result.angle) * direction * distance

    return result




class TestMove(statemachine.State):

    def __init__(self, direction, distance, start_pose):
        self.direction  = direction
        self.distance   = distance
        self.start_pose = start_pose


    def on_enter(self):
        dst = get_destination_pose(self.direction, self.distance, 0.0, True, self.start_pose)
        walk = commonstates.TrajectoryWalk()
        walk.move_to(dst.x, dst.y, self.direction)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestForwardBackward(statemachine.State):

    def __init__(self, distance, start_pose):
        self.distance   = distance
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        dst = get_destination_pose(DIRECTION_FORWARD, self.distance, 0.0, True, self.start_pose)
        walk.move_to(dst.x, dst.y, DIRECTION_FORWARD)
        dst = get_destination_pose(DIRECTION_BACKWARD, self.distance, 0.0, True, dst)
        walk.move_to(dst.x, dst.y, DIRECTION_BACKWARD)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestRotation(statemachine.State):

    def __init__(self, delta_angle, trigo, start_pose):
        self.delta_angle = delta_angle
        self.trigo       = trigo
        self.start_pose  = start_pose


    def on_enter(self):
        dst = get_destination_pose(DIRECTION_FORWARD, 0.0, self.delta_angle, self.trigo, self.start_pose)
        walk = commonstates.TrajectoryWalk()
        walk.rotate_to(dst.angle)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestCircleRotation(statemachine.State):

    def __init__(self, steps, trigo, start_pose):
        self.steps = steps
        self.trigo = trigo
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        for i in xrange(self.steps):
            dst = get_destination_pose(DIRECTION_FORWARD, 0.0, float(i + 1) * math.pi * 2.0 / float(self.steps), self.trigo, self.start_pose)
            walk.rotate_to(dst.angle)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestMoveAndReturn(statemachine.State):

    def __init__(self, direction, distance, start_pose):
        self.direction  = direction
        self.distance   = distance
        self.start_pose = start_pose


    def on_enter(self):
        dst = get_destination_pose(self.direction, self.distance, 0.0, True, self.start_pose)
        walk = commonstates.TrajectoryWalk()
        walk.move_to(dst.x, dst.y, self.direction)
        dst = get_destination_pose(DIRECTION_FORWARD, 0.0, math.pi, True, self.start_pose)
        walk.rotate_to(dst.angle)
        walk.move_to(self.start_pose.x, self.start_pose.y, self.direction)
        walk.rotate_to(self.start_pose.angle)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestSquare(statemachine.State):

    def __init__(self, direction, distance, trigo, start_pose):
        self.direction  = direction
        self.distance = distance
        if trigo:
            self.trigo  =  1
        else:
            self.trigo  = -1
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        dst = trajectory.Pose(self.start_pose.x, self.start_pose.y, self.start_pose.angle)
        for i in xrange(4):
            dst = get_destination_pose(self.direction, self.distance, math.pi / 2.0, self.trigo, dst)
            walk.rotate_to(dst.angle)
            walk.move_to(dst.x, dst.y)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestMultipointMShape(statemachine.State):

    def __init__(self, direction, distance, start_pose):
        self.direction  = direction
        self.distance = distance
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        dst = trajectory.Pose(self.start_pose.x, self.start_pose.y, self.start_pose.angle)

        points = []

        points.append(get_destination_pose(self.direction, self.distance      ,           0.0      , True, self.start_pose))
        points.append(get_destination_pose(self.direction, self.distance / 2.0, 3.0 * math.pi / 4.0, True, points[-1]))
        points.append(get_destination_pose(self.direction, self.distance / 2.0,      -math.pi / 2.0, True, points[-1]))
        points.append(get_destination_pose(self.direction, self.distance      , 3.0 * math.pi / 4.0, True, points[-1]))

        walk.follow(points, self.direction)

        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestMultipointSquare(statemachine.State):

    def __init__(self, direction, distance, trigo, start_pose):
        self.direction  = direction
        self.distance   = distance
        self.trigo      = trigo
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()

        points = [ self.start_pose ]
        for i in xrange(4):
            points.append(get_destination_pose(self.direction, self.distance, math.pi / 2.0, self.trigo, points[-1]))

        walk.follow(points[1:], self.direction)

        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestGripper(statemachine.State):

    def __init__(self, side):
        self.side = side


    def on_enter(self):
        seq = commonstates.Sequence()
        seq.add(commonstates.Gripper(self.side, GRIPPER_OPEN))
        seq.add(commonstates.Gripper(self.side, GRIPPER_CLOSE))
        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestSweeper(statemachine.State):

    def on_enter(self):
        seq = commonstates.Sequence()
        seq.add(commonstates.Sweeper(SWEEPER_OPEN))
        seq.add(commonstates.Sweeper(SWEEPER_CLOSE))
        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestMapArm(statemachine.State):

    def on_enter(self):
        seq = commonstates.Sequence()
        seq.add(commonstates.MapArm(MAP_ARM_OPEN))
        seq.add(commonstates.MapGripper(MAP_GRIPPER_OPEN))
        seq.add(commonstates.MapGripper(MAP_GRIPPER_CLOSE))
        seq.add(commonstates.MapArm(MAP_ARM_CLOSE))
        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestTank(statemachine.State):

    def on_enter(self):
        seq = commonstates.Sequence()
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        seq.add(commonstates.EmptyTank(TANK_DEPLOY))
        seq.add(commonstates.EmptyTank(TANK_RETRACT))
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestGoldBarDetection(statemachine.State):

    def on_enter(self):
        self.switch_to_substate(commonstates.GetGoldBarStatus())


    def on_exit_substate(self, substate):
        logger.log("Gold bar: {}".format(substate.status))
        self.exit_substate()




class TestOpponentDetection(statemachine.State):

    def on_enter(self):
        pass


    def on_exit_substate(self, substate):
        self.exit_substate()
