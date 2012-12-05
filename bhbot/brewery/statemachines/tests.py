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

        trigo = False
        start_poses = [
                       trajectory.Pose(1.0, 1.0,  0.0),
                       trajectory.Pose(1.0, 1.0,  3.0 * math.pi / 4.0),
                       trajectory.Pose(1.0, 1.0,  math.pi),
                       trajectory.Pose(1.0, 1.0, -math.pi / 2)
                      ]

        for start_pose in start_poses:
            # Line moves
            self.add_goto_test(TestMove(DIRECTION_FORWARD , 1.0, start_pose), seq, start_pose)
            self.add_goto_test(TestMove(DIRECTION_BACKWARD, 1.0, start_pose), seq, start_pose)

            self.add_goto_test(TestForwardBackward(1.0, start_pose), seq, start_pose)

            # Rotation tests
            self.add_goto_test(TestRotation(      3.0 *math.pi / 4.0 , trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestRotation(      math.pi       , trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestRotation(3.0 * math.pi / 2.0 , trigo, start_pose), seq, start_pose)

            self.add_goto_test(TestCircleRotation(8, trigo, start_pose), seq, start_pose)

            # Go forward - rotate - go forward - rotate
            self.add_goto_test(TestMoveAndReturn(DIRECTION_FORWARD, 1.0, start_pose), seq, start_pose)

            # Square
            self.add_goto_test(TestSquare(DIRECTION_BACKWARD, 1.0,     trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestSquare(DIRECTION_BACKWARD, 1.0, not trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestSquare(DIRECTION_FORWARD , 1.0,     trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestSquare(DIRECTION_FORWARD , 1.0, not trigo, start_pose), seq, start_pose)

            # Multipoint 'M' shape
            self.add_goto_test(TestMultipointMShape(DIRECTION_FORWARD , 1.0, start_pose), seq, start_pose)
            self.add_goto_test(TestMultipointMShape(DIRECTION_BACKWARD, 1.0, start_pose), seq, start_pose)

            # Multipoint Square
            self.add_goto_test(TestMultipointSquare(DIRECTION_FORWARD , 1.0, trigo, start_pose), seq, start_pose)
            self.add_goto_test(TestMultipointSquare(DIRECTION_BACKWARD, 1.0, trigo, start_pose), seq, start_pose)

            self.add_goto_test(TestTakeTreasure(start_pose), seq, start_pose)

        seq.add(TestLookAt())
        seq.add(TestMultipointCircle(DIRECTION_FORWARD, 0.4))

        # Actuators
        seq.add(TestGripper(GRIPPER_SIDE_LEFT))
        seq.add(TestGripper(GRIPPER_SIDE_RIGHT))
        seq.add(TestGripper(GRIPPER_SIDE_BOTH))

        seq.add(TestSweeper())

        seq.add(TestMapArm())

        seq.add(TestGoldBarDetection())

        seq.add(TestOpponentDetection())

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
        walk.goto(dst.x, dst.y, -math.pi / 2.0, self.direction)
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
        for i in range(self.steps):
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
        for i in range(4):
            dst = get_destination_pose(self.direction, self.distance, math.pi / 2.0, self.trigo, dst)
            walk.rotate_to(dst.angle)
            walk.move_to(dst.x, dst.y)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()

class TestSquare2(statemachine.State):

    def __init__(self):
        pass

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
#  walk.look_at(2.0, 1.0)
        walk.move_to(2.0, 1.0)
        walk.look_at(2.0, 2.0)
        walk.move_to(2.0, 2.0)
        walk.look_at(1.0, 2.0)
        walk.move_to(1.0, 2.0)
        walk.look_at(1.0, 1.0)
        walk.move_to(1.0, 1.0)
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
        for i in range(4):
            points.append(get_destination_pose(self.direction, self.distance, math.pi / 2.0, self.trigo, points[-1]))

        walk.follow(points[1:], self.direction)

        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()




class TestLookAt(statemachine.State):

    def on_enter(self):
        self.switch_to_substate(commonstates.DefinePosition(1.0, 1.0, math.pi))


    def on_exit_substate(self, substate):
        if isinstance(substate, commonstates.DefinePosition):
            walk = commonstates.TrajectoryWalk()
            n = 8
            for i in range(1, n + 1):
                a = float(i) * 2.0 * math.pi / float(n)
                x = 1.0 + math.cos(a) * 0.5
                y = 1.0 + math.sin(a) * 0.5
                walk.look_at_opposite(x, y)
            self.switch_to_substate(walk)
        else:
            self.exit_substate()




class TestMultipointCircle(statemachine.State):

    def __init__(self, direction, radius):
        self.direction  = direction
        self.radius     = radius


    def on_enter(self):
        if self.direction == DIRECTION_FORWARD:
            angle = math.pi / 2.0
        else:
            angle = -math.pi / 2.0
        self.switch_to_substate(commonstates.DefinePosition(1.0 + self.radius, 1.0, angle))


    def on_exit_substate(self, substate):
        if isinstance(substate, commonstates.DefinePosition):
            walk = commonstates.TrajectoryWalk()

            points = []
            n = 20
            for i in range(1, n + 1):
                a = float(i) * 2.0 * math.pi / float(n)
                x = 1.0 + math.cos(a) * self.radius
                y = 1.0 + math.sin(a) * self.radius
                points.append(trajectory.Pose(x, y))

#  walk.rotate_to(0.746324465867)
            walk.look_at(points[0].x, points[0].y)
            walk.follow(points, None, self.direction)

            self.switch_to_substate(walk)
        else:
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



class TestTakeTreasure(statemachine.State):

    def __init__(self, start_pose):
        self.start_pose = start_pose


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        dst = get_destination_pose(DIRECTION_FORWARD, 1.0, 0.0, True, self.start_pose)
        walk.move_to(dst.x, dst.y, DIRECTION_FORWARD)
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
        dst = get_destination_pose(DIRECTION_BACKWARD, 1.0, 0.0, True, dst)
        walk.move_to(dst.x, dst.y, DIRECTION_BACKWARD)
        walk.rotate_to(math.pi / 2.0)
        walk.move_to(1.0, 1.1, DIRECTION_FORWARD)
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.move_to(1.0, 1.0, DIRECTION_BACKWARD)
        walk.rotate_to(0.0)
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()

class MakeRobotsCollide(statemachine.State):
    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.backward(2.5)
        self.switch_to_substate(walk)


class TestTrajectory(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk(None, TEAM_UNKNOWN)
        points = self.event_loop.map.route(self.robot().pose, trajectory.Pose(1.6, 2.0, None, True))
        x = self.robot().pose.x
        y = self.robot().pose.y
        logger.log("{}".format(self.robot().team))
        for p in points:
            x = p.x
            y = p.y
            walk.look_at_opposite(x, y)
            walk.move_to(x, y, DIRECTION_BACKWARD)
        g = packets.Goto()
        g.direction = DIRECTION_BACKWARD
        g.points = points
        #self.send_packet(g)
        self.switch_to_substate(walk)




class TestCommands(statemachine.State):

    def on_enter(self):
        seq = commonstates.Sequence()

        #        seq.add(commonstates.StoreFabric(FABRIC_STORE_LOW))
        #        seq.add(commonstates.MapArm(MAP_ARM_OPEN))
        #        seq.add(commonstates.MapGripper(MAP_GRIPPER_OPEN))
        #        seq.add(commonstates.MapGripper(MAP_GRIPPER_CLOSE))
        #        seq.add(commonstates.MapArm(MAP_ARM_CLOSE))
        #        seq.add(commonstates.StoreFabric(FABRIC_STORE_HIGH))
        #
        #        seq.add(commonstates.Gripper(GRIPPER_SIDE_LEFT, GRIPPER_OPEN))
        #        seq.add(commonstates.Gripper(GRIPPER_SIDE_RIGHT, GRIPPER_OPEN))
        #        seq.add(commonstates.Gripper(GRIPPER_SIDE_LEFT, GRIPPER_CLOSE))
        #        seq.add(commonstates.Gripper(GRIPPER_SIDE_RIGHT, GRIPPER_CLOSE))
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
        #        seq.add(commonstates.EmptyTank(TANK_DEPLOY))
        #        seq.add(commonstates.EmptyTank(TANK_RETRACT))

        #        seq.add(commonstates.Sweeper(SWEEPER_OPEN))
        #        seq.add(commonstates.Sweeper(SWEEPER_CLOSE))


        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.switch_to_state(TestSimpleSquare())


class GotoStartPoint(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(PURPLE_START_X, 0.59)
        if self.robot().team == TEAM_PURPLE:
            self.walk.rotate_to(0.0)
            self.walk.move_to(0.96, 0.59)
        else:
            self.walk.rotate_to(math.pi)
            self.walk.move_to(1.04, 0.59)
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(LoopAroundPeanutIsland())




class LoopAroundPeanutIsland(statemachine.State):

    def on_enter(self):
        points = [(1.50, 1.06,  math.pi / 2.0),
            (1.40, 1.46,  math.pi / 2.0),
            (1.50, 1.86,  math.pi / 2.0),
            (1.04, 2.41,  math.pi      ),
            (0.50, 1.86, -math.pi / 2.0),
            (0.60, 1.46, -math.pi / 2.0),
            (0.50, 1.06, -math.pi / 2.0),
            (0.96, 0.59,  0.0          )]

        if self.robot().team == TEAM_RED:
            points.reverse()
            points = points[1:] + points[:1]
            angle_offset = math.pi
        else:
            angle_offset = 0.0

        self.walk = commonstates.TrajectoryWalk()

        for i in range(len(points) * 3):
            k = i % len(points)
            (x, y, angle) = points[k]
            self.walk.goto(x, y, angle + angle_offset)

        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoCaptainRoom())