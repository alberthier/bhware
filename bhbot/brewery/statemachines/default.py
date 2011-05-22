#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import world
import commonstates
from definitions import *



################################################################################
# Setup states


class Main(statemachine.State):

    def on_device_ready(self, team):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive())




class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_substate(commonstates.Deploy())


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoFirstIntersection())


################################################################################
# Place both first line pieces (Homologation)


class GotoFirstIntersection(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.forward(0.375)
        (p1_x, p1_y) = trajectory.Cell(0, 0).bottom_right()
        self.walk.look_at(p1_x, p1_y)
        self.walk.move_to(p1_x, p1_y)
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        if self.walk.exit_reason == TRAJECTORY_WALK_DESTINATION_REACHED:
            self.switch_to_state(GotoBottomIntersectionHeadFirst())
        elif self.walk.exit_reason == TRAJECTORY_WALK_PIECE_FOUND:
            self.event_loop.figure_detector.detected_at(1, trajectory.Cell(0, 0).bottom_right()[0])
            self.switch_to_state(GotoFirstIntersectionWithPiece())




class GotoFirstIntersectionWithPiece(statemachine.State):

    def on_enter(self):
        self.store_piece = commonstates.DirectStorePiece()

        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(*trajectory.Cell(0, 0).bottom_right())
        self.walk.look_at(*trajectory.Cell(0, 0).top_right())

        self.switch_to_substate(self.store_piece)


    def on_exit_substate(self, substate):
        if substate == self.store_piece:
            self.switch_to_substate(self.walk)
        else:
            self.switch_to_state(GotoBottomIntersectionBackFirst())




class GotoBottomIntersectionHeadFirst(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()
        sensor = self.robot().convert_sensor(SENSOR_LEFT_BOTTOM, TEAM_RED)
        self.sequence.add(self.event_loop.figure_detector.enable(sensor, 2))
        walk = commonstates.TrajectoryWalk()
        walk.backward(0.125)
        walk.look_at_opposite(*trajectory.Cell(0, 0).center_right())
        (p1_x, p1_y) = trajectory.Cell(0, 0).center_right()
        walk.move_to(p1_x, p1_y, DIRECTION_BACKWARD)
        walk.look_at(*trajectory.Cell(5, 0).top_right())
        walk.move_to(*trajectory.Cell(5, 0).top_right())
        self.sequence.add(walk)
        self.switch_to_substate(self.sequence)


    def on_exit_substate(self, substate):
        if substate == self.sequence:
            self.event_loop.figure_detector.detected_at(1, self.robot().pose.x + ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE)
            self.switch_to_substate(self.event_loop.figure_detector.disable())
        else:
            self.switch_to_state(GotoBottomIntersectionWithPiece())




class GotoBottomIntersectionWithPiece(statemachine.State):

    def on_enter(self):
        self.store_piece = commonstates.DirectStorePiece()

        self.walk = commonstates.TrajectoryWalk()
        self.walk.rotate_to(-math.pi / 2.0)
        self.walk.rotate_to(-math.pi)

        self.switch_to_substate(self.store_piece)


    def on_exit_substate(self, substate):
        if substate == self.store_piece:
            self.switch_to_substate(self.walk)
        else:
            self.switch_to_state(GotoBottomIntersectionBackFirst())




class GotoBottomIntersectionBackFirst(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()

        self.sequence.add(commonstates.OpenNippers())

        sensor = self.robot().convert_sensor(SENSOR_RIGHT_BOTTOM, TEAM_RED)
        self.sequence.add(self.event_loop.figure_detector.enable(sensor, 2))
        walk = commonstates.TrajectoryWalk()
        (dest_x, dest_y) = trajectory.Cell(5, 0).top_right()
        walk.move_to(dest_x, dest_y, DIRECTION_BACKWARD)
        walk.look_at(*trajectory.Cell(4, 1).top_right())
        walk.backward(0.100 * math.sqrt(2.0))
        walk.forward(0.100 * math.sqrt(2.0))

        self.sequence.add(walk)

        self.sequence.add(commonstates.CloseNippers())

        walk = commonstates.TrajectoryWalk()
        walk.look_at(*trajectory.Cell(3, 0).top_right())
        walk.move_to(*trajectory.Cell(3, 0).top_right())
        (p1_x, p1_y) = trajectory.Cell(4, 0).top_middle()
        p1_y -= 0.040
        walk.look_at_opposite(p1_x, p1_y)
        walk.move_to(p1_x, p1_y, DIRECTION_BACKWARD)
        walk.rotate_to(math.pi)
        walk.backward(FIELD_CELL_SIZE)
        self.sequence.add(walk)
        self.switch_to_substate(self.sequence)


    def on_exit_substate(self, substate):
        if substate == self.sequence:
            self.switch_to_substate(self.event_loop.figure_detector.disable())
        else:
            self.switch_to_state(ScanGreenZoneFigureConfiguration())




class ScanGreenZoneFigureConfiguration(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()
        sensor = self.robot().convert_sensor(SENSOR_LEFT_TOP, TEAM_RED)
        self.sequence.add(self.event_loop.figure_detector.enable(sensor, 0))
        walk = commonstates.TrajectoryWalk()
        walk.forward(FIELD_CELL_SIZE * 3.2)
        self.sequence.add(walk)
        self.switch_to_substate(self.sequence)


    def on_exit_substate(self, substate):
        if substate == self.sequence:
            self.switch_to_substate(self.event_loop.figure_detector.disable())
        else:
            self.switch_to_state(PlaceSecondPiece())




class PlaceSecondPiece(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()
        walk = commonstates.TrajectoryWalk()
        (dest_x, dest_y) = trajectory.Cell(0, 1).bottom_left()
        dest_x -= 0.060
        dest_y += 0.060
        walk.look_at(dest_x, dest_y)
        walk.move_to(dest_x, dest_y)
        self.sequence.add(walk)
        self.sequence.add(commonstates.ReleasePiece())
        walk = commonstates.TrajectoryWalk()
        walk.backward(0.250)
        self.sequence.add(walk)
        self.switch_to_substate(self.sequence)


    def on_exit_substate(self, substate):
        logger.log("'" + str(self.event_loop.figure_detector.elements) + "'")
        self.switch_to_state(TakeFirstFigure())




class TakeFirstFigure(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()
        walk = commonstates.TrajectoryWalk()
        figure_x = self.event_loop.figure_detector.get_first_match_x(0)
        first_line_y = trajectory.Cell(0, 0).top_right()[1]
        walk.look_at_opposite(figure_x, first_line_y)
        walk.move_to(figure_x, first_line_y, DIRECTION_BACKWARD)
        walk.rotate_to(-math.pi / 2.0)
        self.sequence.add(walk)
        self.sequence.add(commonstates.StorePiece1())
        walk = commonstates.TrajectoryWalk()
        walk.move_to(figure_x, 0.280)
        self.sequence.add(walk)
        self.sequence.add(commonstates.StorePiece2())
        walk = commonstates.TrajectoryWalk()
        walk.move_to(figure_x, first_line_y, DIRECTION_BACKWARD)
        self.sequence.add(walk)
        self.sequence.add(commonstates.StorePiece3())

        pieces_x = self.event_loop.figure_detector.get_green_zone_pawns_x()
        for piece_x in pieces_x[:-2]:
            walk = commonstates.TrajectoryWalk()
            walk.look_at(piece_x, first_line_y)
            walk.move_to(piece_x, first_line_y)
            walk.rotate_to(-math.pi / 2.0)
            self.sequence.add(walk)
            self.sequence.add(commonstates.StorePiece1())
            walk = commonstates.TrajectoryWalk()
            walk.move_to(piece_x, 0.280)
            self.sequence.add(walk)
            self.sequence.add(commonstates.StorePiece2())
            walk = commonstates.TrajectoryWalk()
            walk.move_to(piece_x, first_line_y, DIRECTION_BACKWARD)
            self.sequence.add(walk)
            self.sequence.add(commonstates.StorePiece3())
        self.switch_to_substate(self.sequence)


    def on_exit_substate(self, substate):
        self.switch_to_state(ReleaseConstruction())




class ReleaseConstruction(statemachine.State):

    def on_enter(self):
        self.sequence = commonstates.Sequence()
        walk = commonstates.TrajectoryWalk()
        walk.look_at(*trajectory.Cell(4, 2).top_middle())
        walk.move_to(*trajectory.Cell(4, 2).top_middle())
        walk.look_at(*trajectory.Cell(5, 2).top_middle())
        walk.move_to(*trajectory.Cell(5, 2).top_middle())
        self.sequence.add(walk)
        self.sequence.add(commonstates.ReleasePiece())
        walk = commonstates.TrajectoryWalk()
        walk.backward(0.5)
        self.sequence.add(walk)
        self.switch_to_substate(self.sequence)




class EndOfMatch(statemachine.State):

    def on_enter(self):
        logger.log("Did we won ?")







class TestHomologationWorld(statemachine.State):

    def on_enter(self):
        self.trajectory = world.world.get_trajectory()
        self.current = None
        self.next_move()


    def next_move(self):
        if self.current == None:
            if not len(self.trajectory) == 0:
                self.current = self.trajectory[0]
                self.trajectory = self.trajectory[1:]
                self.robot().look_at(self.current[0] / 1000.0, self.current[1] / 1000.0)
        else:
            self.robot().move_to(self.current[0] / 1000.0, self.current[1] / 1000.0)
            self.current = None


    def on_goto_finished(self, reason, pose):
        self.next_move()



class GotoFieldGenericGoto(statemachine.State):

    def on_enter(self):
        x = 1.0
        if self.robot().team == TEAM_RED:
            y = 1.3
            angle = -math.pi / 2.0
        elif self.robot().team == TEAM_BLUE:
            y = 3.0 - 1.3
            angle = math.pi / 2.0
        self.robot().goto(x, y, angle, DIRECTION_FORWARD)
        self.first = True


    def on_goto_finished(self, reason, pose):
        if self.first:
            self.first = False
            robot = self.robot()
            robot.goto(robot.pose.x, robot.pose.y, robot.pose.angle, DIRECTION_FORWARD)
        else:
            self.switch_to_state(GotoFieldRotate())



class GotoFieldMove(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldRotate())



class GotoFieldRotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 6.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove2())



class GotoFieldMove2(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.4)


    def on_goto_finished(self, reason, pose):
        if reason == REASON_PIECE_FOUND:
            self.switch_to_substate(commonstates.StorePiece1())
        else:
            self.switch_to_state(GotoFieldRotate2())


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoFieldMove3())



class GotoFieldMove3(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.2)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove4())



class GotoFieldMove4(statemachine.State):

    def on_enter(self):
        self.robot().backward(0.1)



class GotoFieldRotate2(statemachine.State):

    def on_enter(self):
        angle = math.pi / 3.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(GotoFieldMove2())



class Rotate(statemachine.State):

    def on_enter(self):
        angle = math.pi / 2.0
        if self.robot().team == TEAM_RED:
            angle = -angle
        self.robot().rotate(angle)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ1())


class RotateQ1(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x - 10.0, self.robot().pose.y - 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ2())


class RotateQ2(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x + 10.0, self.robot().pose.y - 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ3())


class RotateQ3(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x + 10.0, self.robot().pose.y + 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ4())


class RotateQ4(statemachine.State):

    def on_enter(self):
        self.robot().look_at(self.robot().pose.x - 10.0, self.robot().pose.y + 10.0)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(RotateQ1())


class Moving(statemachine.State):

    def on_enter(self):
        self.robot().forward(0.3)


    def on_goto_finished(self, reason, pose):
        self.switch_to_state(Rotate())
