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
        p0 = trajectory.Pose(self.robot().pose.x, 0.400)
        p1 = trajectory.Pose(*trajectory.Cell(0, 0).bottom_right())
        p1.angle = p0.look_at(p1) - math.pi / 8.0

        self.walk = commonstates.TrajectoryWalk()
        self.walk.follow([p0, p1])
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
        self.walk = commonstates.TrajectoryWalk()
        sensor = self.robot().convert_sensor(SENSOR_LEFT_BOTTOM, TEAM_RED)
        self.walk.wait_for(self.event_loop.figure_detector.enable(sensor, 2))
        self.walk.look_at(*trajectory.Cell(5, 0).top_right())
        self.walk.move_to(*trajectory.Cell(5, 0).top_right())
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        if substate == self.walk:
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
        walk = commonstates.TrajectoryWalk()
        walk.wait_for(commonstates.OpenNippers())
        sensor = self.robot().convert_sensor(SENSOR_RIGHT_BOTTOM, TEAM_RED)
        walk.wait_for(self.event_loop.figure_detector.enable(sensor, 2))
        (dest_x, dest_y) = trajectory.Cell(5, 0).top_right()
        walk.move_to(dest_x, dest_y, DIRECTION_BACKWARD)
        walk.wait_for(self.event_loop.figure_detector.disable())
        walk.look_at(*trajectory.Cell(4, 1).top_right())
        walk.backward(0.100 * math.sqrt(2.0))
        walk.forward(0.100 * math.sqrt(2.0))
        walk.wait_for(commonstates.CloseNippers())
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(ReleaseFirstPieceOnBonusCell())




class ReleaseFirstPieceOnBonusCell(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        (p0_x, p0_y) = trajectory.Cell(4, 1).center_right()
        walk.goto(p0_x, p0_y, math.pi / 2.0)
        (p1_x, p1_y) = trajectory.Cell(4, 2).center_middle()
        p1_x += 0.100
        p1_y += 0.030
        walk.goto(p1_x, p1_y, 0.0)
        walk.wait_for(commonstates.ReleasePiece())
        walk.move_to(p1_x - 0.130, p1_y)
        (p1_x, p1_y) = trajectory.Cell(5, 0).top_middle()
        p1_y -= 0.040
        walk.goto(p1_x, p1_y, math.pi)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(ScanGreenZoneFigureConfiguration())




class ScanGreenZoneFigureConfiguration(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        sensor = self.robot().convert_sensor(SENSOR_LEFT_TOP, TEAM_RED)
        walk.wait_for(self.event_loop.figure_detector.enable(sensor, 0))
        (p0_x, p0_y) = trajectory.Cell(1, 0).bottom_middle()
        p0_x -= 0.070
        p0_y -= 0.040
        walk.move_to(p0_x, p0_y)
        walk.wait_for(self.event_loop.figure_detector.disable())
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        logger.log("'" + str(self.event_loop.figure_detector.elements) + "'")
        self.switch_to_state(TakeFirstFigure())




class TakeFirstFigure(statemachine.State):

    def on_enter(self):
        # If the only available figure is the last one, don't take it
        walk = commonstates.TrajectoryWalk()
        figure_x = self.event_loop.figure_detector.get_first_match_x(0)
        first_line_y = trajectory.Cell(0, 0).top_right()[1]
        walk.goto(figure_x, first_line_y, -math.pi / 2.0)
        walk.wait_for(commonstates.StorePiece1())
        walk.move_to(figure_x, 0.250)
        walk.wait_for(commonstates.StorePiece2())
        walk.move_to(figure_x, first_line_y, DIRECTION_BACKWARD)
        walk.wait_for(commonstates.StorePiece3())

        pieces_x = self.event_loop.figure_detector.get_green_zone_pawns_x()
        for piece_x in pieces_x[:-2]:
            walk.look_at(piece_x, first_line_y)
            walk.move_to(piece_x, first_line_y)
            walk.rotate_to(-math.pi / 2.0)
            walk.wait_for(commonstates.StorePiece1())
            walk.move_to(piece_x, 0.250)
            walk.wait_for(commonstates.StorePiece2())
            walk.move_to(piece_x, first_line_y, DIRECTION_BACKWARD)
            walk.wait_for(commonstates.StorePiece3())
        self.switch_to_substate(walk)


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
