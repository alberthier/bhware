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
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
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
        self.walk.wait_for(commonstates.EnableFigureDetector(sensor, 2))
        self.walk.look_at(*trajectory.Cell(5, 0).top_right())
        self.walk.move_to(*trajectory.Cell(5, 0).top_right())
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        if substate == self.walk:
            self.event_loop.figure_detector.detected_at(1, self.robot().pose.x + ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE)
            self.switch_to_substate(commonstates.DisableFigureDetector(False))
        else:
            self.switch_to_state(GotoBottomIntersectionWithPiece())




class GotoBottomIntersectionWithPiece(statemachine.State):

    def on_enter(self):
        self.store_piece = commonstates.DirectStorePiece()

        self.walk = commonstates.TrajectoryWalk()
        self.walk.rotate_to(math.radians(-178))

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
        walk.wait_for(commonstates.EnableFigureDetector(sensor, 2))
        (dest_x, dest_y) = trajectory.Cell(5, 0).top_right()
        dest_x -= 0.120 # Safe zone black area width
        walk.move_to(dest_x, dest_y, DIRECTION_BACKWARD)
        walk.wait_for(commonstates.DisableFigureDetector(True))
        walk.rotate_to(2.0 * math.pi/3)
        # Check: the rotation is enough to push the pawn on the cell
        #walk.backward(0.075 * math.sqrt(2.0))
        #walk.forward(0.075 * math.sqrt(2.0))
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
        walk.backward(0.22)
        walk.rotate_to(math.radians(80))
        (p1_x, p1_y) = trajectory.Cell(5, 0).top_middle()
        p1_y -= 0.040
        walk.goto(p1_x, p1_y, 0.0, DIRECTION_BACKWARD)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(ScanGreenZoneFigureConfiguration())




class ScanGreenZoneFigureConfiguration(statemachine.State):

    def on_enter(self):
        self.detector_disabler = commonstates.DisableFigureDetector(True)
        self.walk = commonstates.TrajectoryWalk()
        self.walk.on_piece_detected = self.on_piece_detected
        self.sensor = self.robot().convert_sensor(SENSOR_LEFT_TOP, TEAM_RED)
        self.walk.wait_for(commonstates.EnableFigureDetector(self.sensor, 0))
        (p0_x, p0_y) = trajectory.Cell(1, 0).bottom_middle()
        p0_x -= 0.070
        p0_y -= 0.040
        self.walk.move_to(p0_x, p0_y)
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        if substate == self.walk:
            # We are at the end of the green zone. Only 0 or 1 figure detected
            self.detection_finished()
        else:
            self.switch_to_state(TakeFirstFigure())


    def on_piece_detected(self, start_pose, start_distance, end_pose, end_distance, sensor, angle):
        if sensor == self.sensor and self.event_loop.figure_detector.get_elements_count(0) == 2:
            # Two figures detected. Stop here.
            self.robot().stop()
            self.detection_finished()


    def detection_finished(self):
        logger.log("'" + str(self.event_loop.figure_detector.elements) + "'")
        self.switch_to_substate(self.detector_disabler)



class TakeFirstFigure(statemachine.State):

    def on_enter(self):
        figure_x = self.event_loop.figure_detector.pop_nearest_match_x(0)

        if figure_x != None:
            walk = commonstates.TrajectoryWalk()
            first_line_y = trajectory.Cell(0, 0).top_right()[1]
            walk.goto(figure_x, first_line_y, -math.pi / 2.0)
            walk.wait_for(commonstates.StorePiece1())
            walk.move_to(figure_x, 0.270)
            walk.wait_for(commonstates.StorePiece2())
            walk.move_to(figure_x, first_line_y, DIRECTION_BACKWARD)
            walk.wait_for(commonstates.StorePiece3())
            self.switch_to_substate(walk)
        else:
            self.switch_to_state(TakeFirstFigurePawn())


    def on_exit_substate(self, substate):
        if substate.exit_reason == TRAJECTORY_WALK_PIECE_FOUND:
            # Skip current destination and continue the walk
            substate.current_goto_packet = None
            self.switch_to_substate(substate)
        else:
            self.switch_to_state(TakeFirstFigurePawn())




class TakeFirstFigurePawn(statemachine.State):

    def on_enter(self):
        # If the only available figure is the last one, don't take it
        walk = commonstates.TrajectoryWalk()
        pawn_x = self.event_loop.figure_detector.pop_nearest_green_zone_pawn_x()
        first_line_y = trajectory.Cell(0, 0).top_right()[1]
        walk.look_at(pawn_x, first_line_y)
        walk.move_to(pawn_x, first_line_y)
        walk.rotate_to(-math.pi / 2.0)
        walk.wait_for(commonstates.StorePiece1())
        walk.move_to(pawn_x, 0.270)
        walk.wait_for(commonstates.StorePiece2())
        walk.move_to(pawn_x, first_line_y, DIRECTION_BACKWARD)
        walk.wait_for(commonstates.StorePiece3())
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        if substate.exit_reason == TRAJECTORY_WALK_PIECE_FOUND:
            # Skip current destination and continue the walk
            substate.current_goto_packet = None
            self.switch_to_substate(substate)
        else:
            self.switch_to_state(TakeConstruction())


class TakeConstruction(statemachine.State):

    def on_enter(self):
        sequence = commonstates.Sequence()

        sequence.add(commonstates.ReleasePiece())

        walk = commonstates.TrajectoryWalk()
        walk.backward(0.070)
        sequence.add(walk)

        sequence.add(commonstates.StorePiece1())

        walk = commonstates.TrajectoryWalk()
        walk.forward(0.100)
        sequence.add(walk)

        sequence.add(commonstates.StorePiece2())
        sequence.add(commonstates.StorePiece3())

        self.switch_to_substate(sequence)


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
