#!/usr/bin/env python
# encoding: utf-8


import statemachine
import packets
from definitions import *



class Timer(statemachine.State):

    def __init__(self, miliseconds):
        statemachine.State.__init__(self)
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, miliseconds)


    def on_keep_alive(self, current_pose, match_started, match_time):
        if datetime.datetime.now() > self.end_time:
            self.exit_substate()




class Sequence(statemachine.State):

    def __init__(self, *args):
        statemachine.State.__init__(self)
        self.substates = deque(args)


    def add(self, substate):
        self.substates.append(substate)


    def on_enter(self):
        self.process_next_substate()


    def on_exit_substate(self):
        self.process_next_substate()


    def process_next_substate(self):
        if len(self.substates) == 0:
            self.exit_substate()
        else:
            self.switch_to_substate(self.substates.popleft())




class SetupPositionControl(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_configured(self):
        self.exit_substate()




class DefinePosition(statemachine.State):

    def __init__(self, x, y, angle):
        statemachine.State.__init__(self)
        self.x = x
        self.y = y
        self.angle = angle
        self.x_sent = False
        self.y_sent = False


    def on_enter(self):
        self.process()


    def on_resettled(self, axis, position, angle):
        self.process()


    def process(self):
        if not self.x_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_ABSCISSA
            packet.position = self.x
            packet.angle = self.angle
            self.send_packet(packet)
            self.x_sent = True
        elif not self.y_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_ORDINATE
            packet.position = self.y
            packet.angle = self.angle
            self.send_packet(packet)
            self.y_sent = True
        else:
            self.exit_substate()




class Deploy(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Deployment())


    def on_deployed(self):
        self.exit_substate()




class StorePiece1(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece1())


    def on_piece_stored1(self, piece_count):
        self.exit_substate()




class StorePiece2(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece2())


    def on_piece_stored2(self, piece_count):
        self.exit_substate()




class StorePiece3(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece3())


    def on_piece_stored3(self, piece_count):
        self.exit_substate()




class ReleasePiece(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.ReleasePiece())


    def on_piece_released(self):
        self.exit_substate()




class OpenNippers(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.OpenNippers())


    def on_nippers_opened(self):
        self.exit_substate()




class CloseNippers(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.CloseNippers())


    def on_nippers_closed(self):
        self.exit_substate()
