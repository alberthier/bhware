import statemachine
import logger
import packets
import commonstates

from definitions import *




class GlassState(statemachine.State):

    def __init__(self, side):
        self.side = side
        self.glass_count = 0


    def on_internal_drop_glasses(self, packet):
        pass




class Main(GlassState):

    def __init__(self, side = None):
        super().__init__(side)


    def on_enter(self):
        logger.log("Init barman : {}".format(self.side))


    def on_device_ready(self, packet):
        yield commonstates.Gripper(self.side, MOVE_OPEN)
        yield NoGlassPresent(self.side)




class EndOfMatch(statemachine.State):

    def on_enter(self):
        logger.log('Barman {} : end of match'.format(self.side))




class NoGlassPresent(GlassState):

    glass_count = 0

    def on_glass_present(self, packet):
        self.send_packet(packets.Gripper(side = self.side, move = MOVE_CLOSE))


    def on_gripper(self, packet):
        if packet.move == MOVE_CLOSE:
            self.send_packet(packets.Lifter(side = self.side, move = LIFTER_MOVE_UP))
        else:
            self.send_packet(packets.Lifter(side = self.side, move = LIFTER_MOVE_DOWN))


    def on_lifter(self, packet):
        if packet.move == LIFTER_MOVE_UP:
            self.send_packet(packets.TopHolder(side = self.side, move = MOVE_CLOSE))
        else:
            # self.robot.glasses_count[self.side] += 1
            yield OneGlassPresent(self.side)


    def on_top_holder(self, packet):
        self.send_packet(packets.Gripper(side = self.side, move = MOVE_OPEN))




class OneGlassPresent(GlassState):

    glass_count = 1

    def on_glass_present(self, packet):
        self.send_packet(packets.Gripper(side = self.side, move = MOVE_CLOSE))


    def on_gripper(self, packet):
        self.send_packet(packets.Lifter(side = self.side, move = LIFTER_MOVE_MIDDLE))


    def on_lifter(self, packet):
        yield TwoGlassesPresent(self.side)


    def on_internal_drop_glasses(self, packet):
        if not packet.done :
            yield UnloadOneGlass(self.side)
            self.send_packet(packets.InternalDropGlasses(can_continue=packet.can_continue, done=True))
            yield NoGlassPresent(self.side)




class TwoGlassesPresent(GlassState):

    glass_count = 2

    def on_glass_present(self, packet):
        self.send_packet(packets.BottomHolder(side = self.side, move = MOVE_CLOSE))


    def on_bottom_holder(self, packet):
        yield ThreeGlassesPresent(self.side)


    def on_internal_drop_glasses(self, packet):
        if not packet.done :
            yield UnloadTwoGlasses(self.side)
            self.send_packet(packets.InternalDropGlasses(can_continue=packet.can_continue, done=True))
            yield NoGlassPresent(self.side)




class ThreeGlassesPresent(GlassState):

    glass_count = 3

    def on_internal_drop_glasses(self, packet):
        if not packet.done :
            yield UnloadThreeGlasses(self.side)
            self.send_packet(packets.InternalDropGlasses(can_continue=packet.can_continue, done=True))
            yield NoGlassPresent(self.side)




class UnloadThreeGlasses(GlassState):

    def on_enter(self):
        yield commonstates.SendPacketsAndWaitAnswer(
            packets.TopHolder(side = self.side, move = MOVE_OPEN),
            packets.Gripper(side = self.side, move = MOVE_OPEN),
            packets.BottomHolder(side = self.side, move = MOVE_OPEN),
            )
        yield None




class UnloadTwoGlasses(GlassState):

    def on_enter(self):
        yield commonstates.SendPacketsAndWaitAnswer(
            packets.BottomHolder(side = self.side, move = MOVE_OPEN),
            packets.TopHolder(side = self.side, move = MOVE_OPEN),
        )
        yield commonstates.SendPacketsAndWaitAnswer(
            packets.Lifter(side = self.side, move = LIFTER_MOVE_DOWN),
        )
        yield commonstates.SendPacketsAndWaitAnswer(
            packets.Gripper(side = self.side, move = MOVE_OPEN),
        )
        yield None




class UnloadOneGlass(GlassState):
    def on_enter(self):
        # just drop the glass
        yield commonstates.SendPacketsAndWaitAnswer(
            self.send_packet(packets.BottomHolder(side = self.side, move = MOVE_OPEN)),
            self.send_packet(packets.TopHolder(side = self.side, move = MOVE_OPEN)),
        )
        yield None

