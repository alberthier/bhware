import statemachine
import logger
import packets
from commonstates import *
from definitions import *




class Main(statemachine.State):

    def on_enter(self):
        self.fsm.glasses_count = 0
        logger.log("Init barman : {}".format(self.fsm.side))


    def on_start(self, packet):
        yield Gripper(self.fsm.side, MOVE_OPEN)
        while True:
            yield CollectGlasses()




class CollectGlasses(statemachine.State):

    def on_enter(self):
        wait = WaitForGlass(self.fsm.side)
        wait.on_internal_drop_glasses = self.on_internal_drop_glasses

        # First glass
        yield wait
        yield Gripper(self.fsm.side, MOVE_CLOSE)
        yield Lifter(self.fsm.side, LIFTER_MOVE_UP)
        yield TopHolder(self.fsm.side, MOVE_CLOSE)
        yield Gripper(self.fsm.side, MOVE_OPEN)
        yield Lifter(self.fsm.side, LIFTER_MOVE_DOWN)
        self.fsm.glasses_count += 1

        # Second glass
        yield wait
        yield Gripper(self.fsm.side, MOVE_CLOSE)
        yield Lifter(self.fsm.side, LIFTER_MOVE_MIDDLE)
        self.fsm.glasses_count += 1

        # Third glass
        yield wait
        yield BottomHolder(self.fsm.side, MOVE_CLOSE)
        self.fsm.glasses_count += 1


    def on_internal_drop_glasses(self, packet):
        if not packet.done:
            yield UnloadGlasses()
            self.send_packet(packets.InternalDropGlasses(can_continue=packet.can_continue, done=True))
        yield None




class UnloadGlasses(statemachine.State):

    def on_enter(self):
        if self.fsm.glasses_count == 1:
            yield TopHolder(self.fsm.side, MOVE_OPEN)
            yield Gripper(self.fsm.side, MOVE_OPEN)
            yield BottomHolder(self.fsm.side, MOVE_OPEN)
        elif self.fsm.glasses_count == 2:
            yield TopHolder(self.fsm.side, MOVE_OPEN)
            yield Lifter(self.fsm.side, LIFTER_MOVE_DOWN)
            yield Gripper(self.fsm.side, MOVE_OPEN)
        elif self.fsm.glasses_count == 3:
            yield TopHolder(self.fsm.side, MOVE_OPEN)
        yield None




class EndOfMatch(statemachine.State):

    def on_enter(self):
        logger.log('Barman {} : end of match'.format(self.fsm.side))

