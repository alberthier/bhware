import statemachine
import logger
import packets
from commonstates import *
from definitions import *




class Main(statemachine.State):

    def on_enter(self):
        self.fsm.glasses_count = 0
        self.fsm.name += "[{}]".format(SIDE.lookup_by_value[self.fsm.side])
        self.log("Init")


    def on_start(self, packet):
        yield Gripper(self.fsm.side, MOVE_OPEN)
        while True:
            yield CollectGlasses()




class CollectGlasses(statemachine.State):

    def on_enter(self):
        wait = WaitForGlass(self.fsm.side)
        wait.on_internal_drop_glasses = self.on_internal_drop_glasses
        self.fsm.glasses_count = 0
        self.dropping = False

        # First glass
        yield wait

        if self.dropping :
            yield None

        yield Gripper(self.fsm.side, MOVE_CLOSE)
        yield Lifter(self.fsm.side, LIFTER_MOVE_UP)
        yield TopHolder(self.fsm.side, MOVE_CLOSE)
        yield Gripper(self.fsm.side, MOVE_OPEN)
        yield Lifter(self.fsm.side, LIFTER_MOVE_DOWN)
        self.fsm.glasses_count += 1
        logger.log('{} glasses count {}'.format(SIDE.lookup_by_value[self.fsm.side], self.fsm.glasses_count))

        # Second glass
        yield wait

        if self.dropping :
            yield None

        yield Gripper(self.fsm.side, MOVE_CLOSE)
        yield Lifter(self.fsm.side, LIFTER_MOVE_MIDDLE)
        self.fsm.glasses_count += 1
        logger.log('{} glasses count {}'.format(SIDE.lookup_by_value[self.fsm.side], self.fsm.glasses_count))

        # Third glass
        yield wait

        if self.dropping :
            yield None

        yield BottomHolder(self.fsm.side, MOVE_OPEN)
        self.fsm.glasses_count += 1
        logger.log('{} glasses count {}'.format(SIDE.lookup_by_value[self.fsm.side], self.fsm.glasses_count))


    def on_internal_drop_glasses(self, packet):
        if not packet.done:
            self.dropping = True
            yield UnloadGlasses()
            self.send_packet(packets.InternalDropGlasses(side=self.fsm.side,can_continue=packet.can_continue, done=True))
            yield Timer(2000.0)
            yield Lifter(self.fsm.side, LIFTER_MOVE_DOWN)
            yield None




class UnloadGlasses(statemachine.State):

    def on_enter(self):
        if self.fsm.glasses_count == 1:
            yield TopHolder(self.fsm.side, MOVE_OPEN)
        elif self.fsm.glasses_count == 2:
            yield TopHolder(self.fsm.side, MOVE_OPEN)
            yield Lifter(self.fsm.side, LIFTER_MOVE_DOWN)
            yield Gripper(self.fsm.side, MOVE_OPEN)
        elif self.fsm.glasses_count == 3:
            yield BottomHolder(self.fsm.side, MOVE_CLOSE)
            yield TopHolder(self.fsm.side, MOVE_OPEN)
            yield Gripper(self.fsm.side, MOVE_OPEN)


        self.fsm.glasses_count = 0
        logger.log('{} glasses count {}'.format(SIDE.lookup_by_value[self.fsm.side], self.fsm.glasses_count))

        yield None




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.log("End of match")

