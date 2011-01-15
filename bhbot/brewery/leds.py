#!/usr/bin/env python
# encoding: utf-8

import config
import packets

from definitions import *



green = None
orange = None




def initialize(event_loop):
    global green
    global orange
    if config.host_device == HOST_DEVICE_ARM:
        green = HardwareLed(config.green_led_device_path)
        orange = HardwareLed(config.orange_led_device_path)
    else:
        green = SimulatorLed(event_loop, SimulatorLed.COLOR_GREEN)
        orange = SimulatorLed(event_loop, SimulatorLed.COLOR_ORANGE)




class BaseLed(object):


    HEARTBEAT = [True, False, True, False, False, False]


    def __init__(self):
        self.heartbeat_index = 0


    def is_on(self):
        return False


    def on(self):
        if not self.is_on():
            self.do_set(True)


    def off(self):
        if self.is_on():
            self.do_set(False)


    def toggle(self):
        self.do_set(not self.is_on())


    def set(self, on):
        if on != self.is_on():
            self.do_set(on)


    def do_set(self, on):
        pass


    def heartbeat_tick(self):
        self.set(BaseLed.HEARTBEAT[self.heartbeat_index])
        if self.heartbeat_index == len(BaseLed.HEARTBEAT) - 1:
            self.heartbeat_index = 0
        else:
            self.heartbeat_index += 1




class HardwareLed(BaseLed):

    def __init__(self, device_file_path):
        BaseLed.__init__(self)
        self.device_file = file(device_file_path, "w")
        self.state = False
        self.do_set(False)


    def is_on(self):
        return self.state == 1


    def do_set(self, on):
        self.state = on
        if self.state:
            self.device_file.write("1")
        else:
            self.device_file.write("0")
        self.device_file.flush()




class SimulatorLed(BaseLed):

    COLOR_ORANGE_FLAG = 0x1
    COLOR_ORANGE = 0x2
    COLOR_GREEN_FLAG = 0x4
    COLOR_GREEN = 0x8

    def __init__(self, event_loop, color):
        BaseLed.__init__(self)
        self.event_loop = event_loop
        self.color = color
        self.packet = packets.SimulatorData()
        if color == SimulatorLed.COLOR_ORANGE:
            self.packet.leds |= SimulatorLed.COLOR_ORANGE_FLAG
        else:
            self.packet.leds |= SimulatorLed.COLOR_GREEN_FLAG


    def is_on(self):
        return self.packet.leds & self.color != 0


    def do_set(self, on):
        if on:
            self.packet.leds |= self.color
        else:
            self.packet.leds &= ~self.color
        self.event_loop.send_packet(self.packet)
