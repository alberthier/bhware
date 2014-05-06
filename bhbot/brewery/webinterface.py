# encoding: utf-8


import json
import os
import socket
import traceback
import urllib.parse

import statemachine


class WebInterface:

    def __init__(self, event_loop):
        self.event_loop = event_loop

    def __call__(self, environ, start_response):
        response = None
        path = environ["PATH_INFO"]
        if path == "/hostname":
            response = socket.gethostname()
        elif path == "/statemachines":
            response = self.statemachines(environ)
        elif path == "/remotecontrol":
            response = self.remotecontrol(environ)
        elif path == "/eval":
            response = self.eval(environ)

        if response is None:
            code = "404 Not Found"
            response = ""
        else:
            code = "200 OK"

        start_response(code, [('Content-type','text/plain; charset=utf-8')])
        return [bytes(response, "utf-8")]


    def hostname(self, environ):
        return socket.gethostname()


    def statemachines(self, environ):
        result = {}
        for fsm in self.event_loop.fsms:
            states = []
            result[fsm.name] = states
            for state in fsm.state_stack:
                states.append(state.name)
        return json.dumps(result)


    def remotecontrol(self, environ):
        result = {}

        actions = []
        actions.append("yield GotoHome()")
        actions.append("yield MoveLineRelative(0.2)")
        actions.append("yield MoveLineRelative(-0.2, DIRECTION_BACKWARDS)")
        actions.append("yield RotateRelative(math.radians(45))")
        actions.append("yield RotateRelative(math.pi / 4.0)")
        actions.append("yield Trigger(ELEVATOR_UP)")
        actions.append("yield Trigger(ELEVATOR_DOWN)")
        actions.append("yield Trigger(ARM_1_OPEN)")
        actions.append("yield Trigger(ARM_1_CLOSE)")
        actions.append("yield Trigger(ARM_2_OPEN)")
        actions.append("yield Trigger(ARM_2_CLOSE)")
        actions.append("yield Trigger(FIRE_FLIPPER_OPEN)")
        actions.append("yield Trigger(FIRE_FLIPPER_CLOSE)")
        actions.append("yield Trigger(TORCH_GUIDE_OPEN)")
        actions.append("yield Trigger(TORCH_GUIDE_CLOSE)")
        actions.append("yield Trigger(FRUITMOTH_HATCH_OPEN)")
        actions.append("yield Trigger(FRUITMOTH_HATCH_CLOSE)")
        actions.append("yield Trigger(FRUITMOTH_TANK_OPEN)")
        actions.append("yield Trigger(FRUITMOTH_TANK_CLOSE)")
        actions.append("yield Trigger(FRUITMOTH_ARM_OPEN)")
        actions.append("yield Trigger(FRUITMOTH_ARM_CLOSE)")
        actions.append("yield Trigger(FRUITMOTH_FINGER_OPEN)")
        actions.append("yield Trigger(FRUITMOTH_FINGER_CLOSE)")
        actions.append("yield Trigger(GUN_FIRE)")
        actions.append("yield Trigger(GUN_LOAD)")
        actions.append("yield Trigger(MAMMOTH_NET_THROW)")
        actions.append("yield Trigger(MAMMOTH_NET_LOAD)")
        actions.append("yield Trigger(VALVE_ON)")
        actions.append("yield Trigger(VALVE_OFF)")
        actions.append("yield Trigger(PAINT_1_RELEASE)")
        actions.append("yield Trigger(PAINT_1_HOLD)")
        actions.append("yield Trigger(PAINT_2_RELEASE)")
        actions.append("yield Trigger(PAINT_2_HOLD)")
        actions.append("yield Trigger(PUMP_ON)")
        actions.append("yield Trigger(PUMP_OFF)")
        result["Actions"] = actions

        misc = []
        misc.append("yield Timer(1000)")
        misc.append("yield StopAll()")
        misc.append("yield DefinePosition(0.0, 0.0, math.pi / 2.0)")
        misc.append("yield AntiBlocking(True)")
        misc.append("yield SpeedControl(0.3)")
        misc.append("yield Rotate(math.pi / 2.0)")
        misc.append("yield LookAt(1.0, 1.0)")
        misc.append("yield LookAtOpposite(1.0, 1.0)")
        misc.append("yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)])")
        misc.append("yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)], DIRECTION_BACKWARDS)")
        misc.append("yield MoveLineTo(0.1, 0.0)")
        misc.append("yield MoveLineTo(0.1, 0.0, DIRECTION_BACKWARDS)")
        misc.append("yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)])")
        misc.append("yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)], DIRECTION_BACKWARDS)")
        result["Misc"] = misc

        return json.dumps(result)


    def eval(self, environ):
        encoding = "iso-8859-1"
        if "CONTENT_TYPE" in environ:
            for key, value in urllib.parse.parse_qsl(environ["CONTENT_TYPE"]):
                if key == "charset":
                    encoding = value
                    break
        code = str(environ["wsgi.input"].read(), encoding)
        fsm = statemachine.StateMachine(self.event_loop, "eval", code = code)
        text = ""
        if fsm.error is not None:
            for l in traceback.format_exception(type(fsm.error), fsm.error, None):
                text += l
        else:
            text = "OK"
        return text
