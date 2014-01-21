# encoding: utf-8

import math
import os
import sys

import goalmanager
import logger
import packets
import statemachine

from commonstates import *
from definitions import *
from position import *


class Main(statemachine.State):

    def on_enter(self):
        self.fsm.error = None
        try:
            if hasattr(self.fsm, "code") and len(self.fsm.code) != 0:
                code = ""
                for line in self.fsm.code.splitlines():
                    code += "            " + line + "\n"
                state = """
class WebCodeState(statemachine.State):
    def on_enter(self):
        try:
{code}
        except BaseException as e:
            self.log_exception(e)
        yield None
"""
                state = state.format(code = code)
                exec(compile(state, "<webcode>", "exec"))
                yield eval("WebCodeState()")
                exec(compile("del WebCodeState", "<webcode>", "exec"))
        except Exception as e:
            self.fsm.error = e
            logger.log_exception(e)
        yield None
