#!/usr/bin/env python
# encoding: utf-8

import sys

import eventloop
import config
import signal


loop = None


def signal_handler(signum, frame):
    global loop
    if loop != None:
        loop.stop()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        state_machine_name = sys.argv[1]
    else:
        state_machine_name = config.state_machine

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    loop = eventloop.EventLoop(state_machine_name)
    loop.start()
