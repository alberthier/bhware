#!/usr/bin/env python3
# encoding: utf-8

import eventloop
import signal
import logger
import argparse
import leds
from definitions import *




loop = None


def signal_handler(signum, frame):
    global loop
    if loop is not None:
        loop.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "BH Team's main strategy program.", add_help = True)
    parser.add_argument("--webserver-port", action = "store", type = int, default = WEB_SERVER_PORT, metavar = "PORT", help = "Internal web server port")
    parser.add_argument('statemachine', action="store", nargs='?', default = STATE_MACHINE)
    args = parser.parse_args()

    logger.initialize()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    loop = eventloop.EventLoop(args.statemachine, args.webserver_port)
    leds.initialize(loop)
    loop.start()
