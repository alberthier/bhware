#!/usr/bin/env python
# encoding: utf-8

import eventloop
import config
import signal
import logger
import argparse
import leds




loop = None


def signal_handler(signum, frame):
    global loop
    if loop != None:
        loop.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "BH Team's main strategy program.", add_help = True)
    parser.add_argument("--webserver-port", action = "store", type = int, default = config.webserver_port, metavar = "PORT", help = "Internal web server port")
    parser.add_argument('statemachine', action="store", nargs='?', default = config.state_machine)
    args = parser.parse_args()

    logger.initialize()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    loop = eventloop.EventLoop(args.statemachine, args.webserver_port)
    leds.initialize(loop)
    loop.start()
