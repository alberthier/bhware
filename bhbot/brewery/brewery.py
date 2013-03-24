#!/usr/bin/env python3
# encoding: utf-8

import signal
import argparse




loop = None


def signal_handler(signum, frame):
    global loop
    if loop is not None:
        loop.stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    import definitions

    parser = argparse.ArgumentParser(description = "BH Team's main strategy program.", add_help = True)
    parser.add_argument("--webserver-port", action = "store", type = int, default = definitions.WEB_SERVER_PORT, metavar = "PORT", help = "Internal web server port")
    parser.add_argument("--main", action = "store_true", default = definitions.IS_MAIN_ROBOT, help = "Main robot definitions")
    parser.add_argument('statemachine', action="store", nargs='?', default = definitions.STATE_MACHINE)
    args = parser.parse_args()

    definitions.IS_MAIN_ROBOT = args.main
    definitions.setup_definitions()

    import eventloop
    import logger
    import leds

    logger.initialize()

    loop = eventloop.EventLoop(args.statemachine, args.webserver_port)
    leds.initialize(loop)
    loop.start()
    logger.close()
