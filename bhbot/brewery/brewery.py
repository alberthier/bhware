#!/usr/bin/env python3
# encoding: utf-8

import signal
import argparse
import socket




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
    parser.add_argument('statemachine', action="store", nargs='?', default = socket.gethostname())
    parser.add_argument("--pydev-debug", nargs=2)
    args = parser.parse_args()

    if args.pydev_debug :
        import sys
        if sys.platform=="darwin":
            sys.path.append('/Applications/PyCharm.app/pycharm-debug-py3k.egg')
            import pydevd
            pydevd.settrace(args.pydev_debug[0], port=int(args.pydev_debug[1]), stdoutToServer=True,
                                                          stderrToServer=True)

    definitions.setup_definitions(args.statemachine == "sheldon")

    import eventloop
    import logger
    import leds

    logger.initialize()

    loop = eventloop.EventLoop(args.statemachine, args.webserver_port)
    leds.initialize(loop)
    loop.start()
    logger.close()
