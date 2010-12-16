#!/usr/bin/env python
# encoding: utf-8

import socket

from definitions import *

if socket.gethostname() == "drunkstar":
    host_device = HOST_DEVICE_ARM
    ## warning PIC IP address has changed since last year
    remote_ip = "192.168.1.200"
    serial_port_path = "/dev/ttyUSB0"
    orange_led_device_path = "/sys/class/leds/dockstar:orange:health/brightness"
    green_led_device_path = "/sys/class/leds/dockstar:green:health/brightness"
else:
    host_device = HOST_DEVICE_PC
    remote_ip = "127.0.0.1"
    serial_port_path = None
    orange_led_device_path = None
    green_led_device_path = None

## ## warning PIC remote port has changed since last year
remote_port = 7001
state_machine = "default"
## should all log messages appear in the console ?
force_log_to_screen = False
# force_log_to_screen = True

# serial port communication is disabled for the moment
serial_port_path = None
