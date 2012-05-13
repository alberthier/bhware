#!/usr/bin/env python
# encoding: utf-8

import os
import math
import socket


########################################################################
# Constants

# Field
FIELD_Y_SIZE                           = 3.0
FIELD_X_SIZE                           = 2.0

# Game elements
COIN_RADIUS                            = 0.060
COIN_COLOR_WHITE                       = "#ffffff"
COIN_COLOR_BLACK                       = "#000000"
GOLD_BAR_WIDTH                         = 0.070
GOLD_BAR_LENGTH                        = 0.150
GOLD_BAR_COLOR                         = "#ffdd00"

# Robot
ROBOT_X_SIZE                           = 0.276
ROBOT_Y_SIZE                           = 0.322
ROBOT_CENTER_X                         = 0.099
ROBOT_CENTER_Y                         = 0.161
ROBOT_GYRATION_RADIUS                  = 0.2386
ROBOT_EXPANDED_GRIPPER_GYRATION_RADIUS = 0.321965
ROBOT_EXPANDED_SWEEPER_GYRATION_RADIUS = 0.289355
ROBOT_EXPANDED_GYRATION_RADIUS         = max(ROBOT_EXPANDED_GRIPPER_GYRATION_RADIUS, ROBOT_EXPANDED_SWEEPER_GYRATION_RADIUS)

# Start positons (the robot starts 90 degrees rotated that's why *_START_Y use ROBOT_X_SIZE and ROBOT_CENTER_X)
RED_START_X                            = 0.310
RED_START_Y                            = FIELD_Y_SIZE - (ROBOT_X_SIZE - ROBOT_CENTER_X)
RED_START_ANGLE                        = math.pi / 2.0
PURPLE_START_X                         = 0.310
PURPLE_START_Y                         = ROBOT_X_SIZE - ROBOT_CENTER_X
PURPLE_START_ANGLE                     = -math.pi / 2.0

# Rule specific
MATCH_DURATION_MS                      = 90000
TEAM_COLOR_RED                         = "#ff3b3d"
TEAM_COLOR_PURPLE                      = "#9632f6"

# Timing
KEEP_ALIVE_DELAY_MS                    = 200
EVENT_LOOP_TICK_RESOLUTION_S           = 0.05

# Brewery execution host
IS_HOST_DEVICE_ARM                     = socket.gethostname() == "drunkstar"
IS_HOST_DEVICE_PC                      = not IS_HOST_DEVICE_ARM

# Remote device connection
if IS_HOST_DEVICE_ARM:
    REMOTE_IP                          = "192.168.2.200"
else:
    REMOTE_IP                          = "127.0.0.1"
REMOTE_PORT                            = 7001
REMOTE_LOG_PORT                        = 23

# Serial port
if IS_HOST_DEVICE_ARM:
    SERIAL_PORT_PATH                   = "/dev/ttyUSB0"
else:
    SERIAL_PORT_PATH                   = None
SERIAL_PORT_SPEED                      = 115200

# Leds:
if IS_HOST_DEVICE_ARM:
    ORANGE_LED_DEVICE_PATH             = "/sys/class/leds/dockstar:orange:misc/brightness"
    GREEN_LED_DEVICE_PATH              = "/sys/class/leds/dockstar:green:health/brightness"
else:
    ORANGE_LED_DEVICE_PATH             = None
    GREEN_LED_DEVICE_PATH              = None

# Log directory
if IS_HOST_DEVICE_ARM:
    LOG_DIR                            = "/root/logs"
else:
    LOG_DIR                            = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "logs")

# Brewery's web sever
WEB_SERVER_PORT                        = 80

# Default state machine name
STATE_MACHINE                          = "default"

# Use multipoint Goto for navigation
NAVIGATION_USES_MULTIPOINT             = False

# Router map resolution
ROUTING_MAP_RESOLUTION                 = 0.02
EVALUATOR_MAP_RESOLUTION               = 0.04
MAP_WALLS_DISTANCE                     = 0.20
ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF  = 1.5
ROUTE_SPLIT_ANGLE                      = 2.0 * math.pi
MAIN_OPPONENT_AVOIDANCE_RANGE          = 0.5
SECONDARY_OPPONENT_AVOIDANCE_RANGE     = 0.4

# Blocked zone
BLOCKED_ZONE_SIZE                      = 0.08
BLOCKED_ZONE_DISAPEARING_MS            = 1000

# Opponent detection
OPPONENT_DETECTION_DISAPEARING_MS      = 1500

# Blocking opponent handling
DEFAULT_OPPONENT_WAIT_MS               = 2000

# Turret detection ranges
TURRET_SHORT_DISTANCE_DETECTION_RANGE  = 0.55
TURRET_LONG_DISTANCE_DETECTION_RANGE   = 1.0


########################################################################
# Enums


ENUMS = {}


class Enum(object):

    def __init__(self, name, description, **kwargs):
        self.name = name
        self.description = description
        self.lookup_by_name = {}
        self.lookup_by_value = {}
        ENUMS[name] = self
        for enum_item_name, enum_item_value in kwargs.items():
            globals()[enum_item_name] = enum_item_value
            self.lookup_by_name[enum_item_name] = enum_item_value
            self.lookup_by_value[enum_item_value] = enum_item_name




Enum("REMOTE_DEVICE",
     "Remote hardware type",
     REMOTE_DEVICE_PIC       = 0,
     REMOTE_DEVICE_SIMULATOR = 1,
     REMOTE_DEVICE_UNKNOWN   = 2,
)


Enum("TEAM",
     "Team color",
     TEAM_PURPLE  = 0,
     TEAM_RED     = 1,
     TEAM_UNKNOWN = 2,
)

Enum("MOVEMENT",
     "Movement",
     MOVEMENT_ROTATE = 0,
     MOVEMENT_MOVE   = 1,
     MOVEMENT_LINE   = 2,
)

Enum("DIRECTION",
     "Direction",
     DIRECTION_FORWARD  =  1,
     DIRECTION_BACKWARD = -1,
)

Enum("REASON",
     "Goto finished reason",
     REASON_DESTINATION_REACHED = 0,
     REASON_BLOCKED_FRONT       = 1,
     REASON_BLOCKED_BACK        = 2,
     REASON_STOP_REQUESTED      = 3,
)

Enum("AXIS",
     "Axis",
     AXIS_X = 0,
     AXIS_Y = 1,
)

Enum("GRIPPER",
     "Gripper Position",
     GRIPPER_CLOSE = 0,
     GRIPPER_OPEN  = 1,
)

Enum("GRIPPER_SIDE",
     "Gripper side",
     GRIPPER_SIDE_LEFT  = 1,
     GRIPPER_SIDE_RIGHT = 2,
     GRIPPER_SIDE_BOTH  = 3,
)

Enum("SWEEPER",
     "Sweeper position",
     SWEEPER_CLOSE = 0,
     SWEEPER_OPEN  = 1,
)

Enum("MAP_ARM",
     "Map arm position",
     MAP_ARM_CLOSE = 0,
     MAP_ARM_OPEN  = 1,
)

Enum("MAP_GRIPPER",
     "Map gripper position",
     MAP_GRIPPER_CLOSE = 0,
     MAP_GRIPPER_OPEN  = 1,
)

Enum("TANK",
     "Tank position",
     TANK_RETRACT = 0,
     TANK_DEPLOY  = 1,
)

Enum("FABRIC_STORE",
     "Fabric store control",
     FABRIC_STORE_LOW  = 0,
     FABRIC_STORE_HIGH = 1,
)

Enum("GOLD_BAR",
     "Gold bar sensor",
     GOLD_BAR_MISSING = 0,
     GOLD_BAR_PRESENT = 1,
)

Enum("TRAJECTORY",
    "Trajectory walk or navigation result",
    TRAJECTORY_DESTINATION_REACHED     = 0,
    TRAJECTORY_BLOCKED                 = 1,
    TRAJECTORY_OPPONENT_DETECTED       = 2,
    TRAJECTORY_DESTINATION_UNREACHABLE = 3,
)

Enum("OPPONENT_ROBOT",
     "Detected opponent robot",
     OPPONENT_ROBOT_MAIN = 0,
     OPPONENT_ROBOT_SECONDARY = 1,
)

Enum("TURRET_INIT_MODE",
     "Turret initialization read/write mode",
    TURRET_INIT_MODE_READ  = 0,
    TURRET_INIT_MODE_WRITE = 1,
)
