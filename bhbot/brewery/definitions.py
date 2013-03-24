# encoding: utf-8

import os
import math
import platform
import socket


########################################################################
# Constants

# Brewery execution host
IS_MAIN_ROBOT                          = socket.gethostname() == "sheldon"
IS_HOST_DEVICE_ARM                     = platform.machine() == "armv5tel"
IS_HOST_DEVICE_PC                      = not IS_HOST_DEVICE_ARM

# Field
FIELD_Y_SIZE                           = 3.0
FIELD_X_SIZE                           = 2.0

# Main robot
MAIN_ROBOT_X_SIZE                      = 0.254
MAIN_ROBOT_Y_SIZE                      = 0.252
MAIN_ROBOT_CENTER_X                    = 0.126
MAIN_ROBOT_CENTER_Y                    = 0.126
MAIN_ROBOT_GYRATION_RADIUS             = 0.174

# Secondary robot
SECONDARY_ROBOT_X_SIZE                 = 0.172
SECONDARY_ROBOT_Y_SIZE                 = 0.127
SECONDARY_ROBOT_CENTER_X               = 0.0530
SECONDARY_ROBOT_CENTER_Y               = 0.0635
SECONDARY_ROBOT_GYRATION_RADIUS        = 0.161

# Main start positons (the robot starts 90 degrees rotated that's why *_START_Y use ROBOT_X_SIZE and ROBOT_CENTER_X)
MAIN_BLUE_START_Y                      = 0.126
MAIN_BLUE_START_X                      = 1.0
MAIN_BLUE_START_ANGLE                  = math.pi / 2.0
MAIN_RED_START_X                       = MAIN_BLUE_START_X
MAIN_RED_START_Y                       = FIELD_Y_SIZE - MAIN_BLUE_START_Y
MAIN_RED_START_ANGLE                   = -math.pi / 2.0

# Secondary start positons (the robot starts 90 degrees rotated that's why *_START_Y use ROBOT_X_SIZE and ROBOT_CENTER_X)
SECONDARY_BLUE_START_Y                 = 0.053
SECONDARY_BLUE_START_X                 = 1.5
SECONDARY_BLUE_START_ANGLE             = math.pi / 2.0
SECONDARY_RED_START_X                  = SECONDARY_BLUE_START_X
SECONDARY_RED_START_Y                  = FIELD_Y_SIZE - SECONDARY_BLUE_START_Y
SECONDARY_RED_START_ANGLE              = -math.pi / 2.0

def setup_definitions():
    globals()["ROBOT_X_SIZE"]          = MAIN_ROBOT_X_SIZE          if IS_MAIN_ROBOT else SECONDARY_ROBOT_X_SIZE
    globals()["ROBOT_Y_SIZE"]          = MAIN_ROBOT_Y_SIZE          if IS_MAIN_ROBOT else SECONDARY_ROBOT_Y_SIZE
    globals()["ROBOT_CENTER_X"]        = MAIN_ROBOT_CENTER_X        if IS_MAIN_ROBOT else SECONDARY_ROBOT_CENTER_X
    globals()["ROBOT_CENTER_Y"]        = MAIN_ROBOT_CENTER_Y        if IS_MAIN_ROBOT else SECONDARY_ROBOT_CENTER_Y
    globals()["ROBOT_GYRATION_RADIUS"] = MAIN_ROBOT_GYRATION_RADIUS if IS_MAIN_ROBOT else SECONDARY_ROBOT_GYRATION_RADIUS
    globals()["BLUE_START_Y"]          = MAIN_BLUE_START_Y          if IS_MAIN_ROBOT else SECONDARY_BLUE_START_Y
    globals()["BLUE_START_X"]          = MAIN_BLUE_START_X          if IS_MAIN_ROBOT else SECONDARY_BLUE_START_X
    globals()["BLUE_START_ANGLE"]      = MAIN_BLUE_START_ANGLE      if IS_MAIN_ROBOT else SECONDARY_BLUE_START_ANGLE
    globals()["RED_START_Y"]           = MAIN_RED_START_Y           if IS_MAIN_ROBOT else SECONDARY_RED_START_Y
    globals()["RED_START_X"]           = MAIN_RED_START_X           if IS_MAIN_ROBOT else SECONDARY_RED_START_X
    globals()["RED_START_ANGLE"]       = MAIN_RED_START_ANGLE       if IS_MAIN_ROBOT else SECONDARY_RED_START_ANGLE

# Rule specific
MATCH_DURATION_MS                      = 90000
TEAM_COLOR_RED                         = "#a1011d"
TEAM_COLOR_BLUE                        = "#004c90"

# Timing
KEEP_ALIVE_DELAY_MS                    = 250
KEEP_ALIVE_MINIMUM_AGE_S               = (KEEP_ALIVE_DELAY_MS * 4.0 / 5.0) / 1000.0
EVENT_LOOP_TICK_RESOLUTION_S           = 0.05

# Remote device connection
if IS_HOST_DEVICE_ARM:
    REMOTE_IP                          = "192.168.2.200"
else:
    REMOTE_IP                          = "127.0.0.1"
REMOTE_PORT                            = 7001
REMOTE_LOG_PORT                        = 23

if IS_HOST_DEVICE_ARM:
    MAIN_INTERBOT_IP = "192.168.1.1"
else:
    MAIN_INTERBOT_IP = "127.0.0.1"
MAIN_INTERBOT_PORT = 7002

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
    LOG_DIR                            = "/tmp/bhlogs"
else:
    LOG_DIR                            = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "logs")

# Brewery's web sever
WEB_SERVER_PORT                        = 8080

# Default state machine name
STATE_MACHINE                          = "default"

# Use multipoint Goto for navigation
NAVIGATION_USES_MULTIPOINT             = False

# Use pathfinding algorithm to evaluate the best goal
GOAL_EVALUATION_USES_PATHFINDING       = True

# Router map resolution
ROUTING_MAP_RESOLUTION                 = 0.02
EVALUATOR_MAP_RESOLUTION               = 0.04
MAP_WALLS_DISTANCE                     = 0.127
ASTAR_EFFECTIVE_VS_HEURISTIC_TRADEOFF  = 1.5
ROUTE_SPLIT_ANGLE                      = 2.0 * math.pi
MAIN_OPPONENT_AVOIDANCE_RANGE          = 0.5
SECONDARY_OPPONENT_AVOIDANCE_RANGE     = 0.4

# Blocked zone
BLOCKED_ZONE_SIZE                      = 0.08
BLOCKED_ZONE_DISAPEARING_MS            = 1000

# Opponent detection
OPPONENT_DETECTION_DISAPEARING_MS      = 3000

# Blocking opponent handling
DEFAULT_OPPONENT_WAIT_MS               = 2000

# Turret detection ranges
TURRET_SHORT_DISTANCE_DETECTION_RANGE  = 0.55
TURRET_LONG_DISTANCE_DETECTION_RANGE   = 1.0
TURRET_SHORT_DISTANCE_DETECTION_ID     = 190
TURRET_LONG_DISTANCE_DETECTION_ID      = 240


########################################################################
# Enums


class Enum(object):

    def __init__(self, description, **kwargs):
        self.description = description
        self.lookup_by_name = {}
        self.lookup_by_value = {}
        auto_value = 0
        for enum_item_name, enum_item_value in list(kwargs.items()):
            if enum_item_value is None:
                enum_item_value = auto_value
            auto_value = enum_item_value + 1
            globals()[enum_item_name] = enum_item_value
            self.lookup_by_name[enum_item_name] = enum_item_value
            self.lookup_by_value[enum_item_value] = enum_item_name




REMOTE_DEVICE = Enum("Remote hardware type",
    REMOTE_DEVICE_PIC       = 0,
    REMOTE_DEVICE_SIMULATOR = 1,
    REMOTE_DEVICE_UNKNOWN   = 2,
)

TEAM = Enum("Team color",
    TEAM_BLUE    = 0,
    TEAM_RED     = 1,
    TEAM_UNKNOWN = 2,
)

MOVEMENT = Enum("Movement",
    MOVEMENT_ROTATE = 0,
    MOVEMENT_MOVE   = 1,
    MOVEMENT_LINE   = 2,
)

DIRECTION = Enum("Direction",
    DIRECTION_AUTO     =  0,
    DIRECTION_FORWARDS  =  1,
    DIRECTION_BACKWARDS = -1,
)

REASON = Enum("Goto finished reason",
    REASON_DESTINATION_REACHED = 0,
    REASON_BLOCKED_FRONT       = 1,
    REASON_BLOCKED_BACK        = 2,
    REASON_STOP_REQUESTED      = 3,
)

AXIS = Enum("Axis",
    AXIS_X = 0,
    AXIS_Y = 1,
)

SIDE = Enum("Side",
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1,
)

MOVE = Enum("Move",
    MOVE_CLOSE = 0,
    MOVE_OPEN  = 1,
)

LIFTER_MOVE = Enum("Lifter move",
    LIFTER_MOVE_DOWN   = 0,
    LIFTER_MOVE_MIDDLE = 1,
    LIFTER_MOVE_UP     = 2,
)

CANDLE_KICKER = Enum("Candle kicker",
    CANDLE_KICKER_LOWER = 0,
    CANDLE_KICKER_UPPER = 1,
)

CANDLE_KICKER_POSITION = Enum("Candle kicker position",
    CANDLE_KICKER_POSITION_IDLE = 0,
    CANDLE_KICKER_POSITION_UP   = 1,
    CANDLE_KICKER_POSITION_KICK = 2,
)

GIFT_OPENER_POSITION = Enum("Gift Opener",
    GIFT_OPENER_POSITION_IDLE  = 0,
    GIFT_OPENER_POSITION_LEFT  = 1,
    GIFT_OPENER_POSITION_RIGHT = 2,
)

PUMP = Enum("Pump",
    PUMP_OFF = 0,
    PUMP_ON  = 1,
)

TRAJECTORY = Enum("Trajectory walk or navigation result",
    TRAJECTORY_DESTINATION_REACHED     = 0,
    TRAJECTORY_BLOCKED                 = 1,
    TRAJECTORY_OPPONENT_DETECTED       = 2,
    TRAJECTORY_DESTINATION_UNREACHABLE = 3,
)

OPPONENT_ROBOT = Enum("Detected opponent robot",
    OPPONENT_ROBOT_MAIN      = 0,
    OPPONENT_ROBOT_SECONDARY = 1,
)

TURRET_INIT_MODE = Enum("Turret initialization read/write mode",
    TURRET_INIT_MODE_READ  = 0,
    TURRET_INIT_MODE_WRITE = 1,
)
