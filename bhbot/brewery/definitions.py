# encoding: utf-8

import os
import math
import platform


########################################################################
# Constants

# Brewery execution host
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
MAIN_ROBOT_UPPER_CANDLE_KICKER_DIST    = 0.334
MAIN_ROBOT_LOWER_CANDLE_KICKER_DIST    = 0.224

# Secondary robot
SECONDARY_ROBOT_X_SIZE                 = 0.172
SECONDARY_ROBOT_Y_SIZE                 = 0.127
SECONDARY_ROBOT_CENTER_X               = 0.0530
SECONDARY_ROBOT_CENTER_Y               = 0.0635
SECONDARY_ROBOT_GYRATION_RADIUS        = 0.161

# Main start positons (the robot starts 90 degrees rotated that's why *_START_Y use ROBOT_X_SIZE and ROBOT_CENTER_X)
MAIN_YELLOW_START_Y                    = MAIN_ROBOT_CENTER_X
MAIN_YELLOW_START_X                    = 1.0
MAIN_YELLOW_START_ANGLE                = math.pi / 2.0
MAIN_RED_START_X                       = MAIN_YELLOW_START_X
MAIN_RED_START_Y                       = FIELD_Y_SIZE - MAIN_YELLOW_START_Y
MAIN_RED_START_ANGLE                   = -math.pi / 2.0

# Secondary start positons (the robot starts 90 degrees rotated that's why *_START_Y use ROBOT_X_SIZE and ROBOT_CENTER_X)
SECONDARY_YELLOW_START_Y               = SECONDARY_ROBOT_CENTER_X
SECONDARY_YELLOW_START_X               = 1.450
SECONDARY_YELLOW_START_ANGLE           = math.pi / 2.0
SECONDARY_RED_START_X                  = SECONDARY_YELLOW_START_X
SECONDARY_RED_START_Y                  = FIELD_Y_SIZE - SECONDARY_YELLOW_START_Y
SECONDARY_RED_START_ANGLE              = -math.pi / 2.0

def setup_definitions(is_main_robot):
    globals()["IS_MAIN_ROBOT"]         = is_main_robot
    globals()["ROBOT_X_SIZE"]          = MAIN_ROBOT_X_SIZE          if IS_MAIN_ROBOT else SECONDARY_ROBOT_X_SIZE
    globals()["ROBOT_Y_SIZE"]          = MAIN_ROBOT_Y_SIZE          if IS_MAIN_ROBOT else SECONDARY_ROBOT_Y_SIZE
    globals()["ROBOT_CENTER_X"]        = MAIN_ROBOT_CENTER_X        if IS_MAIN_ROBOT else SECONDARY_ROBOT_CENTER_X
    globals()["ROBOT_CENTER_Y"]        = MAIN_ROBOT_CENTER_Y        if IS_MAIN_ROBOT else SECONDARY_ROBOT_CENTER_Y
    globals()["ROBOT_GYRATION_RADIUS"] = MAIN_ROBOT_GYRATION_RADIUS if IS_MAIN_ROBOT else SECONDARY_ROBOT_GYRATION_RADIUS
    globals()["YELLOW_START_Y"]        = MAIN_YELLOW_START_Y        if IS_MAIN_ROBOT else SECONDARY_YELLOW_START_Y
    globals()["YELLOW_START_X"]        = MAIN_YELLOW_START_X        if IS_MAIN_ROBOT else SECONDARY_YELLOW_START_X
    globals()["YELLOW_START_ANGLE"]    = MAIN_YELLOW_START_ANGLE    if IS_MAIN_ROBOT else SECONDARY_YELLOW_START_ANGLE
    globals()["RED_START_Y"]           = MAIN_RED_START_Y           if IS_MAIN_ROBOT else SECONDARY_RED_START_Y
    globals()["RED_START_X"]           = MAIN_RED_START_X           if IS_MAIN_ROBOT else SECONDARY_RED_START_X
    globals()["RED_START_ANGLE"]       = MAIN_RED_START_ANGLE       if IS_MAIN_ROBOT else SECONDARY_RED_START_ANGLE

# Rule specific
MATCH_DURATION_MS                      = 90000
FUNNY_ACTION_DURATION_MS               = 10000
FULL_DURATION_MS                       = MATCH_DURATION_MS + FUNNY_ACTION_DURATION_MS
BREWERY_LIFETIME_MS                    = FULL_DURATION_MS + 5000
TEAM_COLOR_RED                         = "#e91009"
TEAM_COLOR_YELLOW                      = "#ffb901"

# Timing
KEEP_ALIVE_DELAY_MS                    = 250
KEEP_ALIVE_MINIMUM_AGE_S               = (KEEP_ALIVE_DELAY_MS * 4.0 / 5.0) / 1000.0
EVENT_LOOP_TICK_RESOLUTION_S           = 0.05

#Teammate collaboration
TEAMMATE_INFO_DELAY_S                  = 1.0 #Time between two position information sent to teammate

# Remote device connection
if IS_HOST_DEVICE_ARM:
    REMOTE_IP                          = "pic"
else:
    REMOTE_IP                          = "localhost"
REMOTE_PORT                            = 7001
REMOTE_LOG_PORT                        = 23

if IS_HOST_DEVICE_ARM:
    MAIN_INTERBOT_IP                   = "doc"
else:
    MAIN_INTERBOT_IP                   = "localhost"
MAIN_INTERBOT_PORT                     = 7002

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
LOG_DIR                                = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "logs")

# Brewery's web sever
WEB_SERVER_PORT                        = 8080

# Use pathfinding algorithm to evaluate the best goal
GOAL_EVALUATION_USES_PATHFINDING       = False

# Pathfinding
MAIN_OPPONENT_AVOIDANCE_RANGE          = 0.5
SECONDARY_OPPONENT_AVOIDANCE_RANGE     = 0.4

# Opponent detection
OPPONENT_DETECTION_DISAPEARING_MS      = 800

# Blocking opponent handling
DEFAULT_OPPONENT_WAIT_MS               = 2000
DEFAULT_OPPONENT_DISAPPEAR_RETRIES     = -1

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
    TEAM_YELLOW  = 0,
    TEAM_RED     = 1,
    TEAM_UNKNOWN = 2,
)

DIRECTION = Enum("Direction",
    DIRECTION_AUTO      =  0,
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

SERVO_TYPE = Enum("Servo type",
    SERVO_TYPE_AX = 0,
    SERVO_TYPE_RX = 1,
)

ACTION = Enum("Action",
    ACTION_OFF = 0,
    ACTION_ON  = 1,
)

TRAJECTORY = Enum("Navigation result",
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

GOAL_STATUS = Enum("Goal status",
    GOAL_AVAILABLE  = 0,
    GOAL_DOING      = 1,
    GOAL_DONE       = 2,
    GOAL_FAILED     = 3,
)
