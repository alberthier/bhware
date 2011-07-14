#!/usr/bin/env python
# encoding: utf-8


import math


########################################################################
# Constants


FIELD_WIDTH = 3.0
FIELD_HEIGHT= 2.1
FIELD_CELL_SIZE = 0.350
FIELD_CELLS_COUNT = 6
FIELD_GREEN_ZONE_WIDTH = 0.450
PIECE_RADIUS = 0.2
ROBOT_WIDTH = 0.337
ROBOT_HEIGHT = 0.256
ROBOT_CENTER_X = ROBOT_WIDTH / 2.0
ROBOT_CENTER_Y = 0.054
ROBOT_CENTER_TO_LATERAL_SENSOR_DISTANCE = 0.150 - ROBOT_CENTER_Y
ROBOT_CENTER_TO_HACKED_SENSOR_DISTANCE = 0.247 - ROBOT_CENTER_Y
ROBOT_EMPTY_GYRATION_RADIUS = 0.288
ROBOT_WITH_PIECE_GYRATION_RADIUS = 0.325
RED_START_X = ROBOT_CENTER_X + 0.052
RED_START_Y = ROBOT_CENTER_Y + 0.120
RED_START_ANGLE = math.pi / 2.0
BLUE_START_X = ROBOT_CENTER_X + 0.052
BLUE_START_Y = FIELD_WIDTH - ROBOT_CENTER_Y - 0.120
BLUE_START_ANGLE = -math.pi / 2.0
OPPONENT_DETECTION_ANGLE = math.pi / 6.0
OPPONENT_DETECTION_DISAPEARING_KEEP_ALIVE_TICKS = 4
KEEP_ALIVE_DELAY_MS = 300
SICK_DETECTION_THRESHOLD = 0.030
TAKE_FIGURE_Y = 0.250


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
    REMOTE_DEVICE_PIC = 0,
    REMOTE_DEVICE_SIMULATOR = 1,
    REMOTE_DEVICE_UNKNOWN = 2,
)

Enum("HOST_DEVICE",
    "Host hardware type",
    HOST_DEVICE_ARM = 0,
    HOST_DEVICE_PC = 1,
)

Enum("TEAM",
    "Team color",
    TEAM_BLUE = 0,
    TEAM_RED  = 1,
    TEAM_UNKNOWN = 2,
)

Enum("MOVEMENT",
    "Movement",
    MOVEMENT_ROTATE = 0,
    MOVEMENT_MOVE = 1,
    MOVEMENT_LINE = 2
)

Enum("DIRECTION",
    "Direction",
    DIRECTION_FORWARD = 1,
    DIRECTION_BACKWARD = -1,
)

Enum("ANGLE",
    "Angle",
    ANGLE_N = math.pi / 2.0,
    ANGLE_NW = 3.0 * math.pi / 4.0,
    ANGLE_W = math.pi,
    ANGLE_SW = 5.0 * math.pi / 4.0,
    ANGLE_S = 3.0 * math.pi / 2.0,
    ANGLE_SE = 7.0 * math.pi / 4.0,
    ANGLE_E = 0.0
)


Enum("REASON",
    "Goto finished reason",
    REASON_DESTINATION_REACHED = 0,
    REASON_PIECE_FOUND = 1,
)

Enum("BLOCKING",
    "Blocking side",
    BLOCKED_FRONT = 1,
    BLOCKED_BACK = -1,
)

Enum("AXIS",
    "Axis",
    AXIS_X = 0,
    AXIS_Y = 1,
)

Enum("SENSOR",
    "Piece sensor",
    SENSOR_LEFT_BOTTOM = 3,
    SENSOR_LEFT_TOP = 2,
    SENSOR_RIGHT_BOTTOM = 1,
    SENSOR_RIGHT_TOP = 0,
    SENSOR_CENTER = 4,
    SENSOR_NONE = 5,
)

Enum("NODE",
    "Node color",
    NODE_BLUE = 0,
    NODE_RED = 1,
    NODE_GREEN = 2,
    NODE_UNKNOWN = 0,
)

Enum("TRAJECTORY_WALK",
    "Trajectory walk result",
    TRAJECTORY_WALK_DESTINATION_REACHED = 0,
    TRAJECTORY_WALK_PIECE_FOUND = 1,
    TRAJECTORY_WALK_BLOCKED = 2,
    TRAJECTORY_WALK_OPPONENT_DETECTED = 3,
)
