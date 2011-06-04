#!/usr/bin/env python
# encoding: utf-8

import math

__lookup={}
__descriptions={}

def enum(name, description, **kwargs) :
    __descriptions[name]=description
    for k,v in kwargs.items() :
        globals()[k]=v
        __lookup.setdefault(name,{})[v]=k


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


enum("REMOTE_DEVICE",
    "Remote hardware type",
    REMOTE_DEVICE_PIC = 0,
    REMOTE_DEVICE_SIMULATOR = 1,
    REMOTE_DEVICE_UNKNOWN = 2,
)

enum("HOST_DEVICE",
    "Host hardware type",
    HOST_DEVICE_ARM = 0,
    HOST_DEVICE_PC = 1,
)

enum("TEAM",
    "Team color",
    TEAM_BLUE = 0,
    TEAM_RED  = 1,
    TEAM_UNKNOWN = 2,
)

enum("MOVEMENT",
    "Movement",
    MOVEMENT_ROTATE = 0,
    MOVEMENT_MOVE = 1,
    MOVEMENT_LINE = 2
)

enum("DIRECTION",
    "Direction",
    DIRECTION_FORWARD = 1,
    DIRECTION_BACKWARD = -1,
)

enum("ANGLE",
    "Angle",
    ANGLE_N = math.pi/2,
    ANGLE_NW = 3*math.pi/4,
    ANGLE_W = math.pi,
    ANGLE_SW = 5*math.pi/4,
    ANGLE_S = 3*math.pi/2,
    ANGLE_SE = 7*math.pi/4,
    ANGLE_E = 0.0
)


enum("REASON",
    "Goto finished reason",
    REASON_DESTINATION_REACHED = 0,
    REASON_PIECE_FOUND = 1,
)

enum("BLOCKING",
    "Blocking side",
    BLOCKED_FRONT = 1,
    BLOCKED_BACK = -1,
)

enum("AXIS",
    "Axis",
    AXIS_ABSCISSA = 0,
    AXIS_ORDINATE = 1,
)

enum("SENSOR",
    "Piece sensor",
    SENSOR_LEFT_BOTTOM = 3,
    SENSOR_LEFT_TOP = 2,
    SENSOR_RIGHT_BOTTOM = 1,
    SENSOR_RIGHT_TOP = 0,
    SENSOR_CENTER = 4,
    SENSOR_NONE = 5,
)

enum("NODE",
    "Node color",
    NODE_BLUE = 0,
    NODE_RED = 1,
    NODE_GREEN = 2,
    NODE_UNKNOWN = 0,
)

enum("TRAJECTORY_WALK",
    "Trajectory walk result",
    TRAJECTORY_WALK_DESTINATION_REACHED = 0,
    TRAJECTORY_WALK_PIECE_FOUND = 1,
    TRAJECTORY_WALK_BLOCKED = 2,
    TRAJECTORY_WALK_OPPONENT_DETECTED = 3,
)

####### ADD DEFINITIONS ABOVE ###############

def get_defs_types() : return __lookup.keys()

def lookup_defs(_type,_value) :
    return __lookup[_type].get(_value,"--- UNKNOWN ---")

def dump_defs() :
    print as_string()

def as_string() :
    ret=""
    for k, v in __lookup.items() :
        ret+=k +" "+ __descriptions[k] + "\n"
        for w,x in v.items() :
            ret+="\t" + str(w) +"\t" + str(x) + "\n"
    return ret

def as_dict() : return __lookup.items()


del enum

if __name__ == "__main__" :
    dump_defs()
    print lookup_defs("REMOTE_DEVICE",1)
