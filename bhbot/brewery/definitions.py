#!/usr/bin/env python
# encoding: utf-8

__lookup={}
__descriptions={}

def enum(name, description, **kwargs) :
    __descriptions[name]=description
    for k,v in kwargs.items() :
        globals()[k]=v
        __lookup.setdefault(name,{})[v]=k

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
)

enum("DIRECTION",
    "Direction",
    DIRECTION_FORWARD = 1,
    DIRECTION_BACKWARD = -1,
)

enum("REASON",
    "Goto finished reason",
    REASON_DESTINATION_REACHED = 0,
    REASON_PAWN_FOUND = 1,
    REASON_QUEEN_FOUND = 2,
    REASON_KING_FOUND = 3,
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

enum("PIECE_SENSOR",
    "Lateral piece sensor",
    LATERAL_SENSOR_NONE = 0,
    LATERAL_SENSOR_PAWN = 1,
    LATERAL_SENSOR_KING_OR_QWEEN = 2,
)

####### ADD DEFINITIONS ABOVE ###############

def get_defs_types() : return __lookup.keys()

def lookup_defs(_type,_value) :
    return __lookup[_type][_value]

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
