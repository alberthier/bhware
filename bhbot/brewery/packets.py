# encoding: utf-8

import math
import inspect
import types
import struct
import sys

import position
from binarizer import *
from definitions import *




# Atari 256 color palette
COLORS = ["#000000", "#412000", "#451904", "#5d1f0c", "#4a1700", "#490036", "#48036c", "#051e81", "#0b0779", "#1d295a", "#004b59", "#004800", "#164000", "#2c3500", "#463a09", "#401a02",
          "#252525", "#542800", "#721e11", "#7a240d", "#721f00", "#66004b", "#5c0488", "#0626a5", "#201c8e", "#1d3876", "#005d6e", "#005400", "#1c5300", "#384400", "#4d3f09", "#581f05",
          "#343434", "#763700", "#9f241e", "#982c0e", "#a81300", "#80035f", "#650d90", "#082fca", "#3531a3", "#1d4892", "#006f84", "#036b03", "#236600", "#445200", "#544509", "#702408",
          "#4e4e4e", "#9a5000", "#b33a20", "#b02f0f", "#c8210a", "#950f74", "#7b23a7", "#263dd4", "#4642b4", "#1d5cac", "#00849c", "#0e760e", "#287800", "#495600", "#6c5809", "#8d3a13",
          "#686868", "#c36806", "#c85120", "#bf3624", "#df2512", "#aa2288", "#933bbf", "#444cde", "#5753c5", "#1d71c6", "#0099bf", "#188018", "#2e8c00", "#607100", "#907609", "#ab511f",
          "#757575", "#e47b07", "#e36920", "#d34e2a", "#ec3b24", "#ba3d99", "#9d45c9", "#4f5aec", "#615dcf", "#3286cf", "#00abca", "#279227", "#3a980c", "#6c7f00", "#ab8b0a", "#b56427",
          "#8e8e8e", "#ff911a", "#fc8120", "#e7623e", "#fa5236", "#ca4da9", "#a74fd3", "#5a68ff", "#6d69db", "#489bd9", "#00bcde", "#36a436", "#47a519", "#798d0a", "#c1a120", "#bf7730",
          "#a4a4a4", "#ffab1d", "#fd8c25", "#f36e4a", "#fc6148", "#d75ab6", "#b25ade", "#6575ff", "#7b77e9", "#4ea8ec", "#00d0f5", "#4eb94e", "#51af23", "#8b9f1c", "#d0b02f", "#d0853a",
          "#b8b8b8", "#ffc51f", "#fe982c", "#fd7854", "#ff705f", "#e467c3", "#bd65e9", "#7183ff", "#8985f7", "#55b6ff", "#10dcff", "#51cd51", "#5cba2e", "#9eb22f", "#debe3d", "#e19344",
          "#c5c5c5", "#ffd03b", "#ffae38", "#ff8a6a", "#ff7e7e", "#ef72ce", "#c56df1", "#8091ff", "#918dff", "#69caff", "#3ee1ff", "#72da72", "#71cf43", "#abbf3c", "#e6c645", "#eda04e",
          "#d0d0d0", "#ffd84c", "#ffb946", "#ff987c", "#ff8f8f", "#fb7eda", "#ce76fa", "#90a0ff", "#9c98ff", "#74cbff", "#64e7ff", "#7ce47c", "#85e357", "#b8cc49", "#edcd4c", "#f9ad58",
          "#d7d7d7", "#ffe651", "#ffbf51", "#ffa48b", "#ff9d9e", "#ff8de1", "#d583ff", "#97a9ff", "#a7a4ff", "#82d3ff", "#76eaff", "#85ed85", "#8deb5f", "#c2d653", "#f5d862", "#fcb75c",
          "#e1e1e1", "#fff456", "#ffc66d", "#ffb39e", "#ffabad", "#ff9de5", "#da90ff", "#9fb2ff", "#b2afff", "#8ddaff", "#8bedff", "#99f299", "#97f569", "#cde153", "#fbe276", "#ffc160",
          "#eaeaea", "#fff970", "#ffd587", "#ffc2b2", "#ffb9bd", "#ffa5e7", "#de9cff", "#afbeff", "#bbb8ff", "#9fd4ff", "#9aefff", "#b3f7b3", "#a0fe72", "#dbef6c", "#fcee98", "#ffca69",
          "#f4f4f4", "#ffff90", "#ffe498", "#ffd0c3", "#ffc7ce", "#ffafea", "#e2a9ff", "#c0cbff", "#c3c1ff", "#b4e2ff", "#b1f3ff", "#c3f9c3", "#b1ff8a", "#e8fc79", "#fdf3a9", "#ffcf7e",
          "#ffffff", "#ffffaa", "#ffe6ab", "#ffdad0", "#ffcade", "#ffb8ec", "#e6b6ff", "#cdd3ff", "#d3d1ff", "#c0ebff", "#c7f6ff", "#cdfccd", "#bcff9a", "#f2ffab", "#fdf3be", "#ffda96"]




################################################################################
# Specific item types




class ShortAsFloat(Float):

    C_TYPE = 'h'
    DESCRIPTION = "4 bytes real (float) stored in a short"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)


    def serialize(self, value, buf):
        buf.append(int(value * 10000.0))


    def deserialize(self, iterator):
        return float(next(iterator)) / 10000.0




class Point(Struct):

    DESCRIPTION = "Field coordinates"

    def __init__(self, description = None):
        Struct.__init__(self, position.Pose, description,
            ('x', ShortAsFloat(0.0, "X coordinate")),
            ('y', ShortAsFloat(0.0, "Y coordinate")),
        )




class Pose(Struct):

    DESCRIPTION = "Pose"

    def __init__(self, description = None):
        Struct.__init__(self, position.Pose, description,
            ('x',     Float(0.0, "X coordinate")),
            ('y',     Float(0.0, "Y coordinate")),
            ('angle', Float(0.0, "Angle")),
        )




class OptionalAngle(AbstractItem):

    C_TYPE = 'Bf'
    DESCRIPTION = "Optional angle"

    def __init__(self, default_value, description = None):
        AbstractItem.__init__(self, default_value, description)


    def serialize(self, value, buf):
        if value is not None:
            buf.append(1)
            buf.append(value)
        else:
            buf.append(0)
            buf.append(-1100000.0)


    def deserialize(self, iterator):
        use_angle = next(iterator)
        angle = next(iterator)
        if not use_angle:
            angle = None
        return angle


    def to_dump(self, value):
        if value is None:
            return str(None)
        return "{:0.4f}".format(value)




################################################################################
# Base packet class




class BasePacket(object):

    MAX_SIZE = 256
    DEFINITION = ()
    DESCRIPTION = ""
    LOGVIEW_DEFAULT_ENABLED = True
    STRUCT = None
    BIN_STRUCT = None
    HANDLER_METHODS = None

    @classmethod
    def static_init(cls):
        if cls.STRUCT is None:
            cls.BIN_STRUCT = Struct(StructInstance, "", *cls.DEFINITION)
            fmt = "<B" + cls.BIN_STRUCT.C_TYPE
            size = struct.calcsize(fmt)
            pad_size = cls.MAX_SIZE - size
            if pad_size > 0:
                fmt += str(pad_size) + "x"
            cls.STRUCT = struct.Struct(fmt)

        if cls.HANDLER_METHODS is None:
            packet_method = "on"
            for c in cls.__name__:
                if c.isupper():
                    packet_method += "_" + c.lower()
                else:
                    packet_method += c
            cls.HANDLER_METHODS = [ packet_method, 'on_packet' ]

    @property
    def name(self):
        return self.__class__.__name__


    def __init__(self, *args, **kwargs):
        for name, item in self.DEFINITION:
            value = None
            for aname, avalue in args:
                if name == aname:
                    value = avalue
                    break
            if value is None:
                if name in kwargs:
                    value = kwargs[name]
                else:
                    # Call the constructor of the value to duplicate it. This is necessary for lists
                    value = copy.deepcopy(item.default_value)
            setattr(self, name, value)


    def serialize(self):
        args = [ self.TYPE ]
        self.BIN_STRUCT.serialize(self, args)
        try :
            return self.STRUCT.pack(*args)
        except Exception as e :
            raise Exception("Error while serializing packet of type {} : {}".format(self.name, e))


    def deserialize(self, buf):
        unpacked = self.STRUCT.unpack(buf)
        it = iter(unpacked)
        # pop the type
        next(it)
        self.BIN_STRUCT.deserialize_to(self, it)


    def to_dump(self):
        return self.BIN_STRUCT.to_dump(self)


    def dispatch_generator(self, obj):
        for method in self.HANDLER_METHODS:
            if hasattr(obj, method):
                g = getattr(obj, method)(self)
                if isinstance(g, types.GeneratorType):
                    yield from g


    def dispatch(self, obj):
        for method in self.HANDLER_METHODS:
            if hasattr(obj, method):
                getattr(obj, method)(self)


################################################################################
# Packet type ranges

TURRET_RANGE_START    = 32
TURRET_RANGE_END      = 50
PIC32_RANGE_START     = TURRET_RANGE_END
PIC32_RANGE_END       = 150
SIMULATOR_RANGE_START = PIC32_RANGE_END
SIMULATOR_RANGE_END   = 200
INTERBOT_RANGE_START  = SIMULATOR_RANGE_END
INTERBOT_RANGE_END    = 230
INTERNAL_RANGE_START  = INTERBOT_RANGE_END
INTERNAL_RANGE_END    = 256

################################################################################
# Packet classes


# Turret packets

class TurretDetect(BasePacket):

    MAX_SIZE = 4
    TYPE = 32
    DEFINITION = (
        ('distance', UInt8 (0, "Detection distance")),
        ('angle'   , UInt8 (0, "Detection angle index (0 <= angle <= 17; 20 deg resolution)")),
        ('robot'   , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
    )




class TurretInit(BasePacket):

    MAX_SIZE = 4
    TYPE = 33
    DEFINITION = (
        ('mode'          , UEnum8(TURRET_INIT_MODE, TURRET_INIT_MODE_READ)),
        ('short_distance', UInt8 (0, "Short distance detection range")),
        ('long_distance' , UInt8 (0, "Long distance detection range")),
    )




class TurretDistances(BasePacket):

    MAX_SIZE = 3
    TYPE = 34
    DEFINITION = (
        ('short_distance', UInt8 (0, "Short distance detection range")),
        ('long_distance' , UInt8 (0, "Long distance detection range")),
    )




class TurretBoot(BasePacket):

    MAX_SIZE = 1
    TYPE = 35


# PIC 32 packets


class Reinitialize(BasePacket):

    TYPE = 50




class ControllerReady(BasePacket):

    TYPE = 51
    LOGVIEW_DEFAULT_ENABLED = True




class DeviceBusy(BasePacket):

    TYPE = 52
    DEFINITION = (
        ('remote_device', UEnum8(REMOTE_DEVICE, REMOTE_DEVICE_PIC)),
    )




class DeviceReady(BasePacket):

    TYPE = 53
    DEFINITION = (
        ('team',          UEnum8(TEAM         , TEAM_UNKNOWN     )),
        ('remote_device', UEnum8(REMOTE_DEVICE, REMOTE_DEVICE_PIC)),
    )




class Start(BasePacket):

    TYPE = 54
    DEFINITION = (
        ('team', UEnum8(TEAM, TEAM_UNKNOWN)),
    )




class Rotate(BasePacket):

    TYPE = 55
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_AUTO)),
        ('angle'    , Float(0.0, "Destination angle")),
    )




class MoveCurve(BasePacket):

    TYPE = 56
    DEFINITION = (
        ('direction', Enum8        (DIRECTION, DIRECTION_FORWARDS)),
        ('angle'    , OptionalAngle(None, "Destination angle")),
        ('points'   , List         (62, Point(), [], "List of points to follow")),
    )




class MoveLine(BasePacket):

    TYPE = 57
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_FORWARDS)),
        ('points'   , List (63, Point(), [], "List of points to follow")),
    )




class MoveArc(BasePacket):

    TYPE = 58
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_FORWARDS)),
        ('center'   , Point()),
        ('radius'   , Float(0.0, "Arc radius")),
        ('points'   , List (61, Float(0.0), [], "List of points to follow")),
    )




class GotoStarted(BasePacket):

    TYPE = 59




class WaypointReached(BasePacket):

    TYPE = 60

    DEFINITION = (
        ('current_point_index', UInt8(0, "Reached waypoint index")),
        ('current_pose' ,       Pose ("Current robot pose")),
    )




class GotoFinished(BasePacket):

    TYPE = 61
    DEFINITION = (
        ('reason'             , UEnum8(REASON, REASON_DESTINATION_REACHED)),
        ('current_pose'       , Pose  ("Robot pose at the end of the movement")),
        ('current_point_index', UInt8 (0, "Last reached point index of the point list given in the Goto packet")),
    )




class EnableAntiBlocking(BasePacket):

    TYPE = 62




class DisableAntiBlocking(BasePacket):

    TYPE = 63




class KeepAlive(BasePacket):

    TYPE = 64
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('current_pose' , Pose  ("Current robot pose")),
        ('match_started', Bool  (False, "Flag defining if the match has already started")),
        ('match_time'   , UInt32(0, "Time elapsed since the start of the match")),
    )




class PositionControlConfig(BasePacket):

    TYPE = 65
    DEFINITION = (
        ('ratio_acc'     , Float(0.0)),
        ('ratio_decc'    , Float(0.0)),
        ('ratio_acc_rot' , Float(0.0)),
        ('ratio_decc_rot', Float(0.0)),
        ('vmax_limit'    , Float(0.0)),
    )




class Stop(BasePacket):

    TYPE = 66




class Resettle(BasePacket):

    TYPE = 67
    DEFINITION = (
        ('axis'    , UEnum8     (AXIS, AXIS_X)),
        ('position', Float      (0.0, "Robot position on the given axis")),
        ('angle'   , FloatRadian(0.0, "Robot angle")),
    )




class StopAll(BasePacket):

    TYPE = 68




class ServoControl(BasePacket):

    TYPE = 69
    DEFINITION = (
        ('type',  UEnum8(SERVO_TYPE, SERVO_TYPE_AX)),
        ('id',    UInt8 (0, "Servo identifier")),
        ('angle', UInt16(0, "Destination angle")),
    )




class ElectromagnetControl(BasePacket):

    TYPE = 70
    DEFINITION = (
        ('id',     UInt8 (0, "Servo identifier")),
        ('action', UEnum8(ACTION, ACTION_OFF)),
    )



class SuctionPump(BasePacket):

    TYPE = 71
    DEFINITION = (
        ('action', UEnum8(ACTION, ACTION_OFF)),
    )


# Simulator


class SimulatorData(BasePacket):

    TYPE = 150
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('leds', UInt8(0, "Dockstar leds status")),
    )




class SimulatorClearGraphMapZones(BasePacket):

    TYPE = 151
    LOGVIEW_DEFAULT_ENABLED = False




class SimulatorAddGraphMapZone(BasePacket):

    TYPE = 152
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('id'    , UInt8(0, "Zone id")),
        ('points', List (63, Float(0.0), [], "Points")),
    )




class SimulatorEnableGraphMapZone(BasePacket):

    TYPE = 153
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('id'     , UInt8(0, "Zone id")),
        ('enabled', Bool (True, "Zone status")),
    )



class SimulatorMoveGraphMapZone(BasePacket):

    TYPE = 154
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('id'     , UInt8(0, "Zone id")),
        ('dx'     , Float(0.0, "X coordinate")),
        ('dy'     , Float(0.0, "Y coordinate")),
    )




class SimulatorClearGraphMapEdges(BasePacket):

    TYPE = 155
    LOGVIEW_DEFAULT_ENABLED = False




class SimulatorGraphMapEdges(BasePacket):

    TYPE = 156
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('points', List(63, Float(0.0), [], "Edges")),
    )




class SimulatorGraphMapRoute(BasePacket):

    TYPE = 157
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('points', List(63, Float(0.0), [], "Edges")),
    )


# Interbot


class InterbotPosition(BasePacket):

    TYPE = 201

    DEFINITION = (
        ('pose', Pose("Other robot pose")),
    )




class InterbotGoalStatus(BasePacket):

    TYPE = 202

    DEFINITION = (
        ('goal_identifier', String(32, "", "Goal identifier")),
        ('goal_status'    , UInt8 (0, "Goal status")),
    )


# Internal


class OpponentPosition(BasePacket):

    TYPE = 230

    DEFINITION = (
        ('robot'    , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
        ('x'        , Float(0.0, "Opponent estimated X coordinate")),
        ('y'        , Float(0.0, "Opponent estimated Y coordinate")),
    )




class OpponentDetected(BasePacket):

    TYPE = 231

    DEFINITION = (
        ('robot'    , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
        ('direction', UEnum8(DIRECTION, DIRECTION_FORWARDS)),
        ('x'        , Float(0.0, "Opponent estimated X coordinate")),
        ('y'        , Float(0.0, "Opponent estimated Y coordinate")),
    )




class OpponentDisappeared(BasePacket):

    TYPE = 232

    DEFINITION = (
        ('robot'    , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
        ('direction', UEnum8(DIRECTION, DIRECTION_FORWARDS)),
    )


################################################################################
# Packets lookup setup


PACKETS_BY_NAME = {}
PACKETS_BY_TYPE = {}
PACKETS_LIST = []


for (item_name, item_type) in inspect.getmembers(sys.modules[__name__]):
    if inspect.isclass(item_type) and issubclass(item_type, BasePacket) and item_type != BasePacket:
        # Create a packet instance a first time to finish the setup
        item_type.static_init()
        assert item_name not in PACKETS_BY_NAME
        PACKETS_BY_NAME[item_name] = item_type
        assert item_type.TYPE not in PACKETS_BY_TYPE
        PACKETS_BY_TYPE[item_type.TYPE] = item_type
    PACKETS_LIST = list(PACKETS_BY_TYPE.values())


def create_packet(buffer):
    (packet_type,) = struct.unpack("<B", buffer[:1])
    packet_class = PACKETS_BY_TYPE[packet_type]
    return packet_class()
