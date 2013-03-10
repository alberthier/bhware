# encoding: utf-8

import sys
import struct
import inspect
import math

from binarizer import *
from definitions import *
import position




# Mozilla Colors (http://colors.bravo9.com/mozilla/)
# Used  Color   Name                        Used  Color   Name
#      #f0f8ff aliceblue                         #ffa07a lightsalmon
#      #faebd7 antiquewhite                   X  #20b2aa lightseagreen
#      #00ffff aqua                              #87cefa lightskyblue
#   X  #7fffd4 aquamarine                        #778899 lightslategray
#      #f0ffff azure                             #778899 lightslategrey
#      #f5f5dc beige                             #ffffe0 lightyellow
#      #ffe4c4 bisque                            #00ff00 lime
#      #000000 black                          X  #32cd32 limegreen
#      #0000ff blue                              #faf0e6 linen
#      #8a2be2 blueviolet                     X  #ff00ff magenta
#   X  #a52a2a brown                             #800000 maroon
#      #deb887 burlywood                         #66cdaa mediumaquamarine
#   X  #5f9ea0 cadetblue                         #0000cd mediumblue
#   X  #7fff00 chartreuse                        #ba55d3 mediumorchid
#   X  #d2691e chocolate                         #9370db mediumpurple
#   X  #ff7f50 coral                             #3cb371 mediumseagreen
#      #6495ed cornflowerblue                    #7b68ee mediumslateblue
#      #fff8dc cornsilk                          #00fa9a mediumspringgreen
#      #dc143c crimson                           #48d1cc mediumturquoise
#   X  #00ffff cyan                              #c71585 mediumvioletred
#   X  #00008b darkblue                       X  #191970 midnightblue
#      #008b8b darkcyan                          #f5fffa mintcream
#      #b8860b darkgoldenrod                     #ffe4e1 mistyrose
#      #a9a9a9 darkgray                          #ffe4b5 moccasin
#      #006400 darkgreen                         #ffdead navajowhite
#      #a9a9a9 darkgrey                          #000080 navy
#      #bdb76b darkkhaki                         #fdf5e6 oldlace
#   X  #8b008b darkmagenta                       #808000 olive
#   X  #556b2f darkolivegreen                    #6b8e23 olivedrab
#      #ff8c00 darkorange                     X  #ffa500 orange
#      #9932cc darkorchid                     X  #ff4500 orangered
#      #8b0000 darkred                           #da70d6 orchid
#      #e9967a darksalmon                        #eee8aa palegoldenrod
#      #8fbc8f darkseagreen                   X  #98fb98 palegreen
#      #483d8b darkslateblue                     #afeeee paleturquoise
#      #2f4f4f darkslategray                  X  #db7093 palevioletred
#      #2f4f4f darkslategrey                     #ffefd5 papayawhip
#      #9400d3 darkviolet                        #ffdab9 peachpuff
#   X  #ff1493 deeppink                          #cd853f peru
#      #00bfff deepskyblue                       #ffc0cb pink
#      #696969 dimgray                           #dda0dd plum
#      #696969 dimgrey                           #b0e0e6 powderblue
#      #1e90ff dodgerblue                        #800080 purple
#   X  #b22222 firebrick                         #ff0000 red
#      #fffaf0 floralwhite                       #bc8f8f rosybrown
#      #228b22 forestgreen                    X  #4169e1 royalblue
#      #ff00ff fuchsia                        X  #8b4513 saddlebrown
#      #dcdcdc gainsboro                         #fa8072 salmon
#      #f8f8ff ghostwhite                        #f4a460 sandybrown
#   X  #ffd700 gold                              #2e8b57 seagreen
#      #daa520 goldenrod                         #fff5ee seashell
#      #808080 gray                              #a0522d sienna
#      #008000 green                             #c0c0c0 silver
#      #adff2f greenyellow                       #87ceeb skyblue
#      #808080 grey                           X  #6a5acd slateblue
#      #f0fff0 honeydew                          #708090 slategray
#      #ff69b4 hotpink                           #708090 slategrey
#      #cd5c5c indianred                         #fffafa snow
#      #4b0082 indigo                            #00ff7f springgreen
#      #fffff0 ivory                          X  #4682b4 steelblue
#      #f0e68c khaki                             #d2b48c tan
#      #e6e6fa lavender                          #008080 teal
#      #fff0f5 lavenderblush                  X  #d8bfd8 thistle
#      #7cfc00 lawngreen                      X  #ff6347 tomato
#      #fffacd lemonchiffon                   X  #40e0d0 turquoise
#      #add8e6 lightblue                         #ee82ee violet
#      #f08080 lightcoral                        #f5deb3 wheat
#      #e0ffff lightcyan                         #ffffff white
#      #fafad2 lightgoldenrodyellow              #f5f5f5 whitesmoke
#      #90ee90 lightgreen                        #ffff00 yellow
#      #d3d3d3 lightgrey                      X  #9acd32 yellowgreen
#      #ffb6c1 lightpink




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




class SimulatorPoint(Struct):

    DESCRIPTION = "Route point"

    def __init__(self, description = None):
        Struct.__init__(self, StructInstance, description,
            ('x', UInt8 (0, "X coordinate")),
            ('y', UInt16(0, "Y coordinate")),
        )




class SimulatorRect(Struct):

    DESCRIPTION = "Route rectangle"

    def __init__(self, description = None):
        Struct.__init__(self, StructInstance, description,
            ('x1', UInt8 (0, "X1 coordinate")),
            ('y1', UInt16(0, "Y1 coordinate")),
            ('x2', UInt8 (0, "X2 coordinate")),
            ('y2', UInt16(0, "Y2 coordinate")),
        )




class SimulatorCircle(Struct):

    DESCRIPTION = "Route circle"

    def __init__(self, description = None):
        Struct.__init__(self, StructInstance, description,
            ('x',      UInt8 (0, "Circle center X coordinate")),
            ('y',      UInt16(0, "Circle center Y coordinate")),
            ('radius', UInt8 (0, "Circle radius")),
        )




################################################################################
# Base packet class




class BasePacket(object):

    MAX_SIZE = 256
    DEFINITION = ()
    DESCRIPTION = ""
    LOGVIEW_COLOR = "#000000"
    LOGVIEW_DEFAULT_ENABLED = True
    STRUCT = None
    BIN_STRUCT = None
    HANDLER_METHOD = None

    def __init__(self, *args, **kwargs):
        cls = type(self)
        if cls.STRUCT is None:
            cls.BIN_STRUCT = Struct(StructInstance, "", *cls.DEFINITION)
            fmt = "<B" + cls.BIN_STRUCT.C_TYPE
            size = struct.calcsize(fmt)
            pad_size = cls.MAX_SIZE - size
            if pad_size > 0:
                fmt += str(pad_size) + "x"
            cls.STRUCT = struct.Struct(fmt)

        if cls.HANDLER_METHOD is None:
            cls.HANDLER_METHOD = "on"
            for c in cls.__name__:
                if c.isupper():
                    cls.HANDLER_METHOD += "_" + c.lower()
                else:
                    cls.HANDLER_METHOD += c

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
        return self.STRUCT.pack(*args)


    def deserialize(self, buf):
        unpacked = self.STRUCT.unpack(buf)
        it = iter(unpacked)
        # pop the type
        next(it)
        self.BIN_STRUCT.deserialize_to(self, it)


    def to_dump(self):
        return self.BIN_STRUCT.to_dump(self)


    def dispatch(self, obj):
        if hasattr(obj, self.HANDLER_METHOD):
            getattr(obj, self.HANDLER_METHOD)(self)
        if hasattr(obj, 'on_packet'):
            getattr(obj, 'on_packet')(self)

    def is_keep_alive(self):
        return False


################################################################################
# Packet classes


# Turret packets

class TurretDetect(BasePacket):

    MAX_SIZE = 4
    TYPE = 32
    LOGVIEW_COLOR = "#9acd32"
    DEFINITION = (
        ('distance', UInt8 (0, "Detection distance")),
        ('angle'   , UInt8 (0, "Detection angle index (0 <= angle <= 17; 20 deg resolution)")),
        ('robot'   , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
    )




class TurretInit(BasePacket):

    MAX_SIZE = 4
    TYPE = 33
    LOGVIEW_COLOR = "#7fffd4"
    DEFINITION = (
        ('mode'          , UEnum8(TURRET_INIT_MODE, TURRET_INIT_MODE_READ)),
        ('short_distance', UInt8 (0, "Short distance detection range")),
        ('long_distance' , UInt8 (0, "Long distance detection range")),
    )




class TurretDistances(BasePacket):

    MAX_SIZE = 3
    TYPE = 34
    LOGVIEW_COLOR = "#191970"
    DEFINITION = (
        ('short_distance', UInt8 (0, "Short distance detection range")),
        ('long_distance' , UInt8 (0, "Long distance detection range")),
    )




class TurretBoot(BasePacket):

    MAX_SIZE = 1
    TYPE = 35
    LOGVIEW_COLOR = "#20b2aa"


# PIC 32 packets


class Reinitialize(BasePacket):

    TYPE = 50
    LOGVIEW_COLOR = "#db7093"




class ControllerReady(BasePacket):

    TYPE = 51
    LOGVIEW_DEFAULT_ENABLED = True
    LOGVIEW_COLOR = "#a52a2a"




class DeviceBusy(BasePacket):

    TYPE = 52
    LOGVIEW_COLOR = "#5f9ea0"
    DEFINITION = (
        ('remote_device', UEnum8(REMOTE_DEVICE, REMOTE_DEVICE_PIC)),
    )




class DeviceReady(BasePacket):

    TYPE = 53
    LOGVIEW_COLOR = "#7fff00"
    DEFINITION = (
        ('team',          UEnum8(TEAM         , TEAM_UNKNOWN     )),
        ('remote_device', UEnum8(REMOTE_DEVICE, REMOTE_DEVICE_PIC)),
    )




class Start(BasePacket):

    TYPE = 54
    LOGVIEW_COLOR = "#d2691e"
    DEFINITION = (
        ('team', UEnum8(TEAM, TEAM_UNKNOWN)),
    )




class Rotate(BasePacket):

    TYPE = 55
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_AUTO)),
        ('angle'    , Float(0.0, "Destination angle")),
    )




class MoveCurve(BasePacket):

    TYPE = 56
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        ('direction', Enum8        (DIRECTION, DIRECTION_FORWARD)),
        ('angle'    , OptionalAngle(None, "Destination angle")),
        ('points'   , List         (62, Point(), [], "List of points to follow")),
    )




class MoveLine(BasePacket):

    TYPE = 57
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_FORWARD)),
        ('points'   , List (63, Point(), [], "List of points to follow")),
    )




class MoveArc(BasePacket):

    TYPE = 58
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        ('direction', Enum8(DIRECTION, DIRECTION_FORWARD)),
        ('center'   , Point()),
        ('radius'   , Float(0.0, "Arc radius")),
        ('points'   , List (61, Float(0.0), [], "List of points to follow")),
    )




class GotoStarted(BasePacket):

    TYPE = 59
    LOGVIEW_COLOR = "#6495ed"




class WaypointReached(BasePacket):

    TYPE = 60
    LOGVIEW_COLOR = "#6495ed"




class GotoFinished(BasePacket):

    TYPE = 61
    LOGVIEW_COLOR = "#daa520"
    DEFINITION = (
        ('reason'             , UEnum8(REASON, REASON_DESTINATION_REACHED)),
        ('current_pose'       , Pose  ("Robot pose at the end of the movement")),
        ('current_point_index', UInt8 (0, "Last reached point index of the point list given in the Goto packet")),
    )




class EnableAntiBlocking(BasePacket):

    TYPE = 62
    LOGVIEW_COLOR = "#00ffff"




class DisableAntiBlocking(BasePacket):

    TYPE = 63
    LOGVIEW_COLOR = "#00008b"




class KeepAlive(BasePacket):

    TYPE = 64
    LOGVIEW_COLOR = "#8b008b"
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('current_pose' , Pose  ("Current robot pose")),
        ('match_started', Bool  (False, "Flag defining if the match has already started")),
        ('match_time'   , UInt32(0, "Time elapsed since the start of the match")),
    )

    def is_keep_alive(self):
        return True




class PositionControlConfig(BasePacket):

    TYPE = 65
    LOGVIEW_COLOR = "#556b2f"
    DEFINITION = (
        ('t_acc'   , Float(0.0)),
        ('f_va_max', Float(0.0)),
    )




class Stop(BasePacket):

    TYPE = 66
    LOGVIEW_COLOR = "#b22222"




class Resettle(BasePacket):

    TYPE = 67
    LOGVIEW_COLOR = "#ff1493"
    DEFINITION = (
        ('axis'    , UEnum8     (AXIS, AXIS_X)),
        ('position', Float      (0.0, "Robot position on the given axis")),
        ('angle'   , FloatRadian(0.0, "Robot angle")),
    )




class StopAll(BasePacket):

    TYPE = 68
    LOGVIEW_COLOR = "#ff1493"




class GlassPresent(BasePacket):

    TYPE = 69
    DEFINITION = (
        ('side', UEnum8(SIDE, SIDE_LEFT)),
    )




class Nipper(BasePacket):

    TYPE = 70
    DEFINITION = (
        ('side', UEnum8(SIDE, SIDE_LEFT)),
        ('move', UEnum8(MOVE, MOVE_CLOSE)),
    )




class Lifter(BasePacket):

    TYPE = 71
    DEFINITION = (
        ('side', UEnum8(SIDE       , SIDE_LEFT)),
        ('move', UEnum8(LIFTER_MOVE, LIFTER_MOVE_DOWN)),
    )




class Gripper(BasePacket):

    TYPE = 72
    DEFINITION = (
        ('side', UEnum8(SIDE, SIDE_LEFT)),
        ('move', UEnum8(MOVE, MOVE_CLOSE)),
    )




class Holder(BasePacket):

    TYPE = 73
    DEFINITION = (
        ('side', UEnum8(SIDE, SIDE_LEFT)),
        ('move', UEnum8(MOVE, MOVE_CLOSE)),
    )




class CandleKicker(BasePacket):

    TYPE = 74
    DEFINITION = (
        ('side'    , UEnum8(SIDE                  , SIDE_LEFT)),
        ('which'   , UEnum8(CANDLE_KICKER         , CANDLE_KICKER_LOWER)),
        ('position', UEnum8(CANDLE_KICKER_POSITION, CANDLE_KICKER_POSITION_IDLE)),
    )




class GiftOpener(BasePacket):

    TYPE = 75
    DEFINITION = (
        ('position', UEnum8(GIFT_OPENER_POSITION, GIFT_OPENER_POSITION_IDLE)),
    )




class Pump(BasePacket):

    TYPE = 76
    DEFINITION = (
        ('action', UEnum8(PUMP, PUMP_OFF)),
    )


# Simulator


class SimulatorData(BasePacket):

    TYPE = 150
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#4169e1"
    DEFINITION = (
        ('leds', UInt8(0, "Dockstar leds status")),
    )




class SimulatorResetRoutePath(BasePacket):

    TYPE = 151
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ff00ff"




class SimulatorRoutePath(BasePacket):

    TYPE = 152
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ('points', List(84, SimulatorPoint(), [], "Route path points")),
    )




class SimulatorSimplifiedRoutePath(BasePacket):

    TYPE = 153
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ('points', List(84, SimulatorPoint(), [], "Route path points")),
    )




class SimulatorRouteResetZones(BasePacket):

    TYPE = 154
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#40e0d0"




class SimulatorRouteRects(BasePacket):

    TYPE = 155
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#d8bfd8"
    DEFINITION = (
        ('is_forbidden_zone', Bool(True, "Represents forbidden zones or not")),
        ('shapes',            List(42, SimulatorRect(), [], "Route rects")),
    )




class SimulatorRouteCircles(BasePacket):

    TYPE = 156
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ff6347"
    DEFINITION = (
        ('is_forbidden_zone', Bool(True, "Represents forbidden zones or not")),
        ('shapes',            List(63, SimulatorCircle(), [], "Route circles")),
    )




class SimulatorOpponentsPositions(BasePacket):

    TYPE = 157
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('robot'   , UEnum8(OPPONENT_ROBOT, OPPONENT_ROBOT_MAIN)),
        ('present' , Bool  (True, "Opponent present")),
        ('x'       , Float (-1.0, "Opponent X coordinate")),
        ('y'       , Float (-1.0, "Opponent Y coordinate")),
        ('distance', Float (-1.0, "Estimated distannce")),
    )




class SimulatorClearGraphMapEdges(BasePacket):

    TYPE = 158
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"




class SimulatorGraphMapEdges(BasePacket):

    TYPE = 159
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('points', List(63, Float(0.0), [], "Route circles")),
    )




class SimulatorGraphMapRoute(BasePacket):

    TYPE = 160
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('points', List(63, Float(0.0), [], "Edges")),
    )




class SimulatorFetchColors(BasePacket):

    TYPE = 161
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('colors', List(20, Bool(False), [], "Detections")),
    )



################################################################################
# Packets lookup setup


PACKETS_BY_NAME = {}
PACKETS_BY_TYPE = {}
PACKETS_LIST = []


for (item_name, item_type) in inspect.getmembers(sys.modules[__name__]):
    if inspect.isclass(item_type) and issubclass(item_type, BasePacket) and item_type != BasePacket:
        # Create a packet instance a first time to finish the setup
        item_type()
        PACKETS_BY_NAME[item_name] = item_type
        PACKETS_BY_TYPE[item_type.TYPE] = item_type
    PACKETS_LIST = list(PACKETS_BY_TYPE.values())


def create_packet(buffer):
    (packet_type,) = struct.unpack("<B", buffer[:1])
    packet_class = PACKETS_BY_TYPE[packet_type]
    return packet_class()
