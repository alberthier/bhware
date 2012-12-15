# encoding: utf-8

import sys
import struct
import inspect
import copy
import math
from collections import OrderedDict

from definitions import *

import trajectory




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
# Packet item types







class PointItem(Item):

    C_TYPE = 'hh'
    DESCRIPTION = "Field coordinates"

    def __init__(self, description = None):
        PacketItem.__init__(self, trajectory.Pose(), description)


    def serialize(self, value, buf):
        buf.append(int(value.x * 10000.0))
        buf.append(int(value.y * 10000.0))


    def deserialize(self, iterator):
        x = next(iterator)
        y = next(iterator)
        return trajectory.Pose(float(x) / 10000.0, float(y) / 10000.0)




class PoseItem(Struct):

    DESCRIPTION = "Robot pose"

    def __init__(self, name, description = None):
        Struct.__init__(self, trajectory.Pose, description,
            ('x'    , Float              (0.0, "X coordinate")),
            ('y'    , Float              (0.0, "Y coordinate")),
            ('angle', OptionalFloatRadian(0.0, "angle")),
        )



class OptionalAngle(FloatRadianItem):

    DESCRIPTION = "Optional angle"

    def __init__(self, name, default_value, description = None):
        FloatRadianItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        if value is not None:
            buf.append(value)
        else:
            buf.append(-1100000.0)


    def from_value_list(self, buf):
        angle = buf[0]
        del buf[0]
        if angle < -1000000.0:
            angle = None
        return angle


    def to_dict_value(self, value, pretty = False):
        if value != None:
            return super(OptionalAngle, self).to_dict_value(value, pretty)
        return "None"


    def from_dict_value(self, value, pretty = False):
        if value == "None":
            return None
        return super(OptionalAngle, self).from_dict_value(value, pretty)



class SimulatorPointItem(PacketItem):

    C_TYPE = 'BH'
    DESCRIPTION = "Route point"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        buf.append(value[0])
        buf.append(value[1])


    def from_value_list(self, buf):
        (x, y) = buf[:2]
        del buf[:2]
        return x, y




class SimulatorRectItem(PacketItem):

    C_TYPE = 'BHBH'
    DESCRIPTION = "Route rectangle"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        buf.append(value[0])
        buf.append(value[1])
        buf.append(value[2])
        buf.append(value[3])


    def from_value_list(self, buf):
        (x1, y1, x2, y2) = buf[:4]
        del buf[:4]
        return x1, y1, x2, y2




class SimulatorCircleItem(PacketItem):

    C_TYPE = 'BHB'
    DESCRIPTION = "Route circle"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        buf.append(value[0])
        buf.append(value[1])
        buf.append(value[2])


    def from_value_list(self, buf):
        (x, y, radius) = buf[:3]
        del buf[:3]
        return x, y, radius




class ListItem(PacketItem):

    C_TYPE = None
    DESCRIPTION = "List"

    def __init__(self, name, default_value, element_type, max_elements, description = None):
        self.element_type = element_type
        self.max_elements = max_elements
        if self.C_TYPE is None:
            self.C_TYPE = "B"
            for i in range(self.max_elements):
                self.C_TYPE += self.element_type.C_TYPE
        PacketItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        buf.append(len(value))
        for elt in value:
            self.element_type.to_value_list(elt, buf)
        for i in range(self.max_elements - len(value)):
            self.element_type.to_value_list(self.element_type.default_value, buf)


    def from_value_list(self, buf):
        value = []
        count = buf[0]
        del buf[0]
        for i in range(count):
            ival = self.element_type.from_value_list(buf)
            value.append(ival)
        # Pop remaining elements
        for i in range(self.max_elements - count):
            self.element_type.from_value_list(buf)
        return value


    def to_dict_value(self, value, pretty = False):
        ret = []
        for elt in value:
            ret.append(self.element_type.to_dict_value(elt, pretty))
        return ret


    def from_dict_value(self, value, pretty = False):
        ret = []
        for elt in value:
            ret.append(self.element_type.from_dict_value(elt, pretty))
        return ret




class PointListItem(ListItem):

    DESCRIPTION = "Pose list"

    def __init__(self, name, default_value, max_elements, description = None):
        ListItem.__init__(self, name, default_value, PointItem(""), max_elements, description)




################################################################################
# Base packet class




class BasePacket(Struct):

    MAX_SIZE = 256
    DEFINITION = ()
    DESCRIPTION = ""
    LOGVIEW_COLOR = "#000000"
    LOGVIEW_DEFAULT_ENABLED = True
    STRUCT = None
    HANDLER_METHOD = None

    def __init__(self, *args, **kwargs):
        cls = type(self)
        Struct.__init__(self
        if cls.STRUCT is None:
            fmt = "<B"
            for elt in self.DEFINITION:
                fmt += elt.C_TYPE
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
            for n, v in args:
                if n == name:
                    value = item.from_dump(v)
                    break
            if name in kwargs:
                value = kwargs[name]

            if value is None:
                # Call the constructor of the value to duplicate it. This is necessary for lists
                value = copy.deepcopy(item.default_value)
            setattr(self, name, value)


    def deserialize(self, buf):
        elements = {}
        iterator = iter(self.STRUCT.unpack(buf))
        # pop the type
        next(iterator)
        for name, item in self.DEFINITION:
            setattr(self, name, item.from_value_list(iterator))


    def serialize(self):
        args = []
        for name, item in self.DEFINITION:
            value = getattr(self, name)
            item.to_value_list(value, args)
        return self.STRUCT.pack(self.TYPE, *args)


    def to_code(self):
        attrs = []
        for name, item in self.DEFINITION:
            value = getattr(self, name)
            attrs.append((name, item.to_dump(value)))
        return type(self).__name__ + str(tuple(attrs))


    def dispatch(self, obj):
        if hasattr(obj, self.HANDLER_METHOD):
            getattr(obj, self.HANDLER_METHOD)(self)
        if hasattr(obj, 'on_packet'):
            getattr(obj, 'on_packet')(self)




################################################################################
# Packet classes




class ControllerReady(BasePacket):

    TYPE = 0
    LOGVIEW_COLOR = "#a52a2a"




class DeviceBusy(BasePacket):

    TYPE = 1
    LOGVIEW_COLOR = "#5f9ea0"
    DEFINITION = (
        ('remote_device' = UEnum8(REMOTE_DEVICE_PIC, REMOTE_DEVICE)),
    )




class DeviceReady(BasePacket):

    TYPE = 2
    LOGVIEW_COLOR = "#7fff00"
    DEFINITION = (
        ('team'         , UEnum8(TEAM_UNKNOWN     , TEAM)),
        ('remote_device', UEnum8(REMOTE_DEVICE_PIC, REMOTE_DEVICE)),
    )




class Start(BasePacket):

    TYPE = 3
    LOGVIEW_COLOR = "#d2691e"
    DEFINITION = (
        ('team', UEnum8(TEAM_UNKNOWN, TEAM)),
    )



class Goto(BasePacket):

    TYPE = 4
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        ('movement' , Enum8Item    (MOVEMENT_MOVE    , MOVEMENT)),
        ('direction', Enum8Item    (DIRECTION_FORWARD, DIRECTION)),
        ('angle'    , OptionalAngle(None, "Destination angle")),
        ('points'   , PointListItem([], 62)),
    )




class GotoStarted(BasePacket):

    TYPE = 5
    LOGVIEW_COLOR = "#6495ed"




class GotoFinished(BasePacket):

    TYPE = 6
    LOGVIEW_COLOR = "#daa520"
    DEFINITION = (
        ('reason'             , Enum8Item(REASON_DESTINATION_REACHED, REASON)),
        ('current_pose'       , PoseItem ()),
        ('current_point_index', UInt8Item(0, "Last reached point index of the point list given in the Goto packet")),
    )




class EnableAntiBlocking(BasePacket):

    TYPE = 8
    LOGVIEW_COLOR = "#00ffff"




class DisableAntiBlocking(BasePacket):

    TYPE = 9
    LOGVIEW_COLOR = "#00008b"




class KeepAlive(BasePacket):

    TYPE = 10
    LOGVIEW_COLOR = "#8b008b"
    LOGVIEW_DEFAULT_ENABLED = False
    DEFINITION = (
        ('current_pose' , PoseItem  ("Current robot pose")),
        ('match_started', BoolItem  (False, "Flag defining if the match has already started")),
        ('match_time'   , UInt32Item(0, "Time elapsed since the start of the match")),
    )




class PositionControlConfig(BasePacket):

    TYPE = 11
    LOGVIEW_COLOR = "#556b2f"
    DEFINITION = (
        ('t_acc'   , FloatItem (0.0)),
        ('f_va_max', FloatItem (0.0)),
    )




class Stop(BasePacket):

    TYPE = 12
    LOGVIEW_COLOR = "#b22222"




class Resettle(BasePacket):

    TYPE = 13
    LOGVIEW_COLOR = "#ff1493"
    DEFINITION = (
       ('axis'    , UEnum8Item     (AXIS_X, AXIS)),
       ('position', FloatItem      (0.0, "Robot position on the given axis")),
       ('angle'   , FloatRadianItem(0.0, "Robot angle")),
    )




class GripperControl(BasePacket):

    TYPE = 14
    LOGVIEW_COLOR = "#ffa500"
    DEFINITION = (
        ('move' , UEnum8Item(GRIPPER_CLOSE,     GRIPPER)),
        ('which', UEnum8Item(GRIPPER_SIDE_BOTH, GRIPPER_SIDE)),
    )




class SweeperControl(BasePacket):

    TYPE = 15
    LOGVIEW_COLOR = "#ff4500"
    DEFINITION = (
        ('move', UEnum8Item(SWEEPER_CLOSE, SWEEPER)),
    )




class MapArmControl(BasePacket):

    TYPE = 16
    LOGVIEW_COLOR = "#98fb98"
    DEFINITION = (
        ('move', UEnum8Item(MAP_ARM_CLOSE, MAP_ARM)),
    )




class MapGripperControl(BasePacket):

    TYPE = 17
    LOGVIEW_COLOR = "#6a5acd"
    DEFINITION = (
        ('move', UEnum8Item(MAP_GRIPPER_CLOSE, MAP_GRIPPER)),
    )




class EmptyTankControl(BasePacket):

    TYPE = 18
    LOGVIEW_COLOR = "#4682b4"
    DEFINITION = (
        ('move', UEnum8Item(TANK_RETRACT, TANK)),
    )




class GoldBarDetection(BasePacket):

    TYPE = 19
    LOGVIEW_COLOR = "#ffd700"
    DEFINITION = (
        ('status', UEnum8Item(GOLD_BAR_MISSING, GOLD_BAR)),
    )




class FabricStoreControl(BasePacket):

    TYPE = 20
    LOGVIEW_COLOR = "#32cd32"
    DEFINITION = (
        ('move', UEnum8Item(FABRIC_STORE_HIGH, FABRIC_STORE)),
    )




class Reinitialize(BasePacket):

    TYPE = 100
    LOGVIEW_COLOR = "#db7093"




class SimulatorData(BasePacket):

    TYPE = 101
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#4169e1"
    DEFINITION = (
        ('leds', UInt8Item(0, "Dockstar leds status")),
    )




class SimulatorResetRoutePath(BasePacket):

    TYPE = 102
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ff00ff"




class SimulatorRoutePath(BasePacket):

    TYPE = 103
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ('points', ListItem([], SimulatorPointItem("", (0, 0)), 84, "Route path points")),
    )




class SimulatorSimplifiedRoutePath(BasePacket):

    TYPE = 104
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ('points', ListItem([], SimulatorPointItem("", (0, 0)), 84, "Route path points")),
    )




class SimulatorRouteResetZones(BasePacket):

    TYPE = 105
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#40e0d0"




class SimulatorRouteRects(BasePacket):

    TYPE = 106
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#d8bfd8"
    DEFINITION = (
        ('is_forbidden_zone', BoolItem(True, "Represents forbidden zones or not")),
        ('shapes'           , ListItem([], SimulatorRectItem("", (0, 0, 0, 0)), 42, "Route rects")),
    )




class SimulatorRouteCircles(BasePacket):

    TYPE = 107
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ff6347"
    DEFINITION = (
        ('is_forbidden_zone', BoolItem(True, "Represents forbidden zones or not")),
        ('shapes'           , ListItem([], SimulatorCircleItem("", (0, 0, 0)), 63, "Route circles")),
    )




class SimulatorOpponentsPositions(BasePacket):

    TYPE = 108
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('robot'   , UEnum8Item(OPPONENT_ROBOT_MAIN, OPPONENT_ROBOT),
        ('present' , BoolItem  (True, "Opponent present"),
        ('x'       , FloatItem (-1.0, "Opponent X coordinate"),
        ('y'       , FloatItem (-1.0, "Opponent Y coordinate"),
        ('distance', FloatItem (-1.0, "Estimated distannce"),
    )




class SimulatorClearGraphMapEdges(BasePacket):

    TYPE = 109
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"




class SimulatorGraphMapEdges(BasePacket):

    TYPE = 110
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('points', ListItem([], FloatItem("", 0.0), 63, "Edges")),
    )




class SimulatorGraphMapRoute(BasePacket):

    TYPE = 111
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        ('points', ListItem([], FloatItem("", 0.0), 63, "Edges")),
    )




class TurretDetect(BasePacket):

    MAX_SIZE = 4
    TYPE = 32
    LOGVIEW_COLOR = "#9acd32"
    DEFINITION = (
        ('distance', UInt8Item (0, "Detection distance")),
        ('angle'   , UInt8Item (0, "Detection angle index (0 <= angle <= 17; 20 deg resolution)")),
        ('robot'   , UEnum8Item(OPPONENT_ROBOT_MAIN, 'OPPONENT_ROBOT')),
    )




class TurretInit(BasePacket):

    MAX_SIZE = 4
    TYPE = 33
    LOGVIEW_COLOR = "#7fffd4"
    DEFINITION = (
        ('mode'          , UEnum8Item(TURRET_INIT_MODE_READ, TURRET_INIT_MODE)),
        ('short_distance', UInt8Item (0, "Short distance detection range")),
        ('long_distance' , UInt8Item (0, "Long distance detection range")),
    )




class TurretDistances(BasePacket):

    MAX_SIZE = 3
    TYPE = 34
    LOGVIEW_COLOR = "#191970"
    DEFINITION = (
        ('short_distance', UInt8Item (0, "Short distance detection range")),
        ('long_distance' , UInt8Item (0, "Long distance detection range")),
    )




class TurretBoot(BasePacket):

    MAX_SIZE = 1
    TYPE = 35
    LOGVIEW_COLOR = "#20b2aa"




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

