#!/usr/bin/env python
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
#   X  #dc143c crimson                           #48d1cc mediumturquoise
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
#      #7cfc00 lawngreen                         #ff6347 tomato
#      #fffacd lemonchiffon                      #40e0d0 turquoise
#      #add8e6 lightblue                         #ee82ee violet
#      #f08080 lightcoral                        #f5deb3 wheat
#      #e0ffff lightcyan                         #ffffff white
#      #fafad2 lightgoldenrodyellow              #f5f5f5 whitesmoke
#      #90ee90 lightgreen                        #ffff00 yellow
#      #d3d3d3 lightgrey                      X  #9acd32 yellowgreen
#      #ffb6c1 lightpink




################################################################################
# Packet item types




class PacketItem(object):

    C_TYPE = None
    DESCRIPTION = None


    def __init__(self, name, default_value, description = None):
        self.name = name
        self.default_value = default_value
        if description == None:
            description = self.DESCRIPTION
        else:
            self.description = description


    def to_value_list(self, value, buf):
        buf.append(value)


    def from_value_list(self, buf):
        value = buf[0]
        del buf[0]
        return value


    def to_dict_value(self, value, pretty = False):
        return value


    def from_dict_value(self, value, pretty = False):
        return value




class Int8Item(PacketItem):

    C_TYPE = 'b'
    DESCRIPTION = "1 byte signed integer (char)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class UInt8Item(PacketItem):

    C_TYPE = 'B'
    DESCRIPTION = "1 byte unsigned integer (unsigned char)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class Int16Item(PacketItem):

    C_TYPE = 'h'
    DESCRIPTION = "2 bytes signed integer (short)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class UInt16Item(PacketItem):

    C_TYPE = 'H'
    DESCRIPTION = "2 bytes unsigned integer (unsigned short)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class Int32Item(PacketItem):

    C_TYPE = 'i'
    DESCRIPTION = "4 bytes signed integer (int)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class UInt32Item(PacketItem):

    C_TYPE = 'I'
    DESCRIPTION = "4 bytes unsigned integer (unsigned int)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class Int64Item(PacketItem):

    C_TYPE = 'q'
    DESCRIPTION = "8 bytes signed integer (long long)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class UInt64Item(PacketItem):

    C_TYPE = 'Q'
    DESCRIPTION = "8 bytes unsigned integer (unsigned long long)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)




class FloatItem(PacketItem):

    C_TYPE = 'f'
    DESCRIPTION = "4 bytes real (float)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)


    def to_dict_value(self, value, pretty = False):
        if pretty:
            return "{:0.4f}".format(value)
        return value


    def from_dict_value(self, value, pretty = False):
        return float(value)




class DoubleItem(FloatItem):

    C_TYPE = 'd'
    DESCRIPTION = "8 bytes real (double)"

    def __init__(self, name, default_value, description = None):
        FloatItem.__init__(self, name, default_value, description)




class FloatRadianItem(FloatItem):

    def __init__(self, name, default_value, description = None):
        FloatItem.__init__(self, name, default_value, description)


    def to_dict_value(self, value, pretty = False):
        if pretty:
            return "{:0.4f} ({:0.4f} deg)".format(value, value / math.pi * 180.0)
        return value


    def from_dict_value(self, value, pretty = False):
        if pretty:
            return float(value[:value.find(" (")])
        else:
            return float(value)




class DoubleRadianItem(DoubleItem):

    def __init__(self, name, default_value, description = None):
        DoubleItem.__init__(self, name, default_value, description)


    def to_dict_value(self, value, pretty = False):
        if pretty:
            return "{:0.4f} ({:0.4f} deg)".format(value, value / math.pi * 180.0)
        return value


    def from_dict_value(self, value, pretty = False):
        if pretty:
            return float(value[:value.find(" (")])
        else:
            return float(value)




class BoolItem(UInt8Item):

    DESCRIPTION = "8 bytes boolean value (unsigned char)"

    def __init__(self, name, default_value, description = None):
        UInt8Item.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        if value:
            buf.append(1)
        else:
            buf.append(0)


    def from_value_list(self, buf):
        value = buf[0]
        del buf[0]
        return value != 0




class Enum8Item(Int8Item):

    DESCRIPTION = "8 bytes enum value (char)"

    def __init__(self, name, default_value, enum_name):
        self.enum = ENUMS[enum_name]
        Int8Item.__init__(self, name, default_value, self.enum.description)


    def to_dict_value(self, value, pretty = False):
        if pretty:
            return self.enum.lookup_by_value[value]
        return value


    def from_dict_value(self, value, pretty = False):
        if pretty:
            return self.enum.lookup_by_name[value]
        return value




class UEnum8Item(UInt8Item):

    DESCRIPTION = "8 bytes enum value (unsigned char)"

    def __init__(self, name, default_value, enum_name):
        self.enum = ENUMS[enum_name]
        UInt8Item.__init__(self, name, default_value, self.enum.description)


    def to_dict_value(self, value, pretty = False):
        if pretty:
            return self.enum.lookup_by_value[value]
        return value


    def from_dict_value(self, value, pretty = False):
        if pretty:
            return self.enum.lookup_by_name[value]
        return value




class PoseWithOptionalAngleItem(PacketItem):

    C_TYPE = 'fffB'
    DESCRIPTION = "Robot pose with optional angle"

    def __init__(self, name, description = None):
        PacketItem.__init__(self, name, trajectory.Pose(), description)


    def to_value_list(self, value, buf):
        buf.append(value.x)
        buf.append(value.y)
        if value.angle == None:
            buf.append(0.0)
            buf.append(0)
        else:
            buf.append(value.angle)
            buf.append(1)


    def from_value_list(self, buf):
        value = trajectory.Pose(buf[0], buf[1], buf[2])
        if buf[3] == 0:
            value.angle = None
        del buf[0:4]
        return value


    def to_dict_value(self, value, pretty = False):
        ret = OrderedDict()
        if pretty:
            ret["x"]     = "{:0.4f}".format(value.x)
            ret["y"]     = "{:0.4f}".format(value.y)
            if value.angle != None:
                ret["angle"] = "{:0.4f} ({:0.4f} deg)".format(value.angle, value.angle / math.pi * 180.0)
            else:
                ret["angle"] = "None"
        else:
            ret["x"]     = value.x
            ret["y"]     = value.y
            ret["angle"] = value.angle
        return ret


    def from_dict_value(self, value, pretty = False):
        pose = trajectory.Pose()
        pose.x = float(value["x"])
        pose.y = float(value["y"])
        angle  = value["angle"]
        if angle != None:
            if pretty:
                pose.angle = float(angle[:angle.find(" (")])
            else:
                pose.angle = float(angle)
        else:
            pose.angle = None
        return pose




class PoseItem(PoseWithOptionalAngleItem):

    C_TYPE = 'fff'
    DESCRIPTION = "Robot pose"

    def __init__(self, name, description = None):
        PoseWithOptionalAngleItem.__init__(self, name, description)
        self.default_value.angle = 0.0


    def to_value_list(self, value, buf):
        buf.append(value.x)
        buf.append(value.y)
        buf.append(value.angle)


    def from_value_list(self, buf):
        value = trajectory.Pose(buf[0], buf[1], buf[2])
        del buf[0:3]
        return value




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
        return (x, y)




class SimulatorRectItem(PacketItem):

    C_TYPE = 'BHBH'
    DESCRIPTION = "Route point"

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
        return (x1, y1, x2, y2)




class ListItem(PacketItem):

    C_TYPE = None
    DESCRIPTION = "List"

    def __init__(self, name, default_value, element_type, max_elements, description = None):
        self.element_type = element_type
        self.max_elements = max_elements
        if self.C_TYPE == None:
            self.C_TYPE = "B"
            for i in xrange(self.max_elements):
                self.C_TYPE += self.element_type.C_TYPE
        PacketItem.__init__(self, name, default_value, description)


    def to_value_list(self, value, buf):
        buf.append(len(value))
        for elt in value:
            self.element_type.to_value_list(elt, buf)
        for i in xrange(self.max_elements - len(value)):
            self.element_type.to_value_list(self.element_type.default_value, buf)


    def from_value_list(self, buf):
        value = []
        count = buf[0]
        del buf[0]
        for i in xrange(count):
            ival = self.element_type.from_value_list(buf)
            value.append(ival)
        # Pop remaining elements
        for i in xrange(self.max_elements - count):
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




class PoseListItem(ListItem):

    DESCRIPTION = "Pose list"

    def __init__(self, name, default_value, max_elements, description = None):
        ListItem.__init__(self, name, default_value, PoseWithOptionalAngleItem(""), max_elements, description)




################################################################################
# Base packet class




class BasePacket(object):

    MAX_SIZE = 256
    DEFINITION = ()
    DESCRIPTION = ""
    LOGVIEW_COLOR = "#000000"
    LOGVIEW_DEFAULT_ENABLED = True
    STRUCT = None
    HANDLER_METHOD = None

    def __init__(self, **kwargs):
        cls = type(self)
        if cls.STRUCT == None:
            fmt = "<B"
            for elt in self.DEFINITION:
                fmt += elt.C_TYPE
            size = struct.calcsize(fmt)
            pad_size = cls.MAX_SIZE - size
            if pad_size != 0:
                fmt += str(pad_size) + "x"
            cls.STRUCT = struct.Struct(fmt)

        if cls.HANDLER_METHOD == None:
            cls.HANDLER_METHOD = "on"
            for c in cls.__name__:
                if c.isupper():
                    cls.HANDLER_METHOD += "_" + c.lower()
                else:
                    cls.HANDLER_METHOD += c

        for elt in self.DEFINITION:
            if kwargs.has_key(elt.name):
                value = kwargs[elt.name]
            else:
                # Call the constructor of the value to duplicate it. This is necessary for lists
                value = copy.deepcopy(elt.default_value)
            setattr(self, elt.name, value)


    def deserialize(self, buf):
        elements = {}
        unpacked = list(self.STRUCT.unpack(buf))
        # pop the type
        del unpacked[0]
        for elt in self.DEFINITION:
            setattr(self, elt.name, elt.from_value_list(unpacked))


    def serialize(self):
        args = []
        for elt in self.DEFINITION:
            value = getattr(self, elt.name)
            elt.to_value_list(value, args)
        return self.STRUCT.pack(self.TYPE, *args)


    def to_dict(self, pretty = False):
        packet_dict = OrderedDict()
        for elt in self.DEFINITION:
            packet_dict[elt.name] = elt.to_dict_value(getattr(self, elt.name), pretty)
        return packet_dict


    def from_dict(self, packet_dict, pretty = False):
        for elt in self.DEFINITION:
            value = elt.from_dict_value(packet_dict[elt.name], pretty)
            setattr(self, elt.name, value)


    def to_code(self):
        code = type(self).__name__ + "("
        first = True
        for elt in self.DEFINITION:
            if not first:
                code += ", "
            else:
                first = False
            code += elt.name + " = " + str(getattr(self, elt.name))
        code += ")"
        return code


    def dispatch(self, obj):
        if hasattr(obj, self.HANDLER_METHOD):
            getattr(obj, self.HANDLER_METHOD)(self)




################################################################################
# Packet classes




class ControllerReady(BasePacket):

    TYPE = 0
    LOGVIEW_DEFAULT_ENABLED = True
    LOGVIEW_COLOR = "#a52a2a"




class DeviceBusy(BasePacket):

    TYPE = 1
    LOGVIEW_COLOR = "#5f9ea0"
    DEFINITION = (
        UEnum8Item('remote_device', REMOTE_DEVICE_PIC, 'REMOTE_DEVICE'),
    )




class DeviceReady(BasePacket):

    TYPE = 2
    LOGVIEW_COLOR = "#7fff00"
    DEFINITION = (
        UEnum8Item('team',          TEAM_UNKNOWN,      'TEAM'),
        UEnum8Item('remote_device', REMOTE_DEVICE_PIC, 'REMOTE_DEVICE'),
    )




class Start(BasePacket):

    TYPE = 3
    LOGVIEW_COLOR = "#d2691e"
    DEFINITION = (
        UEnum8Item('team', TEAM_UNKNOWN, 'TEAM'),
    )



class Goto(BasePacket):

    TYPE = 4
    LOGVIEW_COLOR = "#ff7f50"
    DEFINITION = (
        UEnum8Item  ('movement',   MOVEMENT_MOVE,     'MOVEMENT'),
        Enum8Item   ('direction',  DIRECTION_FORWARD, 'DIRECTION'),
        PoseListItem('points', [], 19),
    )




class GotoStarted(BasePacket):

    TYPE = 5
    LOGVIEW_COLOR = "#6495ed"




class GotoFinished(BasePacket):

    TYPE = 6
    LOGVIEW_COLOR = "#daa520"
    DEFINITION = (
        UEnum8Item('reason',              REASON_DESTINATION_REACHED, 'REASON'),
        PoseItem  ('current_pose'),
        UInt8Item ('current_point_index', 0,                          "Last reached point index of the point list given in the Goto packet"),
    )




class Blocked(BasePacket):

    TYPE = 7
    LOGVIEW_COLOR = "#dc143c"
    DEFINITION = (
        Enum8Item('side', BLOCKED_FRONT, 'BLOCKING'),
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
        PoseItem  ('current_pose',         "Current robot pose"),
        BoolItem  ('match_started', False, "Flag defining if the match has already started"),
        UInt32Item('match_time',    0,     "Time elapsed since the start of the match"),
    )




class PositionControlConfig(BasePacket):

    TYPE = 11
    LOGVIEW_COLOR = "#556b2f"
    DEFINITION = (
        FloatItem ('t_acc',                             0.0),
        FloatItem ('f_va_max',                          0.0),
    )




class Stop(BasePacket):

    TYPE = 12
    LOGVIEW_COLOR = "#b22222"




class Resettle(BasePacket):

    TYPE = 13
    LOGVIEW_COLOR = "#ff1493"
    DEFINITION = (
        UEnum8Item     ('axis',     AXIS_X, 'AXIS'),
        FloatItem      ('position', 0.0,    "Robot position on the given axis"),
        FloatRadianItem('angle',    0.0,    "Robot angle"),
    )




class GripperControl(BasePacket):

    TYPE = 14
    LOGVIEW_COLOR = "#ffa500"
    DEFINITION = (
        UEnum8Item('move',  GRIPPER_CLOSE,     'GRIPPER'),
        UEnum8Item('which', GRIPPER_SIDE_BOTH, 'GRIPPER_SIDE'),
    )




class SweeperControl(BasePacket):

    TYPE = 15
    LOGVIEW_COLOR = "#ff4500"
    DEFINITION = (
        UEnum8Item('move',  SWEEPER_CLOSE,     'SWEEPER'),
    )




class MapArmControl(BasePacket):

    TYPE = 16
    LOGVIEW_COLOR = "#98fb98"
    DEFINITION = (
        UEnum8Item('move',  MAP_ARM_CLOSE,     'MAP_ARM'),
    )




class MapGripperControl(BasePacket):

    TYPE = 17
    LOGVIEW_COLOR = "#6a5acd"
    DEFINITION = (
        UEnum8Item('move',  MAP_GRIPPER_CLOSE,     'MAP_GRIPPER'),
    )




class EmptyTankControl(BasePacket):

    TYPE = 18
    LOGVIEW_COLOR = "#4682b4"
    DEFINITION = (
        UEnum8Item('move',  TANK_RETRACT,     'TANK'),
    )




class GoldBarDetection(BasePacket):

    TYPE = 19
    LOGVIEW_COLOR = "#ffd700"
    DEFINITION = (
        UEnum8Item('status',  GOLD_BAR_MISSING,     'GOLD_BAR'),
    )




class FabricStoreControl(BasePacket):

    TYPE = 20
    LOGVIEW_COLOR = "#32cd32"
    DEFINITION = (
        UEnum8Item('move',  FABRIC_STORE_HIGH,     'FABRIC_STORE'),
    )




class Reinitialize(BasePacket):

    TYPE = 100
    LOGVIEW_COLOR = "#db7093"




class SimulatorData(BasePacket):

    TYPE = 103
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#4169e1"
    DEFINITION = (
        UInt8Item('leds', 0, "Dockstar leds status"),
    )




class SimulatorResetRoutePath(BasePacket):

    TYPE = 104
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ff00ff"




class SimulatorRoutePath(BasePacket):

    TYPE = 105
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ListItem('points', [], SimulatorPointItem("", (0, 0)), 84, "Route path points"),
    )




class SimulatorSimplifiedRoutePath(BasePacket):

    TYPE = 106
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#8b4513"
    DEFINITION = (
        ListItem('points', [], SimulatorPointItem("", (0, 0)), 84, "Route path points"),
    )




class SimulatorRouteWalls(BasePacket):

    TYPE = 107
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#d8bfd8"
    DEFINITION = (
        ListItem('walls', [], SimulatorRectItem("", (0, 0, 0, 0)), 42, "Route walls"),
    )




class SimulatorOpponentsPositions(BasePacket):

    TYPE = 108
    LOGVIEW_DEFAULT_ENABLED = False
    LOGVIEW_COLOR = "#ba55d3"
    DEFINITION = (
        UEnum8Item('robot',    OPPONENT_ROBOT_MAIN, 'OPPONENT_ROBOT'),
        BoolItem  ('present',  True,                "Opponent present"),
        FloatItem ('x',        -1.0,                "Opponent X coordinate"),
        FloatItem ('y',        -1.0,                "Opponent Y coordinate"),
        FloatItem ('distance', -1.0,                "Estimated distannce"),
    )




class TurretDetect(BasePacket):

    MAX_SIZE = 4
    TYPE = 32
    LOGVIEW_COLOR = "#9acd32"
    DEFINITION = (
        UInt8Item ('distance', 0, "Detection distance"),
        UInt8Item ('angle',    0, "Detection angle index (0 <= angle <= 17; 20 deg resolution)"),
        UEnum8Item('robot',    OPPONENT_ROBOT_MAIN, 'OPPONENT_ROBOT'),
    )




class TurretInit(BasePacket):

    MAX_SIZE = 4
    TYPE = 33
    LOGVIEW_COLOR = "#7fffd4"
    DEFINITION = (
        UEnum8Item('mode',           TURRET_INIT_MODE_READ, 'TURRET_INIT_MODE'),
        UInt8Item ('short_distance', 0, "Short distance detection range"),
        UInt8Item ('long_distance',  0, "Long distance detection range"),
    )




class TurretDistances(BasePacket):

    MAX_SIZE = 3
    TYPE = 34
    LOGVIEW_COLOR = "#191970"
    DEFINITION = (
        UInt8Item ('short_distance', 0, "Short distance detection range"),
        UInt8Item ('long_distance',  0, "Long distance detection range"),
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
    PACKETS_LIST = PACKETS_BY_TYPE.values()


def create_packet(buffer):
    (packet_type,) = struct.unpack("<B", buffer[0])
    packet_class = PACKETS_BY_TYPE[packet_type]
    return packet_class()
