#!/usr/bin/env python
# encoding: utf-8

import sys
import struct
import inspect

from definitions import *

import trajectory




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


    def to_pretty_value(self, value):
        return value


    def from_pretty_value(self, value):
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


    def to_pretty_value(self, value):
        return "{0:0.4f}".format(value)


    def from_pretty_value(self, value):
        return float(value)




class DoubleItem(PacketItem):

    C_TYPE = 'd'
    DESCRIPTION = "8 bytes real (double)"

    def __init__(self, name, default_value, description = None):
        PacketItem.__init__(self, name, default_value, description)


    def to_pretty_value(self, value):
        return "{0:0.4f}".format(value)


    def from_pretty_value(self, value):
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


    def to_pretty_value(self, value):
        return self.enum.lookup_by_value[value]


    def from_pretty_value(self, value):
        return self.enum.lookup_by_name[value]




class UEnum8Item(UInt8Item):

    DESCRIPTION = "8 bytes enum value (unsigned char)"

    def __init__(self, name, default_value, enum_name):
        self.enum = ENUMS[enum_name]
        UInt8Item.__init__(self, name, default_value, self.enum.description)


    def to_pretty_value(self, value):
        return self.enum.lookup_by_value[value]


    def from_pretty_value(self, value):
        return self.enum.lookup_by_name[value]



class PoseItem(PacketItem):

    C_TYPE = 'fff'
    DESCRIPTION = "Robot pose"

    def __init__(self, name, description = None):
        PacketItem.__init__(self, name, trajectory.Pose(0.0, 0.0, 0.0), description)


    def to_value_list(self, value, buf):
        buf.append(value.x)
        buf.append(value.y)
        buf.append(value.angle)


    def from_value_list(self, buf):
        value = trajectory.Pose(buf[0], buf[1], buf[2])
        del buf[0:3]
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




class PoseListItem(ListItem):

    DESCRIPTION = "Pose list"

    def __init__(self, name, default_value, max_elements, description = None):
        ListItem.__init__(self, name, default_value, PoseWithOptionalAngleItem(""), max_elements, description)




################################################################################
# Packet classes




class BasePacket(object):

    MAX_SIZE = 256
    DEFINITION = ()
    DESCRIPTION = ""
    LOGVIEW_COLOR = "#000000"
    LOGVIEW_DEFAULT_ENABLED = True
    STRUCT = None

    def __init__(self, initialize_members = True):
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

        if initialize_members:
            for elt in self.DEFINITION:
                setattr(self, elt.name, elt.default_value)


    def do_deserialize(self, buf):
        elements = {}
        unpacked = list(self.STRUCT.unpack(buf))
        # pop the type
        del unpacked[0]
        for elt in self.DEFINITION:
            elements[elt.name] = elt.from_value_list(unpacked)
        return elements


    def deserialize(self, buf):
        for k, v in self.do_deserialize(buf).iteritems():
            setattr(self, k, v)


    def do_serialize(self, elements):
        args = []
        for elt in self.DEFINITION:
            elt.to_value_list(elements[elt.name], args)
        return self.STRUCT.pack(self.TYPE, *args)


    def serialize(self):
        elements = {}
        for elt in self.DEFINITION:
            elements[elt.name] = getattr(self, elt.name)
        return self.do_serialize(elements)


    def to_dict(self):
        packet_dict = {}
        for elt in self.DEFINITION:
            packet_dict[elt.name] = getattr(self, elt.name)
        return packet_dict


    def to_pretty_dict(self):
        packet_dict = {}
        for elt in self.DEFINITION:
            packet_dict[elt.name] = elt.to_pretty_value(getattr(self, elt.name))
        return packet_dict


    def from_dict(self, packet_dict):
        for elt in self.DEFINITION:
            value = packet_dict[elt.name]
            setattr(self, elt.name, value)


    def from_pretty_dict(self, packet_dict):
        for elt in self.DEFINITION:
            value = elt.from_pretty_value(packet_dict[elt.name])
            setattr(self, elt.name, value)




class ControllerReady(BasePacket):

    TYPE = 0




class DeviceBusy(BasePacket):

    TYPE = 1
    DEFINITION = (
        UEnum8Item('remote_device', REMOTE_DEVICE_PIC, 'REMOTE_DEVICE'),
    )




class DeviceReady(BasePacket):

    TYPE = 2
    DEFINITION = (
        UEnum8Item('team',          TEAM_UNKNOWN,      'TEAM'),
        UEnum8Item('remote_device', REMOTE_DEVICE_PIC, 'REMOTE_DEVICE'),
    )




class Start(BasePacket):

    TYPE = 3
    DEFINITION = (
        UEnum8Item('team', TEAM_UNKNOWN, 'TEAM'),
    )



class Goto(BasePacket):

    TYPE = 4
    DEFINITION = (
        UEnum8Item  ('movement',   MOVEMENT_MOVE,     'MOVEMENT'),
        Enum8Item   ('direction',  DIRECTION_FORWARD, 'DIRECTION'),
        PoseListItem('points', [], 19),
    )




class GotoStarted(BasePacket):

    TYPE = 5




class GotoFinished(BasePacket):

    TYPE = 6
    DEFINITION = (
        UEnum8Item('reason',              REASON_DESTINATION_REACHED, 'REASON'),
        PoseItem  ('current_pose'),
        UInt8Item ('current_point_index', 0,                          "Last reached point index of the point list given in the Goto packet"),
    )




class Blocked(BasePacket):

    TYPE = 7
    DEFINITION = (
        Enum8Item('side', BLOCKED_FRONT, 'BLOCKING'),
    )





class EnableAntiBlocking(BasePacket):

    TYPE = 8




class DisableAntiBlocking(BasePacket):

    TYPE = 9




class KeepAlive(BasePacket):

    TYPE = 10
    DEFINITION = (
        PoseItem  ('current_pose',         "Current robot pose"),
        BoolItem  ('match_started', False, "Flag defining if the match has already started"),
        UInt32Item('match_time',    0,     "Time elapsed since the start of the match"),
    )




class PositionControlConfig(BasePacket):

    TYPE = 11
    DEFINITION = (
        FloatItem ('t_acc',                             0.0),
        FloatItem ('f_va_max',                          0.0),
        FloatItem ('u_max',                             0.0),
        FloatItem ('k1',                                0.0),
        FloatItem ('k2',                                0.0),
        FloatItem ('k3',                                0.0),
        FloatItem ('g_re_1',                            0.0),
        FloatItem ('g_re_2',                            0.0),
        FloatItem ('dist_min',                          0.0),
        FloatItem ('angle_min',                         0.0),
        UInt16Item('nbr_pas_evit_detect',               0),
        FloatItem ('ecart_roue_libre',                  0.0),
        FloatItem ('ecart_roue_motrice',                0.0),
        FloatItem ('k_p_d',                             0.0),
        FloatItem ('k_i_d',                             0.0),
        FloatItem ('v_max_d',                           0.0),
        FloatItem ('d_roue_d',                          0.0),
        UInt16Item('nb_pas_d',                          0),
        FloatItem ('k_p_g',                             0.0),
        FloatItem ('k_i_g',                             0.0),
        FloatItem ('v_max_g',                           0.0),
        FloatItem ('d_roue_g',                          0.0),
        UInt16Item('nb_pas_g',                          0),
        UInt32Item('app_net_ip',                        0),
        UInt32Item('app_net_mask',                      0),
        UInt32Item('app_net_gateway',                   0),
        UInt32Item('app_net_port',                      0),
        UInt32Item('test_mode',                         0),
        FloatItem ('coefficient_glissement_lateral',    0.0),
    )




class Stop(BasePacket):

    TYPE = 12




class Resettle(BasePacket):

    TYPE = 13
    DEFINITION = (
        UEnum8Item('axis',     AXIS_X, 'AXIS'),
        FloatItem ('position', 0.0,    "Robot position on the given axis"),
        FloatItem ('angle',    0.0,    "Robot angle"),
    )




class Deployment(BasePacket):

    TYPE = 14




class PieceDetected(BasePacket):

    TYPE = 15
    DEFINITION = (
        PoseItem  ('start_pose',                  "Detection start pose"),
        FloatItem ('start_distance', 0.0,         "Detection start distance"),
        PoseItem  ('end_pose',                    "Detection end pose"),
        FloatItem ('end_distance',   0.0,         "Detection end distance"),
        UEnum8Item('sensor',         SENSOR_NONE, 'SENSOR'),
        FloatItem ('angle',          0.0,         "SICK detection angle"),
    )




class StorePiece1(BasePacket):

    TYPE = 16
    DEFINITION = (
        UInt8Item('piece_count', 0, "Stored piece count"),
    )




class StorePiece2(BasePacket):

    TYPE = 17
    DEFINITION = (
        UInt8Item('piece_count', 0, "Stored piece count"),
    )




class StorePiece3(BasePacket):

    TYPE = 18
    DEFINITION = (
        UInt8Item('piece_count', 0, "Stored piece count"),
    )




class ReleasePiece(BasePacket):

    TYPE = 19




class OpenNippers(BasePacket):

    TYPE = 20




class CloseNippers(BasePacket):

    TYPE = 21




class EnableLateralSensors(BasePacket):

    TYPE = 22




class DisableLateralSensors(BasePacket):

    TYPE = 23




class CloseMandibles(BasePacket):

    TYPE = 24




class OpenMandibles(BasePacket):

    TYPE = 25




class Reinitialize(BasePacket):

    TYPE = 102




class SimulatorData(BasePacket):

    TYPE = 103
    DEFINITION = (
        UInt8Item('leds', 0, "Dockstar leds status"),
    )




class TurretDetect(BasePacket):

    MAX_SIZE = 5
    TYPE = 32
    DEFINITION = (
        FloatItem('angle', 0.0, "Opponent detection angle"),
    )




################################################################################
# Packets lookup setup


PACKETS_BY_NAME = {}
PACKETS_BY_TYPE = {}


for (item_name, item_type) in inspect.getmembers(sys.modules[__name__]):
    if inspect.isclass(item_type) and issubclass(item_type, BasePacket) and item_type != BasePacket:
        PACKETS_BY_NAME[item_name] = item_type
        PACKETS_BY_TYPE[item_type.TYPE] = item_type


def create_packet(buffer):
    (packet_type,) = struct.unpack("<B", buffer[0])
    packet_class = PACKETS_BY_TYPE[packet_type]
    return packet_class()
