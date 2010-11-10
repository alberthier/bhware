#!/usr/bin/env python
# encoding: utf-8

import struct

from definitions import *

import trajectory



class BasePacket(object):

    MAX_SIZE = 256
    FORMAT = None

    def __init__(self, format):
        cls = type(self)
        if cls.FORMAT == None:
            cls.FORMAT = "!B" + format
            size = struct.calcsize(cls.FORMAT)
            cls.FORMAT += str(BasePacket.MAX_SIZE - size) + "x"


    def do_deserialize(self, buf):
        unpacked = struct.unpack(self.FORMAT, buf)
        # pop the type
        return unpacked[1:]


    def do_serialize(self, *args):
        return struct.pack(self.FORMAT, self.TYPE, *args)


    def deserialize(self, buf):
        pass


    def serialize(self):
        return self.do_serialize()


    def to_dict(self):
        return self.__dict__.copy()


    def from_dict(self, d):
        self.__dict__ = d.copy()




class ControllerReady(BasePacket):

    TYPE = 0

    def __init__(self):
        BasePacket.__init__(self, "")




class DeviceBusy(BasePacket):

    TYPE = 1

    def __init__(self):
        BasePacket.__init__(self, "")



class DeviceReady(BasePacket):

    TYPE = 2

    def __init__(self):
        BasePacket.__init__(self, "B")
        self.team = TEAM_BLUE


    def deserialize(self, buf):
        (self.team,) = self.do_deserialize(buf)


    def serialize(self):
        return self.do_serialize(self.team)




class Start(BasePacket):

    TYPE = 3

    def __init__(self):
        BasePacket.__init__(self, "B")
        self.team = TEAM_UNKNOWN


    def deserialize(self, buf):
        (self.team,) = self.do_deserialize(buf)


    def serialize(self):
        return self.do_serialize(self.team)



class Goto(BasePacket):

    TYPE = 4
    MAX_POINTS = 19

    def __init__(self):
        BasePacket.__init__(self, "BbBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffBfffB")
        self.movement = MOVEMENT_MOVE
        self.direction = DIRECTION_FORWARD
        self.points = []


    def deserialize(self, buf):
        values = self.do_deserialize(buf)
        self.movement = values[0]
        self.direction = values[1]
        nb_points = values[2]
        self.points = []
        for i in xrange(nb_points):
            offset = 3 + 4 * i
            x = values[offset]
            y = values[offset + 1]
            use_angle = values[offset + 3]
            if use_angle != 0:
                angle = values[offset + 2]
            else:
                angle = None
            self.points.append(trajectory.Pose(x, y, angle))



    def serialize(self):
        values = [self.movement, self.direction, len(self.points)]
        for p in self.points:
            values.append(p.x)
            values.append(p.y)
            if p.angle != None:
                values.append(p.angle)
                values.append(1)
            else:
                values.append(0.0)
                values.append(0)

        for i in xrange(Goto.MAX_POINTS - len(self.points)):
            values.append(0.0)
            values.append(0.0)
            values.append(0.0)
            values.append(0)

        return self.do_serialize(*values)




class GotoStarted(BasePacket):

    TYPE = 5

    def __init__(self):
        BasePacket.__init__(self, "")





class GotoFinished(BasePacket):

    TYPE = 6

    def __init__(self):
        BasePacket.__init__(self, "B")
        self.reason = REASON_DESTINATION_REACHED


    def deserialize(self, buf):
        (self.reason,) = self.do_deserialize(buf)


    def serialize(self):
        return self.do_serialize(self.reason)




class Blocked(BasePacket):

    TYPE = 7

    def __init__(self):
        BasePacket.__init__(self, "B")
        self.side = BLOCKED_FRONT


    def deserialize(self, buf):
        (self.side,) = self.do_deserialize(buf)


    def serialize(self):
        return self.do_serialize(self.side)




class EnableAntiBlocking(BasePacket):

    TYPE = 8

    def __init__(self):
        BasePacket.__init__(self, "")




class DisableAntiBlocking(BasePacket):

    TYPE = 9

    def __init__(self):
        BasePacket.__init__(self, "")




class KeepAlive(BasePacket):

    TYPE = 10

    def __init__(self):
        BasePacket.__init__(self, "fffBI")
        self.current_pose = trajectory.Pose(0.0, 0.0, 0.0)
        self.match_started = False
        self.match_time = 0


    def deserialize(self, buf):
        values = self.do_deserialize(buf)
        self.current_pose.x = values[0]
        self.current_pose.y = values[1]
        self.current_pose.angle = values[2]
        self.match_started = values[3] != 0
        self.match_time = values[4]


    def serialize(self):
        values = []
        values.append(self.current_pose.x)
        values.append(self.current_pose.y)
        values.append(self.current_pose.angle)
        values.append(int(self.match_started))
        values.append(self.match_time)
        return self.do_serialize(*values)




class PositionControlConfig(BasePacket):

    TYPE = 11

    def __init__(self):
        BasePacket.__init__(self, "ffffffffffHffffffHffffH")
        self.t_acc = 0.0
        self.f_va_max = 0.0
        self.u_max = 0.0
        self.k1 = 0.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.g_re_1 = 0.0
        self.g_re_2 = 0.0
        self.dist_min = 0.0
        self.angle_min = 0.0
        self.nbr_pas_evit_detect = 0
        self.ecart_roue_libre = 0.0
        self.ecart_roue_motrice = 0.0
        self.k_p_d = 0.0
        self.k_i_d = 0.0
        self.v_max_d = 0.0
        self.d_roue_d = 0.0
        self.nb_pas_d = 0
        self.k_p_g = 0.0
        self.k_i_g = 0.0
        self.v_max_g = 0.0
        self.d_roue_g = 0.0
        self.nb_pas_g = 0


    def deserialize(self, buf):
        values = self.do_deserialize(buf)
        self.t_acc = values[0]
        self.f_va_max = values[1]
        self.u_max = values[2]
        self.k1 = values[3]
        self.k2 = values[4]
        self.k3 = values[5]
        self.g_re_1 = values[6]
        self.g_re_2 = values[7]
        self.dist_min = values[8]
        self.angle_min = values[9]
        self.nbr_pas_evit_detect = values[10]
        self.ecart_roue_libre = values[11]
        self.ecart_roue_motrice = values[12]
        self.k_p_d = values[13]
        self.k_i_d = values[14]
        self.v_max_d = values[15]
        self.d_roue_d = values[16]
        self.nb_pas_d = values[17]
        self.k_p_g = values[18]
        self.k_i_g = values[19]
        self.v_max_g = values[20]
        self.d_roue_g = values[21]
        self.nb_pas_g = values[22]

    def serialize(self):
        values = []
        values.append(self.t_acc)
        values.append(self.f_va_max)
        values.append(self.u_max)
        values.append(self.k1)
        values.append(self.k2)
        values.append(self.k3)
        values.append(self.g_re_1)
        values.append(self.g_re_2)
        values.append(self.dist_min)
        values.append(self.angle_min)
        values.append(self.nbr_pas_evit_detect)
        values.append(self.ecart_roue_libre)
        values.append(self.ecart_roue_motrice)
        values.append(self.k_p_d)
        values.append(self.k_i_d)
        values.append(self.v_max_d)
        values.append(self.d_roue_d)
        values.append(self.nb_pas_d)
        values.append(self.k_p_g)
        values.append(self.k_i_g)
        values.append(self.v_max_g)
        values.append(self.d_roue_g)
        values.append(self.nb_pas_g)
        return self.do_serialize(*values)




class Stop(BasePacket):

    TYPE = 12

    def __init__(self):
        BasePacket.__init__(self, "")




class Resettle(BasePacket):

    TYPE = 13

    def __init__(self):
        BasePacket.__init__(self, "Bff")
        self.axis = AXIS_ABSCISSA
        self.position = 0.0
        self.angle = 0.0


    def deserialize(self, buf):
        values = self.do_deserialize(buf)
        self.axis = values[0]
        self.position = values[1]
        self.angle = values[2]


    def serialize(self):
        return self.do_serialize(self.axis, self.position, self.angle)




class Reinitialize(BasePacket):

    TYPE = 102

    def __init__(self):
        BasePacket.__init__(self, "")




class SimulatorData(BasePacket):

    TYPE = 103

    def __init__(self):
        BasePacket.__init__(self, "B")
        self.whatever = 0


    def deserialize(self, buf):
        values = self.do_deserialize(buf)
        self.whatever = values[0]


    def serialize(self):
        values = []
        values.append(self.whatever)
        return self.do_serialize(*values)



class TurretDetect(BasePacket):

    TYPE = 32

    def __init__(self):
        BasePacket.__init__(self, "hH")
        self.mean_angle = 0
        self.angular_size = 0


    def deserialize(self, buf):
        (self.mean_angle, self.angular_size) = self.do_deserialize(buf)


    def serialize(self):
        return self.do_serialize(self.mean_angle, self.angular_size)
