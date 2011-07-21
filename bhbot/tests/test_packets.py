#!/usr/bin/env python
# encoding: utf-8

import unittest

import packets
import trajectory

from definitions import *



########################################################################
# Packet test infrastructure



class PacketTestMixin(object):

    def create_packet(self):
        return None


    def initialize_packet(self, packet):
        pass


    def assert_packet_equal(self, packet1, packet2):
        self.assert_equal(packet1.__dict__, packet2.__dict__)


    def assert_equal(self, value1, value2):
        self.assertEqual(type(value1), type(value2))
        if isinstance(value1, dict):
            self.assert_dict_equal(value1, value2)
        elif isinstance(value1, list) or isinstance(value1, tuple):
            self.assert_sequence_equal(value1, value2)
        elif isinstance(value1, trajectory.Pose):
            self.assert_pose_equal(value1, value2)
        elif isinstance(value1, float):
            self.assert_float_equal(value1, value2)
        else:
            self.assertEqual(value1, value2)


    def assert_dict_equal(self, dict1, dict2):
        self.assertEqual(len(dict1), len(dict2))
        for key, value1 in dict1.iteritems():
            value2 = dict2[key]
            self.assert_equal(value1, value2)


    def assert_sequence_equal(self, seq1, seq2):
        self.assertEqual(len(seq1), len(seq2))
        for i in xrange(len(seq1)):
            self.assert_equal(seq1[i], seq2[i])


    def assert_pose_equal(self, pose1, pose2):
        self.assert_float_equal(pose1.x, pose2.x)
        self.assert_float_equal(pose1.y, pose2.y)
        self.assert_float_equal(pose1.angle, pose2.angle)


    def assert_float_equal(self, value1, value2):
        self.assertAlmostEqual(value1, value2, None, None, 0.01)


    def test_serialization_length(self):
        packet = self.create_packet()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)


    def test_buffer_serialization_deserialization(self):
        packet1 = self.create_packet()
        self.initialize_packet(packet1)
        buf = packet1.serialize()
        packet2 = self.create_packet()
        packet2.deserialize(buf)
        self.assert_packet_equal(packet1, packet2)


    def test_dict_serialization_deserialization(self):
        packet1 = self.create_packet()
        self.initialize_packet(packet1)
        d = packet1.to_dict()
        packet2 = self.create_packet()
        packet2.from_dict(d)
        self.assert_packet_equal(packet1, packet2)


    def test_pretty_dict_serialization_deserialization(self):
        packet1 = self.create_packet()
        self.initialize_packet(packet1)
        d = packet1.to_pretty_dict()
        packet2 = self.create_packet()
        packet2.from_pretty_dict(d)
        self.assert_packet_equal(packet1, packet2)


    def test_packet_id_uniqueness(self):
        count = 0
        packet_type = self.create_packet().TYPE
        for name, packet_class in packets.PACKETS_BY_NAME.iteritems():
            if packet_class.TYPE == packet_type:
                count += 1
        self.assertEqual(count, 1)



########################################################################
# Packet test cases



class ControllerReadyPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.ControllerReady()




class DeviceBusyPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.DeviceBusy()


    def initialize_packet(self, packet):
        packet.remote_device = REMOTE_DEVICE_SIMULATOR




class DeviceReadyPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.DeviceReady()


    def initialize_packet(self, packet):
        packet.team = TEAM_RED
        packet.remote_device = REMOTE_DEVICE_SIMULATOR




class StartPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Start()


    def initialize_packet(self, packet):
        packet.team = TEAM_RED




class GotoPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Goto()


    def initialize_packet(self, packet):
        packet.movement = MOVEMENT_ROTATE
        packet.direction = DIRECTION_BACKWARD
        packet.points.append(trajectory.Pose(10.5, 20.3, 56.4))
        packet.points.append(trajectory.Pose(84.6, 45.7, 85.0))
        packet.points.append(trajectory.Pose(75.4, 75.2, 12.8))
        packet.points.append(trajectory.Pose(85.4, 45.7, 89.4))




class GotoStartedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GotoStarted()




class GotoFinishedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GotoFinished()


    def initialize_packet(self, packet):
        packet.reason = REASON_PIECE_FOUND
        packet.current_pose = trajectory.Pose(20.0, 30.0, 5.5)
        packet.current_point_index = 12




class BlockedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Blocked()


    def initialize_packet(self, packet):
        packet.side = BLOCKED_BACK




class EnableAntiBlockingPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.EnableAntiBlocking()




class DisableAntiBlockingPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.DisableAntiBlocking()




class KeepAlivePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.KeepAlive()


    def initialize_packet(self, packet):
        packet.current_pose = trajectory.Pose(564.8, 452.3, 96.4)
        packet.match_started = True
        packet.match_time = 56




class PositionControlConfigPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.PositionControlConfig()


    def initialize_packet(self, packet):
        packet.t_acc = 453.45
        packet.f_va_max = 684.2
        packet.u_max = 4857.3
        packet.k1 = 85404.2
        packet.k2 = 64.5
        packet.k3 = 6874.5
        packet.g_re_1 = 0.2854
        packet.g_re_2 = 0.587
        packet.dist_min = 354.3
        packet.angle_min = 354.584
        packet.nbr_pas_evit_detect = 84
        packet.ecart_roue_libre = 84.354
        packet.ecart_roue_motrice = 684.3541
        packet.k_p_d = 6874.3554
        packet.k_i_d = 684.684
        packet.v_max_d = 687.123
        packet.d_roue_d = 9873.669
        packet.nb_pas_d = 6832
        packet.k_p_g = 912.415
        packet.k_i_g = 97.1
        packet.v_max_g = 0.684
        packet.d_roue_g = 1.6387
        packet.nb_pas_g = 3
        packet.app_net_ip = 3456874
        packet.app_net_mask = 1234698
        packet.app_net_gateway = 988769
        packet.app_net_port = 9867
        packet.test_mode = 890
        packet.coefficient_glissement_lateral = 2345.66




class StopPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Stop()




class ResettlePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Resettle()


    def initialize_packet(self, packet):
        packet.axis = AXIS_Y
        packet.position = 684.87
        packet.angle = 156.5




class DeploymentPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Deployment()




class PieceDetectedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.PieceDetected()


    def initialize_packet(self, packet):
        packet.start_pose = trajectory.Pose(12.3, 67.9, 56.4)
        packet.start_distance = 54.8
        packet.end_pose = trajectory.Pose(2.3, 32.9, 98.3)
        packet.end_distance = 34.0
        packet.sensor = SENSOR_RIGHT_BOTTOM
        packet.angle = 32.8




class StorePiece1PacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.StorePiece1()


    def initialize_packet(self, packet):
        packet.piece_count = 34




class StorePiece2PacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.StorePiece2()


    def initialize_packet(self, packet):
        packet.piece_count = 52




class StorePiece3PacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.StorePiece3()


    def initialize_packet(self, packet):
        packet.piece_count = 64




class ReleasePiecePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.ReleasePiece()




class OpenNippersPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.OpenNippers()




class CloseNippersPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.CloseNippers()




class EnableLateralSensorsPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.EnableLateralSensors()




class DisableLateralSensorsPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.DisableLateralSensors()




class CloseMandiblesPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.CloseMandibles()




class OpenMandiblesPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.OpenMandibles()




class ReinitializePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Reinitialize()




class SimulatorDataPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.SimulatorData()


    def initialize_packet(self, packet):
        packet.leds = 10




class TurretDetectPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TurretDetect()


    def initialize_packet(self, packet):
        packet.angle = 8.64
