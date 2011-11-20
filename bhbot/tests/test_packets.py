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
        d = packet1.to_dict(True)
        packet2 = self.create_packet()
        packet2.from_dict(d, True)
        self.assert_packet_equal(packet1, packet2)


    def test_packet_id_uniqueness(self):
        count = 0
        packet_type = self.create_packet().TYPE
        for name, packet_class in packets.PACKETS_BY_NAME.iteritems():
            if packet_class.TYPE == packet_type:
                count += 1
        self.assertEqual(count, 1)


    def inspect_logview_structure(self, s, parent):
        self.assertTrue(s.has_key("name"))
        if parent != None:
            self.assertTrue(s.has_key("parent"))
            self.assertEqual(s["parent"], parent)

        if s.has_key("children"):
            for c in s["children"]:
                self.inspect_logview_structure(c, s)
        elif parent != None:
            self.assertTrue(s.has_key("value"))



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
        packet.reason = REASON_DESTINATION_REACHED
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




class GripperControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GripperControl()


    def initialize_packet(self, packet):
        packet.move = GRIPPER_OPEN
        packet.which = GRIPPER_SIDE_BOTH




class SweeperControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.SweeperControl()


    def initialize_packet(self, packet):
        packet.move = SWEEPER_OPEN




class MapArmControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.MapArmControl()


    def initialize_packet(self, packet):
        packet.move = MAP_ARM_OPEN




class MapGripperControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.MapGripperControl()


    def initialize_packet(self, packet):
        packet.move = MAP_GRIPPER_OPEN




class EmptyTankControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.EmptyTankControl()


    def initialize_packet(self, packet):
        packet.move = TANK_DEPLOY




class GoldBullionDetectedTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GoldBullionDetected()




class TissueStoreControlTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TissueStoreControl()


    def initialize_packet(self, packet):
        packet.move = TISSUE_STORE_LOW




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
        packet.distance = 26
        packet.angle = 3
