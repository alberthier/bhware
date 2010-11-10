#!/usr/bin/env python
# encoding: utf-8

import unittest
import packets


class ControllerReadyPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.ControllerReady

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DeviceBusyPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.DeviceBusy

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DeviceReadyPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.DeviceReady

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class StartPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Start

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class GotoPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Goto

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class GotoStartedPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.GotoStarted

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class GotoFinishedPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.GotoFinished

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class BlockedPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Blocked

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class EnableAntiBlockingPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.EnableAntiBlocking

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DisableAntiBlockingPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.DisableAntiBlocking

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class KeepAlivePacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.KeepAlive

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class PositionControlConfigPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.PositionControlConfig

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class StopPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Stop

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class ResettlePacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Resettle

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class ReinitializePacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.Reinitialize

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class SimulatorDataPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.SimulatorData

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class TurretDetectPacketTestCase(unittest.TestCase):

    def setUp(self):
        self.packet_ctor = packets.TurretDetect

    def test_serialization_length(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        self.assertTrue(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = self.packet_ctor()
        buf = packet.serialize()
        packet2 = self.packet_ctor()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)
