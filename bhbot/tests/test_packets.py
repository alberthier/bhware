# encoding: utf-8

import unittest
import run_tests
run_tests.patch_pythonpath()


import packets
import position
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
        elif isinstance(value1, position.Pose):
            self.assert_pose_equal(value1, value2)
        elif isinstance(value1, float):
            self.assert_float_equal(value1, value2)
        else:
            self.assertEqual(value1, value2)


    def assert_dict_equal(self, dict1, dict2):
        self.assertEqual(len(dict1), len(dict2))
        for key, value1 in dict1.items():
            value2 = dict2[key]
            self.assert_equal(value1, value2)


    def assert_sequence_equal(self, seq1, seq2):
        self.assertEqual(len(seq1), len(seq2))
        for i in range(len(seq1)):
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


    def test_packet_id_uniqueness(self):
        count = 0
        packet_type = self.create_packet().TYPE
        for name, packet_class in packets.PACKETS_BY_NAME.items():
            if packet_class.TYPE == packet_type:
                count += 1
        self.assertEqual(count, 1)




########################################################################
# Packet test cases




class TurretDetectPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TurretDetect()


    def initialize_packet(self, packet):
        packet.distance = 26
        packet.angle = 3




class TurretInitPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TurretInit()


    def initialize_packet(self, packet):
        packet.mode = TURRET_INIT_MODE_WRITE
        packet.short_distance = 12
        packet.long_distance = 25




class TurretDistancesPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TurretDistances()


    def initialize_packet(self, packet):
        packet.short_distance = 12
        packet.long_distance = 25




class TurretBootPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.TurretBoot()




class ReinitializePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Reinitialize()




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




class RotatePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Rotate()


    def initialize_packet(self, packet):
        packet.direction = DIRECTION_BACKWARDS
        packet.angle = 2.56




class MoveCurvePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.MoveCurve()


    def initialize_packet(self, packet):
        packet.direction = DIRECTION_BACKWARDS
        packet.angle = 2.56
        packet.points.append(position.Pose(1.5, 2.3))
        packet.points.append(position.Pose(0.6, 1.7))
        packet.points.append(position.Pose(2.0, 0.2))
        packet.points.append(position.Pose(1.2, 2.7))




class MoveLinePacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.MoveLine()


    def initialize_packet(self, packet):
        packet.direction = DIRECTION_BACKWARDS
        packet.points.append(position.Pose(1.5, 2.3))
        packet.points.append(position.Pose(0.6, 1.7))
        packet.points.append(position.Pose(2.0, 0.2))
        packet.points.append(position.Pose(1.2, 2.7))




class MoveArcPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.MoveArc()


    def initialize_packet(self, packet):
        packet.direction = DIRECTION_BACKWARDS
        packet.center = position.Pose(2.34, 1.78)
        packet.radius = 2.56
        packet.points.append(1.5)
        packet.points.append(1.7)
        packet.points.append(2.0)
        packet.points.append(2.7)




class GotoStartedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GotoStarted()




class WaypointReachedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.WaypointReached()




class GotoFinishedPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GotoFinished()


    def initialize_packet(self, packet):
        packet.reason = REASON_STOP_REQUESTED
        packet.current_pose = position.Pose(20.0, 30.0, 5.5)
        packet.current_point_index = 12




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
        packet.current_pose = position.Pose(564.8, 452.3, 96.4)
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




class StopAllPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.StopAll()




class GlassPresentPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GlassPresent()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT




class NipperPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Nipper()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT
        packet.move = MOVE_OPEN




class LifterPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Lifter()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT
        packet.move = MOVE_OPEN




class GripperPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Gripper()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT
        packet.move = MOVE_OPEN




class HolderPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Holder()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT
        packet.move = MOVE_OPEN




class CandleKickerPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.CandleKicker()


    def initialize_packet(self, packet):
        packet.side = SIDE_RIGHT
        packet.which = CANDLE_KICKER_UPPER
        packet.position = CANDLE_KICKER_POSITION_UP




class GiftOpenerPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.GiftOpener()


    def initialize_packet(self, packet):
        packet.position = GIFT_OPENER_POSITION_LEFT




class PumpPacketTestCase(unittest.TestCase, PacketTestMixin):

    def create_packet(self):
        return packets.Pump()


    def initialize_packet(self, packet):
        packet.action = PUMP_ON

class StructTestCase(unittest.TestCase):

    def test_to_dump(self):
        from packets import Struct, ShortAsFloat

        class PointTest(Struct):

            DESCRIPTION = "Test"

            def __init__(self, description = None):
                Struct.__init__(self, position.Pose, description,
                    ('x', ShortAsFloat(0.0, "X coordinate")),
                    ('y', ShortAsFloat(0.0, "Y coordinate")),
                )

        p = PointTest()
        p.x = 1.0
        p.y = 2.0

        self.assertEquals("(('x', 1.0000), ('y', 2.0000))",p.to_dump(p))

        # should not crash
        p = PointTest()
        p.x = None
        p.y = None

        self.assertEquals("(('x', None), ('y', None))",p.to_dump(p))