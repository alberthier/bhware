#!/usr/bin/env python
# encoding: utf-8

import unittest

import packets
import trajectory

from definitions import *


def compare_floats(f1, f2):
    return abs(f1 - f2) < 0.01


class ControllerReadyPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.ControllerReady()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.ControllerReady()
        buf = packet.serialize()
        packet2 = packets.ControllerReady()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DeviceBusyPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.DeviceBusy()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.DeviceBusy()
        packet.remote_device = REMOTE_DEVICE_SIMULATOR
        buf = packet.serialize()
        packet2 = packets.DeviceBusy()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DeviceReadyPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.DeviceReady()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.DeviceReady()
        packet.team = TEAM_RED
        packet.remote_device = REMOTE_DEVICE_SIMULATOR
        buf = packet.serialize()
        packet2 = packets.DeviceReady()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class StartPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Start()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Start()
        packet.team = TEAM_RED
        buf = packet.serialize()
        packet2 = packets.Start()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class GotoPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Goto()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Goto()
        packet.movement = MOVEMENT_ROTATE
        packet.direction = DIRECTION_BACKWARD
        packet.points.append(trajectory.Pose(10.5, 20.3, 56.4))
        packet.points.append(trajectory.Pose(84.6, 45.7, 85.0))
        packet.points.append(trajectory.Pose(75.4, 75.2, 12.8))
        packet.points.append(trajectory.Pose(85.4, 45.7, 89.4))
        buf = packet.serialize()
        packet2 = packets.Goto()
        packet2.deserialize(buf)
        self.assertEqual(packet.movement, packet2.movement)
        self.assertEqual(packet.direction, packet2.direction)
        for i in xrange(len(packet.points)):
            self.assertTrue(compare_floats(packet.points[i].x, packet2.points[i].x))
            self.assertTrue(compare_floats(packet.points[i].y, packet2.points[i].y))
            if packet.points[i].angle == None:
                self.assertEqual(packet.points[i].angle, packet2.points[i].angle)
            else:
                self.assertTrue(compare_floats(packet.points[i].angle, packet2.points[i].angle))





class GotoStartedPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.GotoStarted()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.GotoStarted()
        buf = packet.serialize()
        packet2 = packets.GotoStarted()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class GotoFinishedPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.GotoFinished()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.GotoFinished()
        packet.reason = REASON_PIECE_FOUND
        packet.current_pose = trajectory.Pose(20.0, 30.0, 5.5)
        packet.current_point_index = 12
        buf = packet.serialize()
        packet2 = packets.GotoFinished()
        packet2.deserialize(buf)
        self.assertEqual(packet.reason, packet2.reason)
        self.assertTrue(compare_floats(packet.current_pose.x, packet2.current_pose.x))
        self.assertTrue(compare_floats(packet.current_pose.y, packet2.current_pose.y))
        self.assertTrue(compare_floats(packet.current_pose.angle, packet2.current_pose.angle))




class BlockedPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Blocked()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Blocked()
        packet.side = BLOCKED_BACK
        buf = packet.serialize()
        packet2 = packets.Blocked()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class EnableAntiBlockingPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.EnableAntiBlocking()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.EnableAntiBlocking()
        buf = packet.serialize()
        packet2 = packets.EnableAntiBlocking()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class DisableAntiBlockingPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.DisableAntiBlocking()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.DisableAntiBlocking()
        buf = packet.serialize()
        packet2 = packets.DisableAntiBlocking()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class KeepAlivePacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.KeepAlive()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.KeepAlive()
        packet.current_pose = trajectory.Pose(564.8, 452.3, 96.4)
        packet.match_started = True
        packet.match_time = 56
        buf = packet.serialize()
        packet2 = packets.KeepAlive()
        packet2.deserialize(buf)
        self.assertTrue(compare_floats(packet.current_pose.x, packet2.current_pose.x))
        self.assertTrue(compare_floats(packet.current_pose.y, packet2.current_pose.y))
        if packet.current_pose.angle == None:
            self.assertEqual(packet.current_pose.angle, packet2.current_pose.angle)
        else:
            self.assertTrue(compare_floats(packet.current_pose.angle, packet2.current_pose.angle))
        self.assertEqual(packet.match_started, packet2.match_started)
        self.assertEqual(packet.match_time, packet2.match_time)




class PositionControlConfigPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.PositionControlConfig()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.PositionControlConfig()
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

        buf = packet.serialize()
        packet2 = packets.PositionControlConfig()
        packet2.deserialize(buf)
        self.assertTrue(compare_floats(packet.t_acc, packet2.t_acc))
        self.assertTrue(compare_floats(packet.f_va_max, packet2.f_va_max))
        self.assertTrue(compare_floats(packet.u_max, packet2.u_max))
        self.assertTrue(compare_floats(packet.k1, packet2.k1))
        self.assertTrue(compare_floats(packet.k2, packet2.k2))
        self.assertTrue(compare_floats(packet.k3, packet2.k3))
        self.assertTrue(compare_floats(packet.g_re_1, packet2.g_re_1))
        self.assertTrue(compare_floats(packet.g_re_2, packet2.g_re_2))
        self.assertTrue(compare_floats(packet.dist_min, packet2.dist_min))
        self.assertTrue(compare_floats(packet.angle_min, packet2.angle_min))
        self.assertTrue(compare_floats(packet.nbr_pas_evit_detect, packet2.nbr_pas_evit_detect))
        self.assertTrue(compare_floats(packet.ecart_roue_libre, packet2.ecart_roue_libre))
        self.assertTrue(compare_floats(packet.ecart_roue_motrice, packet2.ecart_roue_motrice))
        self.assertTrue(compare_floats(packet.k_p_d, packet2.k_p_d))
        self.assertTrue(compare_floats(packet.k_i_d, packet2.k_i_d))
        self.assertTrue(compare_floats(packet.v_max_d, packet2.v_max_d))
        self.assertTrue(compare_floats(packet.d_roue_d, packet2.d_roue_d))
        self.assertTrue(compare_floats(packet.nb_pas_d, packet2.nb_pas_d))
        self.assertTrue(compare_floats(packet.k_p_g, packet2.k_p_g))
        self.assertTrue(compare_floats(packet.k_i_g, packet2.k_i_g))
        self.assertTrue(compare_floats(packet.v_max_g, packet2.v_max_g))
        self.assertTrue(compare_floats(packet.d_roue_g, packet2.d_roue_g))
        self.assertTrue(compare_floats(packet.nb_pas_g, packet2.nb_pas_g))
        self.assertEqual(packet.app_net_ip, packet2.app_net_ip)
        self.assertEqual(packet.app_net_mask, packet2.app_net_mask)
        self.assertEqual(packet.app_net_gateway, packet2.app_net_gateway)
        self.assertEqual(packet.app_net_port, packet2.app_net_port)
        self.assertEqual(packet.test_mode, packet2.test_mode)
        self.assertTrue(compare_floats(packet.coefficient_glissement_lateral, packet2.coefficient_glissement_lateral))




class StopPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Stop()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Stop()
        buf = packet.serialize()
        packet2 = packets.Stop()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class ResettlePacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Resettle()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Resettle()
        packet.axis = AXIS_Y
        packet.position = 684.87
        packet.angle = 156.5
        buf = packet.serialize()
        packet2 = packets.Resettle()
        packet2.deserialize(buf)
        self.assertEqual(packet.axis, packet2.axis)
        self.assertTrue(compare_floats(packet.position, packet2.position))
        self.assertTrue(compare_floats(packet.angle, packet2.angle))




class DeploymentPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Deployment()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Deployment()
        buf = packet.serialize()
        packet2 = packets.Deployment()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class PieceDetectedPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.PieceDetected()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization_with_angle(self):
        packet = packets.PieceDetected()
        packet.start_pose = trajectory.Pose(12.3, 67.9, 56.4)
        packet.start_distance = 54.8
        packet.end_pose = trajectory.Pose(2.3, 32.9, 98.3)
        packet.end_distance = 34.0
        packet.sensor = SENSOR_RIGHT_BOTTOM
        packet.angle = 32.8
        buf = packet.serialize()
        packet2 = packets.PieceDetected()
        packet2.deserialize(buf)
        self.assertTrue(compare_floats(packet.start_pose.x, packet2.start_pose.x))
        self.assertTrue(compare_floats(packet.start_pose.y, packet2.start_pose.y))
        self.assertTrue(compare_floats(packet.start_pose.angle, packet2.start_pose.angle))
        self.assertTrue(compare_floats(packet.start_distance, packet2.start_distance))
        self.assertTrue(compare_floats(packet.end_pose.x, packet2.end_pose.x))
        self.assertTrue(compare_floats(packet.end_pose.y, packet2.end_pose.y))
        self.assertTrue(compare_floats(packet.end_pose.angle, packet2.end_pose.angle))
        self.assertTrue(compare_floats(packet.end_distance, packet2.end_distance))
        self.assertEqual(packet.sensor, packet2.sensor)
        self.assertTrue(compare_floats(packet.angle, packet2.angle))




class StorePiece1PacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.StorePiece1()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.StorePiece1()
        packet.piece_count = 34
        buf = packet.serialize()
        packet2 = packets.StorePiece1()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class StorePiece2PacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.StorePiece2()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.StorePiece2()
        packet.piece_count = 52
        buf = packet.serialize()
        packet2 = packets.StorePiece2()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class StorePiece3PacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.StorePiece3()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.StorePiece3()
        packet.piece_count = 64
        buf = packet.serialize()
        packet2 = packets.StorePiece3()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class ReleasePiecePacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.ReleasePiece()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.ReleasePiece()
        buf = packet.serialize()
        packet2 = packets.ReleasePiece()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class OpenNippersPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.OpenNippers()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.OpenNippers()
        buf = packet.serialize()
        packet2 = packets.OpenNippers()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class CloseNippersPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.CloseNippers()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.CloseNippers()
        buf = packet.serialize()
        packet2 = packets.CloseNippers()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class ReinitializePacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.Reinitialize()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.Reinitialize()
        buf = packet.serialize()
        packet2 = packets.Reinitialize()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class SimulatorDataPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.SimulatorData()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.SimulatorData()
        buf = packet.serialize()
        packet2 = packets.SimulatorData()
        packet2.deserialize(buf)
        self.assertEqual(packet.__dict__, packet2.__dict__)




class TurretDetectPacketTestCase(unittest.TestCase):

    def test_serialization_length(self):
        packet = packets.TurretDetect()
        buf = packet.serialize()
        self.assertEqual(len(buf), packet.MAX_SIZE)

    def test_serialization_deserialization(self):
        packet = packets.TurretDetect()
        packet.angle = 8.64
        buf = packet.serialize()
        packet2 = packets.TurretDetect()
        packet2.deserialize(buf)
        self.assertTrue(compare_floats(packet.angle, packet2.angle))
