"""Split official outer loop and real RPYT decoder command semantics."""
import ctypes as ct
import math
import struct
import unittest
import numpy as np
from .position_bridge import PCPositionPid
from .flight_controller import StockPid

class PositionBridgeTests(unittest.TestCase):
    def test_world_velocity_is_rotated_to_body_attitude_once(self):
        bridge=PCPositionPid([1,2,1])
        forward=bridge.update([1,2,1],[0,0,0],0.,[.2,0,0])
        bridge=PCPositionPid([1,2,1])
        rotated=bridge.update([1,2,1],[0,0,0],math.pi/2,[0,.2,0])
        self.assertLess(forward["pitch_deg"],0.)
        self.assertAlmostEqual(forward["roll_deg"],0.,places=5)
        self.assertAlmostEqual(forward["pitch_deg"],rotated["pitch_deg"],places=5)
        self.assertAlmostEqual(rotated["roll_deg"],0.,places=5)
        self.assertEqual(forward["legacy_collective"],rotated["legacy_collective"])

    def test_nonfinite_estimate_is_not_a_flight_command(self):
        bridge=PCPositionPid([1,2,1])
        with self.assertRaises(ValueError):bridge.update([1,2,1],[float("nan"),0,0],0.,[0,0,0])

    def test_real_decoder_requires_zero_unlock_and_zero_stops_all_motors(self):
        pid=StockPid()
        arrays=[(ct.c_float*3)(*v) for v in ([0,0,0],[0,0,0],[0,0,1])]
        def packet(tick,collective):
            raw=struct.pack("<fffH",0.,0.,0.,collective)
            wire=(ct.c_uint8*14).from_buffer_copy(raw)
            pid.lib.flight_update_rpyt_wire(tick,*arrays,wire,14,4.2,1)
            return [pid.lib.flight_motor(i) for i in range(4)]
        self.assertEqual(packet(0,35000),[0]*4)
        self.assertEqual(packet(2,0),[0]*4)
        self.assertTrue(all(v>0 for v in packet(4,35000)))
        self.assertEqual(packet(6,0),[0]*4)

if __name__=="__main__":unittest.main()
