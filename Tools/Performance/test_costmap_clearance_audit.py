#!/usr/bin/env python3

import base64
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
import zlib

from audit_nav2_costmap_clearance import audit, decode_snapshot, world_to_grid


def snapshot(data, size_x=5, size_y=5, resolution=1.0):
    raw = bytes(data)
    return {
        'frameId': 'odom',
        'stamp': {'sec': 1, 'nanosec': 2},
        'resolution': resolution,
        'sizeX': size_x,
        'sizeY': size_y,
        'origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'dataEncoding': 'base64+zlib+uint8-row-major',
        'dataSha256': hashlib.sha256(raw).hexdigest(),
        'data': base64.b64encode(zlib.compress(raw)).decode('ascii'),
    }


class CostmapClearanceAuditTests(unittest.TestCase):
    def test_decode_rejects_corrupt_digest(self):
        value = snapshot([0] * 25)
        value['dataSha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'SHA-256'):
            decode_snapshot(value)

    def test_world_to_grid_respects_rotated_origin(self):
        value = snapshot([0] * 25)
        value['origin'] = {'x': 10.0, 'y': 20.0, 'yaw': math.pi / 2.0}
        self.assertEqual(world_to_grid(value, 9.5, 20.5), (0, 0))

    def test_audit_detects_blocked_goal_and_disconnected_grid(self):
        data = [0] * 25
        data[2 * 5 + 2] = 254
        summary = {
            'scope': 'synthetic',
            'latestCostmapSnapshot': snapshot(data),
            'actionResultPose': {'x': 0.5, 'y': 0.5, 'yaw': 0.0},
        }
        result = audit(summary, 2.5, 2.5, 0.0, 1.0, 0.4, 0.8, 1.0)
        self.assertEqual(result['goal']['cost'], 254)
        self.assertTrue(result['goal']['blockedAt253'])
        self.assertFalse(result['connectedBelowCost253'])
        self.assertEqual(result['approach']['minimumLethalClearanceMeters'], 0.0)


if __name__ == '__main__':
    unittest.main()
