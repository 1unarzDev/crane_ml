#!/usr/bin/env python3
"""Load and validate an explicit odom-frame path for the Nav2 FollowPath fixture."""

import hashlib
import json
import math
from pathlib import Path


SCHEMA = 'crane-nav2-path-v1'


def load_path_spec(path):
    source = Path(path)
    raw = source.read_bytes()
    value = json.loads(raw)
    if value.get('schema') != SCHEMA:
        raise ValueError(f"unsupported path schema: {value.get('schema')}")
    frame_id = value.get('frameId')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError('path frameId must be a non-empty string')
    poses = value.get('poses')
    if not isinstance(poses, list) or not poses:
        raise ValueError('path poses must be a non-empty list')
    normalized = []
    for index, pose in enumerate(poses):
        if not isinstance(pose, dict):
            raise ValueError(f'path pose {index} must be an object')
        row = {}
        for field in ('x', 'y', 'yaw'):
            number = pose.get(field)
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f'path pose {index} {field} must be numeric')
            number = float(number)
            if not math.isfinite(number):
                raise ValueError(f'path pose {index} {field} must be finite')
            row[field] = number
        normalized.append(row)
    return {
        'frameId': frame_id,
        'poses': normalized,
        'sourcePath': str(source),
        'sourceSha256': hashlib.sha256(raw).hexdigest(),
    }
