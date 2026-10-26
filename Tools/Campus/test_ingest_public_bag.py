"""Prevent physical-bag ingestion from fabricating timing or frame evidence."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from ingest_public_bag import flu_pose, ingest_tum, stamp_seconds


def test_unstamped_command_has_no_acquisition_time():
    assert stamp_seconds(NS(linear=NS(x=1))) is None
    assert stamp_seconds(NS(header=NS(stamp=NS(sec=0,nanosec=0)))) == 0


def test_ros_left_turn_and_position_convert_to_unity():
    pose=flu_pose(NS(x=2,y=3,z=4), NS(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
    assert pose['position']==dict(x=-3,y=4,z=2)
    assert pose['yaw']==pytest.approx(-90)


def test_reference_retains_epoch_and_unknown_clock_uncertainty(tmp_path):
    source=tmp_path/'gt.txt'; output=tmp_path/'reference.jsonl'
    source.write_text('1700000000.0 1 2 3 0 0 0 1\n1700000000.1 2 2 3 0 0 0 1\n')
    report=ingest_tum(source, output)
    rows=[json.loads(line) for line in output.read_text().splitlines()]
    assert rows[0]['acquisitionTime']==1700000000.0
    assert rows[0]['clockUncertaintySeconds'] is None
    assert report['sampledPathMeters']==1


@pytest.mark.parametrize('bad', ['0 1 2 3 0 0 0 0', '0 nan 2 3 0 0 0 1', '0 1 2 3 0 0 0 1'])
def test_invalid_or_duplicate_reference_is_rejected(tmp_path, bad):
    source=tmp_path/'gt.txt'
    source.write_text('0 1 2 3 0 0 0 1\n'+bad+'\n')
    with pytest.raises(ValueError):
        ingest_tum(source, tmp_path/'out.jsonl')
