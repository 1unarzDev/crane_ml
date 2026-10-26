import base64
import hashlib
import math
import pytest
from physical_ros_capture import clean, envelope, parse_topics


def test_raw_payload_preserved_when_json_view_has_invalid_depth():
    raw = b'\x00\x01\xff\x7f'
    row = envelope('/depth', 'sensor_msgs/msg/Image', 1,
                   {'header': {'stamp': {'sec': 2, 'nanosec': 3}}, 'sample': math.nan},
                   raw, 10, 20, 30, 'synthetic-fixture')
    assert base64.b64decode(row['cdrBase64']) == raw
    assert row['cdrSha256'] == hashlib.sha256(raw).hexdigest()
    assert row['message']['sample'] is None
    assert row['acquisitionStamp'] == {'sec': 2, 'nanosec': 3}
    assert row['receptionRosNanoseconds'] == 10
    assert row['receptionMonotonicNanoseconds'] == 20
    assert row['dataSource'] == 'synthetic-fixture'


def test_unstamped_cmd_never_invents_acquisition_time():
    row = envelope('/cmd_vel', 'geometry_msgs/msg/Twist', 1, {}, b'', 1, 2, 3, 'physical')
    assert row['acquisitionStamp'] is None


@pytest.mark.parametrize('values', [ ['/a=bad'], ['a=nav_msgs/msg/Odometry'],
                                     ['/a=nav_msgs/msg/Odometry', '/a=sensor_msgs/msg/Imu']])
def test_invalid_topic_contract_rejected(values):
    with pytest.raises(ValueError):
        parse_topics(values)


def test_dynamic_topic_type_contract():
    assert parse_topics(['/odom=nav_msgs/msg/Odometry'])[0]['messageType'] == 'nav_msgs/msg/Odometry'
