import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
from types import SimpleNamespace

import pytest
from scservo_sdk.scservo_def import COMM_SUCCESS, COMM_RX_TIMEOUT, COMM_TX_FAIL
from coherent_servo_telemetry import decode_telemetry, read_servo_telemetry


def payload(position=3198, velocity=0, load=0, status=0):
    data = [0] * 15
    for offset, value in ((0, position), (2, velocity), (4, load), (13, 513)):
        data[offset:offset+2] = [value & 255, value >> 8]
    data[6], data[7], data[9], data[10] = 120, 37, status, 1
    # Reserved bytes must not be mistaken for status/current.
    data[8], data[11], data[12] = 255, 254, 253
    return data


def test_one_complete_transaction_and_exact_field_offsets():
    calls = []
    port = object()
    def read(*args):
        calls.append(args)
        return payload(), COMM_SUCCESS, 0
    bus = SimpleNamespace(port_handler=port, motors={'elbow': SimpleNamespace(id=3, model='sts3215')},
                          packet_handler=SimpleNamespace(readTxRx=read))
    assert read_servo_telemetry(bus, 'elbow') == {
        'Present_Position': 3198, 'Present_Velocity': 0, 'Present_Load': 0,
        'Present_Voltage': 120, 'Status': 0,
        'Moving': 1, 'Present_Current': 513}
    assert calls == [(port, 3, 56, 15)]


def test_sign_magnitude_and_health_status_are_preserved():
    row = decode_telemetry(payload(0x8000 | 123, 0x8000 | 200, 0x400 | 128, 32), COMM_SUCCESS, 0)
    assert (row['Present_Position'], row['Present_Velocity'], row['Present_Load']) == (-123, -200, -128)
    assert row['Status'] == 32  # Owner must log before rejecting this health sample.
    assert row['Present_Current'] == 513


@pytest.mark.parametrize('data', [[], [0], [0]*14, [0]*16, None, [False]*15, [256]*15, [-1]*15])
def test_malformed_payload_never_becomes_a_normal_health_reading(data):
    with pytest.raises(RuntimeError):
        decode_telemetry(data, COMM_SUCCESS, 0)


@pytest.mark.parametrize('communication,error', [(COMM_RX_TIMEOUT, 0), (COMM_TX_FAIL, 0), (COMM_SUCCESS, 32)])
def test_failed_transaction_and_packet_fault_reject_before_decoding(communication, error):
    with pytest.raises(RuntimeError):
        decode_telemetry(payload(), communication, error)


def test_failed_read_is_not_retried():
    calls = []
    def read(*args):
        calls.append(args)
        return [], COMM_RX_TIMEOUT, 0
    bus = SimpleNamespace(port_handler=object(), motors={'j': SimpleNamespace(id=3, model='sts3215')},
                          packet_handler=SimpleNamespace(readTxRx=read))
    with pytest.raises(RuntimeError):
        read_servo_telemetry(bus, 'j')
    assert len(calls) == 1
