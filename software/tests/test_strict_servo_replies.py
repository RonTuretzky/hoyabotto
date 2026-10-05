import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
from types import SimpleNamespace

import pytest
from scservo_sdk import PacketHandler
from scservo_sdk.scservo_def import COMM_RX_CORRUPT, COMM_SUCCESS, COMM_RX_TIMEOUT
from strict_servo_replies import guard_replies


def packet(data=(), error=0):
    p = [255, 255, 9, len(data) + 2, error, *data]
    return [*p, (~sum(p[2:])) & 255]


class Port:
    is_using = False
    def setPacketTimeout(self, length):
        pass


def bus_for(reply, *, guard=True, result=COMM_SUCCESS):
    ph = PacketHandler(0)
    ph.txPacket = lambda port, request: COMM_SUCCESS
    ph.rxPacket = lambda port: (reply, result)
    bus = SimpleNamespace(packet_handler=ph)
    if guard:
        guard_replies(bus)
    return bus


def test_reproduces_existing_index_error_on_write_ack():
    with pytest.raises(IndexError):
        bus_for(packet(), guard=False).packet_handler.read2ByteTxRx(Port(), 9, 56)


def test_write_ack_cannot_be_decoded_as_temperature():
    value, result, _ = bus_for(packet()).packet_handler.read1ByteTxRx(Port(), 9, 63)
    assert result == COMM_RX_CORRUPT


def test_short_reply_returns_comm_error_instead_of_index_error():
    value, result, _ = bus_for(packet()).packet_handler.read2ByteTxRx(Port(), 9, 56)
    assert result == COMM_RX_CORRUPT


@pytest.mark.parametrize("data,expected", [([34], 34), ([0x34, 0x12], 0x1234)])
def test_valid_reply_is_preserved(data, expected):
    ph = bus_for(packet(data)).packet_handler
    read = ph.read1ByteTxRx if len(data) == 1 else ph.read2ByteTxRx
    assert read(Port(), 9, 56) == (expected, COMM_SUCCESS, 0)


def test_group_read_does_not_ignore_fault_flags():
    ph = bus_for(packet([35], error=4)).packet_handler
    with pytest.raises(RuntimeError, match="fault flags 4"):
        ph.readRx(Port(), 9, 1)


def test_existing_transport_failure_preserved():
    assert bus_for([], result=COMM_RX_TIMEOUT).packet_handler.read2ByteTxRx(Port(), 9, 56)[1] == COMM_RX_TIMEOUT


def test_overlong_reply_rejected():
    ph = bus_for(packet([35, 45])).packet_handler
    assert ph.read1ByteTxRx(Port(), 9, 63)[1] == COMM_RX_CORRUPT


def test_group_read_validates_requested_length():
    ph = bus_for(packet()).packet_handler
    port = Port()
    ph.syncReadTx(port, 56, 2, [9], 1)
    assert ph.readRx(port, 9, 2)[1] == COMM_RX_CORRUPT
