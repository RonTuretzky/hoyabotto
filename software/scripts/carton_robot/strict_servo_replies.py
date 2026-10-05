"""Task-local receive validation for the installed Feetech SDK.

The SDK validates checksums but accepts a status packet of the wrong payload
length. A write acknowledgement can therefore become a bogus one-byte value,
or cause IndexError for a two-byte read. Reject it before decoding. Existing
bus retry bounds and all motor limits remain in effect.
"""
from scservo_sdk.scservo_def import (
    COMM_RX_CORRUPT, COMM_SUCCESS, INST_READ, INST_SYNC_READ,
)


def guard_replies(bus):
    """Install on one bus instance, before connecting; no hardware commands."""
    ph = bus.packet_handler
    if getattr(ph, "_strict_replies", False):
        return
    old_tx, old_rx = ph.txPacket, ph.rxPacket
    expected = {"length": None}
    events = []
    bus.reply_validation_events = events

    def tx(port, packet):
        # Do not change the transaction state when the SDK reports port-busy.
        if not port.is_using:
            instruction = packet[4]
            expected["length"] = packet[6] if instruction in (INST_READ, INST_SYNC_READ) else 0
        return old_tx(port, packet)

    def rx(port):
        try:
            packet, result = old_rx(port)
        except IndexError:
            port.is_using = False
            events.append({"reason": "malformed status packet"})
            return [], COMM_RX_CORRUPT
        if result != COMM_SUCCESS:
            return packet, result
        size = expected["length"]
        if len(packet) < 6:
            events.append({"reason": "short status packet", "packet": packet})
            return packet, COMM_RX_CORRUPT
        # GroupSyncRead discards the SDK error result. Fail here so a real
        # servo fault cannot disappear into a successful grouped reading.
        if packet[4]:
            raise RuntimeError(f"Servo {packet[2]} reported fault flags {packet[4]}")
        if size is not None and (packet[3] != size + 2 or len(packet) != size + 6):
            events.append({"reason": "unexpected payload length", "expected": size, "packet": packet})
            return packet, COMM_RX_CORRUPT
        return packet, result

    ph.txPacket, ph.rxPacket = tx, rx
    ph._strict_replies = True
