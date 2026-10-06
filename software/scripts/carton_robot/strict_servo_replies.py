"""Task-local receive validation for the installed Feetech SDK.

The SDK validates checksums but accepts a status packet of the wrong payload
length. A write acknowledgement can therefore become a bogus one-byte value,
or cause IndexError for a two-byte read. Reject it before decoding. Existing
bus retry bounds and all motor limits remain in effect.
"""
from scservo_sdk.scservo_def import (
    COMM_RX_CORRUPT, COMM_SUCCESS, INST_READ, INST_SYNC_READ,
)
import time


def guard_replies(bus):
    """Install on one bus instance, before connecting; no hardware commands."""
    ph = bus.packet_handler
    if getattr(ph, "_strict_replies", False):
        return
    old_tx, old_rx = ph.txPacket, ph.rxPacket
    expected = {"length": None}
    events = []
    bus.reply_validation_events = events

    def record(reason, packet=None, **details):
        event = {"reason": reason, "captured_at": time.time(), **details}
        if packet is not None:
            event.update(packet=list(packet[:64]), packet_length=len(packet))
        events.append(event)
        del events[:-64]

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
            record("malformed status packet")
            return [], COMM_RX_CORRUPT
        if result != COMM_SUCCESS:
            # Preserve the SDK failure and bounded bytes; do not retry, flush,
            # reinterpret it as success, or attribute it to USB without evidence.
            record("SDK receive failure", packet, communication=result,
                   expected_payload_length=expected["length"])
            return packet, result
        size = expected["length"]
        if len(packet) < 6:
            record("short status packet", packet)
            return packet, COMM_RX_CORRUPT
        # GroupSyncRead discards the SDK error result. Fail here so a real
        # servo fault cannot disappear into a successful grouped reading.
        if packet[4]:
            record("servo fault flags", packet, packet_error=packet[4])
            raise RuntimeError(f"Servo {packet[2]} reported fault flags {packet[4]}")
        if size is not None and (packet[3] != size + 2 or len(packet) != size + 6):
            record("unexpected payload length", packet, expected=size)
            return packet, COMM_RX_CORRUPT
        return packet, result

    ph.txPacket, ph.rxPacket = tx, rx
    ph._strict_replies = True
