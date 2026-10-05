"""One read-only STS3215 telemetry transaction, registers 56 through 70.

The installed LeRobot Feetech STS_SMS_SERIES_CTRL_TABLE defines these offsets.
Position/velocity use sign-magnitude bit 15; load uses bit 10. Current is unsigned.
No connection, retry, motor write, calibration normalization or status filtering.
Install strict_servo_replies.guard_replies on the bus before connecting, as the
owner already does, to reject wrong-length/servo-error packets before SDK decoding.
"""
from scservo_sdk.scservo_def import COMM_SUCCESS

ADDRESS = 56
LENGTH = 15


def decode_telemetry(data, communication, packet_error):
    """Validate the complete SDK reply before decoding any health field."""
    if communication != COMM_SUCCESS:
        raise RuntimeError(f'Coherent servo read communication failure: {communication}')
    if packet_error:
        raise RuntimeError(f'Coherent servo read packet fault: {packet_error}')
    if not isinstance(data, (list, tuple, bytes, bytearray)) or len(data) != LENGTH:
        raise RuntimeError('Coherent servo read requires exactly 15 payload bytes')
    if any(type(value) is not int or not 0 <= value <= 255 for value in data):
        raise RuntimeError('Coherent servo read contains invalid payload bytes')
    def word(offset):
        return data[offset] | (data[offset + 1] << 8)
    def signed(value, bit):
        return -(value & ~(1 << bit)) if value & (1 << bit) else value
    # Status is deliberately returned, even when nonzero: persist this exact
    # sample BEFORE the owner's existing health guard evaluates it.
    return {
        'Present_Position': signed(word(0), 15),
        'Present_Velocity': signed(word(2), 15),
        'Present_Load': signed(word(4), 10),
        'Present_Voltage': data[6],
        'Present_Temperature': data[7],
        'Status': data[9],
        'Moving': data[10],
        'Present_Current': word(13),
    }


def read_servo_telemetry(bus, name):
    """Exactly one SDK readTxRx; caller retains all bus/device ownership."""
    motor = bus.motors[name]
    if motor.model != 'sts3215':
        raise ValueError('Coherent telemetry mapping is restricted to STS3215')
    data, communication, packet_error = bus.packet_handler.readTxRx(
        bus.port_handler, motor.id, ADDRESS, LENGTH)
    return decode_telemetry(data, communication, packet_error)
