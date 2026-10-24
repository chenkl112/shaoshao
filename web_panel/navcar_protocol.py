"""Navcar UART1 V2 codec and single-flight ACK session; no ROS/serial dependency."""
from dataclasses import dataclass
import math
import struct
import time

HEADER = b'\xa5\x5a'
CONTROL, ACK, STATE = 1, 0x80, 0x81


def crc16_modbus(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def frame(kind, sequence, payload):
    if type(sequence) is not int or not 0 <= sequence <= 65535 or len(payload) > 64:
        raise ValueError('Invalid sequence or payload length')
    body = struct.pack('<BBHH', 1, kind, sequence, len(payload)) + payload
    return HEADER + body + struct.pack('<H', crc16_modbus(body))


def control(sequence, values):
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected finite vy,vx,rot,yaw_deg')
    try:
        payload = struct.pack('<4f', *values)
    except (OverflowError, struct.error) as exc:
        raise ValueError('Values must fit float32') from exc
    if not all(math.isfinite(v) for v in struct.unpack('<4f', payload)):
        raise ValueError('Values must fit finite float32')
    return frame(CONTROL, sequence, payload)


@dataclass(frozen=True)
class Packet:
    kind: int
    sequence: int
    values: tuple


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_rx = None

    def feed(self, data, now):
        if self.last_rx is not None and now - self.last_rx > .1:
            self.buffer.clear()
        if data:
            self.last_rx = now
            self.buffer.extend(data)
        packets = []
        while self.buffer:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = b'\xa5' if self.buffer[-1] == 0xa5 else b''
                break
            del self.buffer[:start]
            if len(self.buffer) < 8:
                break
            version, kind, seq, size = struct.unpack_from('<BBHH', self.buffer, 2)
            if size > 64:
                del self.buffer[0]
                continue
            total = size + 10
            if len(self.buffer) < total:
                break
            if crc16_modbus(self.buffer[2:8+size]) != struct.unpack_from('<H', self.buffer, 8+size)[0]:
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[8:8+size])
            del self.buffer[:total]
            if version != 1:
                continue
            if kind == ACK and size == 1:
                packets.append(Packet(kind, seq, (payload[0],)))
            elif kind == STATE and size == 16:
                values = struct.unpack('<4f', payload)
                speeds = values[:3]
                speeds_valid = (all(math.isfinite(v) for v in speeds) or
                                all(math.isnan(v) for v in speeds))
                if speeds_valid and math.isfinite(values[3]):
                    packets.append(Packet(kind, seq, values))
        return packets


class Session:
    """Each poll returns at most one packet to write; targets never queue up."""
    def __init__(self):
        self.parser = Parser()
        self.sequence = None
        self.state = None
        self.state_time = None
        self.pending = None
        self.sent_at = self.next_at = 0.
        self.retries = 0
        self.error = ''
        self.last_ack = None
        self.ack_time = None

    def receive(self, data, now):
        for packet in self.parser.feed(data, now):
            if packet.kind == STATE:
                self.state, self.state_time = packet, now
                if self.sequence is None and self.pending is None:
                    self.sequence = (packet.sequence + 1) & 65535
            elif self.pending is not None and packet.sequence == self.sequence:
                code = packet.values[0]
                self.last_ack = code
                if code in (0, 1):
                    self.pending = None
                    self.sequence = (self.sequence + 1) & 65535
                    self.ack_time = now
                else:
                    self.fail('MCU ACK=' + str(code))

    def fail(self, message):
        self.error = message
        self.pending = None
        self.sequence = None

    def poll(self, values, now):
        if self.error or self.sequence is None:
            return None
        if self.state_time is None or now - self.state_time > .3:
            self.fail('MCU status timeout; reconnect and synchronize')
            return None
        if self.pending is not None:
            if now - self.sent_at < .05:
                return None
            if self.retries >= 3:
                self.fail('ACK timeout after 3 retries; reconnect and synchronize')
                return None
            self.retries += 1
            self.sent_at = now
            return self.pending
        if now < self.next_at:
            return None
        self.pending = control(self.sequence, values)
        self.sent_at, self.next_at, self.retries = now, now + .02, 0
        return self.pending


def stop_confirmed(session, after):
    """Success requires an accepted zero command and a subsequent V2 state frame."""
    return (not session.error and session.pending is None and session.last_ack == 0
            and session.ack_time is not None and session.ack_time >= after
            and session.state_time is not None and session.state_time >= session.ack_time
            and session.state.sequence == ((session.sequence - 1) & 65535))
