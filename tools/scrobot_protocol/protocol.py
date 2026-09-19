from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable

SOF = b"\xAA\x55"
PROTOCOL_VERSION = 2
MAX_PAYLOAD = 64

TYPE_SETPOINT = 0x10
TYPE_ARM = 0x11
TYPE_DISARM = 0x12
TYPE_FEEDBACK = 0x20
TYPE_DIAGNOSTICS = 0x21
TYPE_SYSID_COMMAND = 0x30
TYPE_SYSID_SAMPLE = 0x31
TYPE_SYSID_STOP = 0x32
TYPE_PID_SET = 0x40
TYPE_PID_GET = 0x41
TYPE_PID_RESPONSE = 0x42
TYPE_INFO_REQUEST = 0x50
TYPE_INFO_RESPONSE = 0x51
TYPE_ERROR = 0x7F

MOTOR_IDS = {"WR": 0, "WL": 1, "BR": 2, "BL": 3, "CV": 4}
MOTOR_NAMES = {v: k for k, v in MOTOR_IDS.items()}

STATUS_ARMED = 1 << 0
STATUS_ESTOP = 1 << 1
STATUS_COMM_TIMEOUT = 1 << 2
STATUS_SYSID = 1 << 3
STATUS_UART_ERROR = 1 << 4
STATUS_INVALID_OUTPUT = 1 << 5
STATUS_TX_DROP = 1 << 6
STATUS_INVALID_COMMAND = 1 << 7


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(slots=True)
class Frame:
    msg_type: int
    seq: int
    payload: bytes
    version: int = PROTOCOL_VERSION


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = struct.pack("<BBHB", PROTOCOL_VERSION, msg_type & 0xFF, seq & 0xFFFF, len(payload)) + payload
    crc = crc16_ccitt_false(body)
    return SOF + body + struct.pack("<H", crc)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        out: list[Frame] = []

        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                if self.buffer[-1:] == SOF[:1]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                del self.buffer[:start]

            if len(self.buffer) < 7:
                break

            version, msg_type, seq, payload_len = struct.unpack_from("<BBHB", self.buffer, 2)
            if payload_len > MAX_PAYLOAD:
                self.invalid_frames += 1
                del self.buffer[0]
                continue

            total = 2 + 5 + payload_len + 2
            if len(self.buffer) < total:
                break

            raw = bytes(self.buffer[:total])
            body = raw[2:-2]
            expected = struct.unpack_from("<H", raw, total - 2)[0]
            actual = crc16_ccitt_false(body)
            if actual != expected:
                self.crc_errors += 1
                del self.buffer[0]
                continue

            payload = raw[7:-2]
            out.append(Frame(msg_type=msg_type, seq=seq, payload=payload, version=version))
            del self.buffer[:total]

        return out


def pack_setpoint(values: Iterable[float]) -> bytes:
    values = tuple(values)
    if len(values) != 5:
        raise ValueError("setpoint requires WR, WL, BR, BL, CV")
    return struct.pack("<5f", *values)


def pack_sysid_command(motor_id: int, duty: float) -> bytes:
    return struct.pack("<Bf", motor_id, duty)


def pack_pid(motor_id: int, kp: float, ki: float, kd: float, tf: float) -> bytes:
    return struct.pack("<B4f", motor_id, kp, ki, kd, tf)


def decode_feedback(payload: bytes) -> dict:
    if len(payload) != 50:
        raise ValueError(f"feedback payload length {len(payload)} != 50")
    control_tick, status, last_setpoint_seq = struct.unpack_from("<IIH", payload, 0)
    counts = struct.unpack_from("<5i", payload, 10)
    rpm = struct.unpack_from("<5f", payload, 30)
    return {
        "control_tick": control_tick,
        "status": status,
        "last_setpoint_seq": last_setpoint_seq,
        "counts": dict(zip(("WR", "WL", "BR", "BL", "CV"), counts)),
        "rpm": dict(zip(("WR", "WL", "BR", "BL", "CV"), rpm)),
    }


def decode_diagnostics(payload: bytes) -> dict:
    if len(payload) != 32:
        raise ValueError(f"diagnostics payload length {len(payload)} != 32")
    vals = struct.unpack("<7I4B", payload)
    return {
        "uptime_ms": vals[0],
        "reset_flags_raw": vals[1],
        "rx_frames_ok": vals[2],
        "crc_errors": vals[3],
        "invalid_frames": vals[4],
        "uart_errors": vals[5],
        "tx_queue_drops": vals[6],
        "fw_version": f"{vals[7]}.{vals[8]}.{vals[9]}",
        "protocol_version": vals[10],
    }


def decode_sysid_sample(payload: bytes) -> dict:
    if len(payload) != 23:
        raise ValueError(f"sysid payload length {len(payload)} != 23")
    command_seq, control_tick, motor_id = struct.unpack_from("<HIB", payload, 0)
    duty, rpm, encoder_count, status = struct.unpack_from("<ffiI", payload, 7)
    return {
        "command_seq": command_seq,
        "control_tick": control_tick,
        "motor_id": motor_id,
        "motor": MOTOR_NAMES.get(motor_id, f"ID{motor_id}"),
        "duty": duty,
        "rpm": rpm,
        "encoder_count": encoder_count,
        "status": status,
    }


def decode_pid_response(payload: bytes) -> dict:
    if len(payload) != 17:
        raise ValueError(f"PID payload length {len(payload)} != 17")
    motor_id, kp, ki, kd, tf = struct.unpack("<B4f", payload)
    return {"motor_id": motor_id, "motor": MOTOR_NAMES.get(motor_id, str(motor_id)),
            "kp": kp, "ki": ki, "kd": kd, "tf": tf}


def decode_info(payload: bytes) -> dict:
    if len(payload) != 4:
        raise ValueError(f"info payload length {len(payload)} != 4")
    major, minor, patch, protocol = struct.unpack("<4B", payload)
    return {"fw_version": f"{major}.{minor}.{patch}", "protocol_version": protocol}


def decode_error(payload: bytes) -> dict:
    if len(payload) != 2:
        raise ValueError(f"error payload length {len(payload)} != 2")
    request_type, code = struct.unpack("<2B", payload)
    return {"request_type": request_type, "code": code}


def status_names(status: int) -> list[str]:
    mapping = [
        (STATUS_ARMED, "ARMED"),
        (STATUS_ESTOP, "ESTOP"),
        (STATUS_COMM_TIMEOUT, "COMM_TIMEOUT"),
        (STATUS_SYSID, "SYSID"),
        (STATUS_UART_ERROR, "UART_ERROR"),
        (STATUS_INVALID_OUTPUT, "INVALID_OUTPUT"),
        (STATUS_TX_DROP, "TX_DROP"),
        (STATUS_INVALID_COMMAND, "INVALID_COMMAND"),
    ]
    return [name for bit, name in mapping if status & bit]
