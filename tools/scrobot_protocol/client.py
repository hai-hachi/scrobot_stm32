from __future__ import annotations

import time
import serial

from .protocol import (
    FrameParser, encode_frame,
    TYPE_SETPOINT, TYPE_ARM, TYPE_DISARM, TYPE_SYSID_COMMAND, TYPE_SYSID_STOP,
    TYPE_PID_SET, TYPE_PID_GET, TYPE_INFO_REQUEST,
    pack_setpoint, pack_sysid_command, pack_pid,
)


class SerialClient:
    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.02):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.parser = FrameParser()
        self.seq = 0

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def next_seq(self) -> int:
        value = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        return value

    def send(self, msg_type: int, payload: bytes = b"", seq: int | None = None) -> int:
        if seq is None:
            seq = self.next_seq()
        self.ser.write(encode_frame(msg_type, seq, payload))
        return seq

    def arm(self) -> int:
        return self.send(TYPE_ARM)

    def disarm(self) -> int:
        return self.send(TYPE_DISARM)

    def info(self) -> int:
        return self.send(TYPE_INFO_REQUEST)

    def setpoint(self, wr=0.0, wl=0.0, br=0.0, bl=0.0, cv=0.0) -> int:
        return self.send(TYPE_SETPOINT, pack_setpoint((wr, wl, br, bl, cv)))

    def sysid(self, motor_id: int, duty: float) -> int:
        return self.send(TYPE_SYSID_COMMAND, pack_sysid_command(motor_id, duty))

    def sysid_stop(self) -> int:
        return self.send(TYPE_SYSID_STOP)

    def pid_set(self, motor_id: int, kp: float, ki: float, kd: float, tf: float) -> int:
        return self.send(TYPE_PID_SET, pack_pid(motor_id, kp, ki, kd, tf))

    def pid_get(self, motor_id: int) -> int:
        return self.send(TYPE_PID_GET, bytes((motor_id,)))

    def read_frames(self, duration: float = 0.0):
        deadline = time.monotonic() + duration
        first = True
        while first or time.monotonic() < deadline:
            first = False
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                for frame in self.parser.feed(data):
                    yield frame
            elif duration <= 0.0:
                break

    def drain(self, duration: float = 0.1):
        return list(self.read_frames(duration))
