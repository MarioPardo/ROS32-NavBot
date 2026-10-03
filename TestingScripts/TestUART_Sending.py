#!/usr/bin/env python3
"""TestUART_Sending.py - send preset frames to the STM32 over UART.

Mirrors ROS_ws/src/ros32bot_middleware/src/package_bytes.cpp, so the bytes
this puts on the wire are the bytes ros2_control sends. Used on the Pi to
check that the STM32 receives and parses frames (watch SEGGER RTT).

Usage:
    python3 TestUART_Sending.py [port]

Defaults to /dev/serial0 at 115200 8N1, the link the middleware opens.
"""

import sys
import time

import serial

SYNC_0 = 0xA5
SYNC_1 = 0x5A

# Must match uart_comms.h
MSG_SET_WHEEL_VEL = 0x01

DEFAULT_PORT = "/dev/ttyAMA0"
BAUD_RATE = 115200
PERIOD_S = 1.0

seq = 0


def preset_message():
    """Edit me: the wheel targets sent by the test, in centirad/s (100 = 1 rad/s)."""
    left_crad_s = 100
    right_crad_s = 100

    return left_crad_s, right_crad_s


# CCITT-FALSE. Same polynomial and seed as the STM32 side.
def crc16_ccitt(body):
    crc = 0xFFFF

    for byte in body:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def int16(value):
    """Little endian two's complement, like appendInt16 in package_bytes.cpp."""
    return (value & 0xFFFF).to_bytes(2, "little")


# Wraps id, seq, len and payload in sync bytes and a trailing crc.
def make_frame(msg_id, payload):
    global seq

    body = bytes([msg_id, seq, len(payload)]) + payload
    seq = (seq + 1) % 256

    return bytes([SYNC_0, SYNC_1]) + body + crc16_ccitt(body).to_bytes(2, "little")


def package_wheel_velocities(left_crad_s, right_crad_s):
    payload = int16(left_crad_s) + int16(right_crad_s)

    return make_frame(MSG_SET_WHEEL_VEL, payload)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    print(f"opening {port} at {BAUD_RATE} 8N1")

    with serial.Serial(port, BAUD_RATE, timeout=1) as uart:
        try:
            while True:
                left_crad_s, right_crad_s = preset_message()
                frame = package_wheel_velocities(left_crad_s, right_crad_s)

                uart.write(frame)
                print(f"sent L={left_crad_s} R={right_crad_s} crad/s: {frame.hex(' ')}")

                time.sleep(PERIOD_S)
        except KeyboardInterrupt:
            print("stopped")


if __name__ == "__main__":
    main()
