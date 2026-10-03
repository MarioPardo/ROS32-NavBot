# UART protocol

How the Pi and the STM32 talk. 115200 8N1 on USART3 (PB10 TX / PB11 RX).

The Pi owns ROS. Over this link it sends wheel targets down, and the STM32
reports back what the robot is actually doing.

## Frame

    A5 5A | id | seq | len | payload | crc16

| field | notes |
|---|---|---|
| `A5 5A` | sync |
| `id` | message type |
| `seq` | wraps at 255, lets you spot dropped frames |
| `len` | payload bytes |
| `crc16` | CCITT-FALSE over `id`, `seq`, `len` and payload, little endian |

Everything is little endian.

A receiver that loses sync scans for `A5 5A` and checks the CRC, so a corrupt
frame gets dropped instead of wedging the parser. Unknown ids are ignored, so
either end can add messages without breaking the other.

## Messages

Pi -> STM32

| id | name | payload |
|---|---|---|
| 0x01 | SET_WHEEL_VEL | int16 left_crad_s, int16 right_crad_s |
| 0x03 | SET_ENABLE | uint8 enabled |

STM32 -> Pi

| id | name | payload |
|---|---|---|
| 0x10 | ODOM | int32 left_crad, int32 right_crad, int16 left_crad_s, int16 right_crad_s, uint16 dt_ms |
| 0x20 | IMU | not decided yet |
| 0x30 | STATUS | not decided yet |

## Notes

- Wheel speeds and angles are in radians: centirad/s and centirad on the wire
  (value / 100 = rad/s). The count-to-radian conversion lives on the STM32, so
  the Pi never sees raw encoder counts, and the firmware needs no wheel radius.
- The Pi sends per-wheel targets, not `cmd_vel`. The diff drive mix happens
  on the Pi, so the rest of the stack only ever sees standard ROS messages.
- 115200 gives about 11.5 kB/s. ODOM at 50 Hz plus IMU at 100 Hz is under
  4 kB/s, so there is plenty of room left.
- No watchdog yet. Wheel commands are real now, so losing the link leaves the
  last target latched. TestUART_Sending.py sends at 1 Hz, so any timeout
  shorter than that would fight the test.
