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
| 0x01 | SET_WHEEL_VEL | int16 left_mm_s, int16 right_mm_s |
| 0x03 | SET_ENABLE | uint8 enabled |

STM32 -> Pi

| id | name | payload |
|---|---|---|
| 0x10 | ODOM | int32 left_mm, int32 right_mm, int16 left_mm_s, int16 right_mm_s, uint16 dt_ms |
| 0x20 | IMU | not decided yet |
| 0x30 | STATUS | not decided yet |

## Notes

- Odometry is in millimetres. The tick-to-mm conversion lives on the STM32,
  so the Pi never sees raw encoder counts.
- The Pi sends per-wheel targets, not `cmd_vel`. The diff drive mix happens
  on the Pi, so the rest of the stack only ever sees standard ROS messages.
- 115200 gives about 11.5 kB/s. ODOM at 50 Hz plus IMU at 100 Hz is under
  4 kB/s, so there is plenty of room left.
- No watchdog yet. Once wheel commands are real, losing them for 500 ms
  should stop the motors.
