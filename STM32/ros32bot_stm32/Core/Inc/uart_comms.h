#ifndef UART_COMMS_H
#define UART_COMMS_H

#include <stdint.h>

/*
  Frame: A5 5A | id | seq | len | payload | crc16

  crc16 is CCITT-FALSE over id, seq, len and payload, sent little endian.
  Payload fields are little endian too. See STM32/UART_PROTOCOL.md.
*/
#define UART_SYNC_0       0xA5U
#define UART_SYNC_1       0x5AU
#define UART_HEADER_BYTES 5U
#define UART_CRC_BYTES    2U
#define UART_MAX_PAYLOAD  32U

/* Pi -> STM32
   0x01 SET_WHEEL_VEL   int16 left_mm_s, int16 right_mm_s
   0x03 SET_ENABLE      uint8 enabled
*/
#define MSG_SET_WHEEL_VEL  0x01U
#define MSG_SET_ENABLE     0x03U

/* STM32 -> Pi
   0x10 ODOM    int32 left_mm, int32 right_mm, int16 left_mm_s,
                int16 right_mm_s, uint16 dt_ms
*/
#define MSG_ODOM   0x10U
#define MSG_IMU    0x20U
#define MSG_STATUS 0x30U

/* Owns USART3: sets up the interrupt and runs the receive loop. Never returns. */
void uart_comms_task(void *argument);

/* Frames one message and blocks until it is on the wire. */
void uart_comms_send(uint8_t msg_id, const void *payload, uint8_t len);

#endif
