#include <stdbool.h>
#include <string.h>

#include "uart_comms.h"

#include "main.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"

extern UART_HandleTypeDef huart3;

#define RX_RING_SIZE  64U
#define PARSE_BUF_SIZE (UART_HEADER_BYTES + UART_MAX_PAYLOAD + UART_CRC_BYTES)

/* Written by the USART3 interrupt, drained by the comms task. */
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t rx_byte;

/* Bytes accepted so far, waiting for a whole frame to show up. */
static uint8_t parse_buf[PARSE_BUF_SIZE];
static uint16_t parse_len;

static uint8_t tx_seq;
static uint8_t last_seq;
static bool seq_seen;

///// Utilities ///////

//error detecting code
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t bit = 0; bit < 8U; bit++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
  }

  return crc;
}


static int16_t read_i16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}


static bool rx_ring_pop(uint8_t *out)
{
  if (rx_tail == rx_head)
  {
    return false;
  }

  *out = rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
  return true;
}

/* Throw away the first n bytes, keeping the rest. */
static void parse_drop(uint16_t n)
{
  if (n >= parse_len)
  {
    parse_len = 0U;
    return;
  }

  memmove(parse_buf, &parse_buf[n], (size_t)(parse_len - n));
  parse_len = (uint16_t)(parse_len - n);
}

///// Deliver ///////

/* 0x01: wheel velocity targets in mm/s. */
static void deliver_wheel_vel(const uint8_t *payload, uint8_t len)
{
  if (len != 4U)
  {
    return;
  }

  int16_t left  = read_i16(payload);
  int16_t right = read_i16(payload + 2U);

  /* TODO: hand these to the motor PI loop, once it exists.
     motor_set_velocity(MOTOR_LEFT, left);
     motor_set_velocity(MOTOR_RIGHT, right); */
  SEGGER_RTT_printf(0, "cmd  wheel_vel  L=%d R=%d mm/s\r\n", left, right);
}

/* 0x03: global enable. */
static void deliver_enable(const uint8_t *payload, uint8_t len)
{
  if (len != 1U)
  {
    return;
  }

  SEGGER_RTT_printf(0, "cmd  enable %u\r\n", (unsigned)payload[0]);
}


//distributes received message to correct subsystem
static void deliver_message(uint8_t msg_id, uint8_t seq, const uint8_t *payload, uint8_t len)
{
  if (!seq_seen)
  {
    seq_seen = true;
  }
  else if (seq != (uint8_t)(last_seq + 1U))
  {
    SEGGER_RTT_printf(0, "uart dropped frame: seq %u -> %u\r\n", (unsigned)last_seq, (unsigned)seq);
  }
  last_seq = seq;

  switch (msg_id)
  {
    case MSG_SET_WHEEL_VEL:
      deliver_wheel_vel(payload, len);
      break;

    case MSG_SET_ENABLE:
      deliver_enable(payload, len);
      break;

    default:
      SEGGER_RTT_printf(0, "uart unknown msg 0x%02X\r\n", (unsigned)msg_id);
      break;
  }
}

///// Receiving ///////

/* Pull as many whole frames as we can out of parse_buf and handle it */
static void parse_frames(void)
{
  for (;;)
  {

    if (parse_len < UART_HEADER_BYTES)
    {
      return;
    }

    /* Lost sync: shift along one byte and look again. */
    if ((parse_buf[0] != UART_SYNC_0) || (parse_buf[1] != UART_SYNC_1))
    {
      parse_drop(1U);
      continue;
    }

    //check length
    uint8_t len = parse_buf[4];
    if (len > UART_MAX_PAYLOAD)
    {
      parse_drop(UART_HEADER_BYTES);
      continue;
    }

    uint16_t total = (uint16_t)(UART_HEADER_BYTES + len + UART_CRC_BYTES);
    if (parse_len < total)
    {
      return; /* rest of the frame is still arriving */
    }


    //check message integrity
    uint16_t crc_rx = (uint16_t)(parse_buf[UART_HEADER_BYTES + len] |
                                 ((uint16_t)parse_buf[UART_HEADER_BYTES + len + 1U] << 8));

    if (crc16_ccitt(&parse_buf[2], (uint16_t)(3U + len)) == crc_rx)
      deliver_message(parse_buf[2], parse_buf[3], &parse_buf[UART_HEADER_BYTES], len);
    else
      SEGGER_RTT_printf(0, "uart bad crc\r\n");


    //message parsed, delete it
    parse_drop(total);
  }
}


//take whats been received by UART and put it into our parsing buffer
static void poll_receive(void)
{
  uint8_t byte;

  while (rx_ring_pop(&byte))
  {
    if (parse_len < PARSE_BUF_SIZE)
    {
      parse_buf[parse_len++] = byte;
    }
  }

  parse_frames();
}

///// Sending ///////

void uart_comms_send(uint8_t msg_id, const void *payload, uint8_t len)
{
  uint8_t frame[PARSE_BUF_SIZE];

  if (len > UART_MAX_PAYLOAD)
  {
    return;
  }

  frame[0] = UART_SYNC_0;
  frame[1] = UART_SYNC_1;
  frame[2] = msg_id;
  frame[3] = tx_seq++;
  frame[4] = len;

  if (len > 0U)
  {
    memcpy(&frame[UART_HEADER_BYTES], payload, len);
  }

  uint16_t crc = crc16_ccitt(&frame[2], (uint16_t)(3U + len));
  frame[UART_HEADER_BYTES + len]      = (uint8_t)(crc & 0xFFU);
  frame[UART_HEADER_BYTES + len + 1U] = (uint8_t)(crc >> 8);

  HAL_UART_Transmit(&huart3, frame, (uint16_t)(UART_HEADER_BYTES + len + UART_CRC_BYTES), 100U);
}

///// Task ///////

static void uart_comms_init(void)
{
  /* Nothing here calls the RTOS from an ISR, so priority 6 is safe. */
  HAL_NVIC_SetPriority(USART3_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void uart_comms_task(void *argument)
{
  (void)argument;

  uart_comms_init();
  SEGGER_RTT_printf(0, "uart ready on USART3\r\n");

  for (;;)
  {
    poll_receive();
    osDelay(1U);
  }
}

/* CubeMX generated no handler for USART3, so this takes over the weak one
   from the startup file. */
void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);

  if (next != rx_tail) /* drop the byte if the task has fallen behind */
  {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }

  HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

/* HAL treats an overrun as blocking: it disables the RX interrupt and leaves
   the receiver off. Without this the link goes deaf until the board is reset,
   which is exactly what happens if the ISR ever misses a byte. Clear the flags
   and start listening again. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  huart->ErrorCode = HAL_UART_ERROR_NONE;

  (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}
