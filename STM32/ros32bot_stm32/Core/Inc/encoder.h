#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum
{
  ENCODER_LEFT = 0,   /* TIM1: PA8 (A), PA9 (B) */
  ENCODER_RIGHT,      /* TIM3: PA6 (A), PA7 (B) */
  ENCODER_COUNT
} encoder_id_t;

/* 11 pulse/rev through the 35:1 gearbox, counted x4 by the quadrature decoder. */
#define ENCODER_COUNTS_PER_REV 1540

void encoder_init(void);

/* Sample both encoders. Call from a fixed-rate loop; dt_ms is that period. */
void encoder_update(uint32_t dt_ms);

/* Live 16-bit timer counter (signed, wraps at +/-32768). */
int16_t encoder_get_count(encoder_id_t encoder);

/* Signed counts accumulated since encoder_init(). */
int32_t encoder_get_position(encoder_id_t encoder);

/* Signed speed over the last encoder_update() period, in counts/second. */
int32_t encoder_get_velocity(encoder_id_t encoder);

/* 1 = forward, -1 = backward, 0 = stopped. */
int8_t encoder_get_dir(encoder_id_t encoder);

#endif
