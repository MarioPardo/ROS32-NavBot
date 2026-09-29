#include <stddef.h>

#include "encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

typedef struct
{
  TIM_HandleTypeDef *htim;
  int16_t            last;      /* raw counter at previous update */
  int32_t            position;  /* signed counts since init */
  int32_t            velocity;  /* counts per second */
  int8_t             dir;       /* sign of velocity */
} encoder_t;

static encoder_t encoders[ENCODER_COUNT] =
{
  [ENCODER_LEFT]  = { .htim = &htim1 },
  [ENCODER_RIGHT] = { .htim = &htim3 },
};

static encoder_t *encoder_get(encoder_id_t encoder)
{
  if ((unsigned)encoder >= (unsigned)ENCODER_COUNT)
  {
    return NULL;
  }

  return &encoders[encoder];
}

void encoder_init(void)
{
  for (encoder_id_t i = 0; i < ENCODER_COUNT; i++)
  {
    __HAL_TIM_SET_COUNTER(encoders[i].htim, 0);
    HAL_TIM_Encoder_Start(encoders[i].htim, TIM_CHANNEL_ALL); //since both channel mode
  }
}

void encoder_update(uint32_t dt_ms)
{
  if (dt_ms == 0U)
  {
    return;
  }

  for (encoder_id_t i = 0; i < ENCODER_COUNT; i++)
  {
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(encoders[i].htim);

    /* 16-bit subtraction wraps correctly, so the counter is never reset. */
    int16_t delta = (int16_t)(now - encoders[i].last);
    encoders[i].last = now;

    encoders[i].position += delta;
    encoders[i].velocity = ((int32_t)delta * 1000) / (int32_t)dt_ms;
    encoders[i].dir = (int8_t)((delta > 0) - (delta < 0));
  }
}

int16_t encoder_get_count(encoder_id_t encoder)
{
  encoder_t *enc = encoder_get(encoder);

  return (enc != NULL) ? (int16_t)__HAL_TIM_GET_COUNTER(enc->htim) : 0;
}

int32_t encoder_get_position(encoder_id_t encoder)
{
  encoder_t *enc = encoder_get(encoder);

  return (enc != NULL) ? enc->position : 0;
}

int32_t encoder_get_velocity(encoder_id_t encoder)
{
  encoder_t *enc = encoder_get(encoder);

  return (enc != NULL) ? enc->velocity : 0;
}

int8_t encoder_get_dir(encoder_id_t encoder)
{
  encoder_t *enc = encoder_get(encoder);

  return (enc != NULL) ? enc->dir : 0;
}
