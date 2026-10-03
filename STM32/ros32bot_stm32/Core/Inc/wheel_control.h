#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include <stdint.h>

#include "motor.h"

/* One wheel's live state: rad/s, rad/s, permille. */
typedef struct
{
  float   target;  /* rad/s, what the loop is driving towards */
  float   meas;    /* rad/s, measured from the encoder */
  int16_t duty;    /* permille, last duty the loop asked the motor for */
} wheel_state_t;


void wheel_control_init(void);

void wheel_control_set_target(motor_id_t motor, float rad_s);

void wheel_control_set_enabled(uint8_t enable);

void wheel_control_update(uint32_t dt_ms);

void wheel_control_get_state(motor_id_t motor, wheel_state_t *out);

#endif
