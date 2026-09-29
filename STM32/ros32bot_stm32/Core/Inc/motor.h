#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_DUTY_MAX 1000

typedef enum
{
  MOTOR_LEFT = 0,
  MOTOR_RIGHT,
  MOTOR_COUNT
} motor_id_t;

void motor_init(void);
void motor_set_duty(motor_id_t motor, int16_t permille);
void motor_stop_all(void);

#endif
