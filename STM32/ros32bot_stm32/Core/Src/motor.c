#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t           channel;
  GPIO_TypeDef      *dir_port;
  uint16_t           dir_a_pin;
  uint16_t           dir_b_pin;
} motor_hw_t;

static const motor_hw_t motors[MOTOR_COUNT] =
{
  [MOTOR_LEFT]  = { &htim4, TIM_CHANNEL_1, LMOT_DIR_A_GPIO_Port, LMOT_DIR_A_Pin, LMOT_DIR_B_Pin },
  [MOTOR_RIGHT] = { &htim2, TIM_CHANNEL_1, RMOT_DIR_A_GPIO_Port, RMOT_DIR_A_Pin, RMOT_DIR_B_Pin },
};

static void motor_write_direction(const motor_hw_t *motor, int16_t permille)
{
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_a_pin, (permille > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_b_pin, (permille < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t motor_duty_to_ccr(const motor_hw_t *motor, int16_t permille)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(motor->htim) + 1U;
  return ((uint32_t)permille * period) / MOTOR_DUTY_MAX;
}

void motor_init(void)
{
  motor_stop_all();

  for (motor_id_t i = 0; i < MOTOR_COUNT; i++)
  {
    HAL_TIM_PWM_Start(motors[i].htim, motors[i].channel);
  }
}

void motor_set_duty(motor_id_t motor, int16_t duty)
{
  if ((unsigned)motor >= (unsigned)MOTOR_COUNT)
  {
    return;
  }

  if (duty > MOTOR_DUTY_MAX)
  {
    duty = MOTOR_DUTY_MAX;
  }
  else if (duty < -MOTOR_DUTY_MAX)
  {
    duty = -MOTOR_DUTY_MAX;
  }

  const motor_hw_t *hw = &motors[motor];

  __HAL_TIM_SET_COMPARE(hw->htim, hw->channel, 0U);
  motor_write_direction(hw, duty);
  __HAL_TIM_SET_COMPARE(hw->htim, hw->channel, motor_duty_to_ccr(hw, (duty < 0) ? -duty : duty));
}

void motor_stop_all(void)
{
  for (motor_id_t i = 0; i < MOTOR_COUNT; i++)
  {
    __HAL_TIM_SET_COMPARE(motors[i].htim, motors[i].channel, 0U);
    HAL_GPIO_WritePin(motors[i].dir_port, motors[i].dir_a_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motors[i].dir_port, motors[i].dir_b_pin, GPIO_PIN_RESET);
  }
}
