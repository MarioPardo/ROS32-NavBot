#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "wheel_control.h"

#include "encoder.h"
#include "motor.h"
#include "SEGGER_RTT.h"


//Converting Arduino PID Constants -> STM32  since arduino PWM max = 255, stm32 pwm max = 1000
#define ARDUINO_PWM_OUTPUT_MAX 255.0f
#define PID_CONVERSION_FACTOR ((float)MOTOR_DUTY_MAX / ARDUINO_PWM_OUTPUT_MAX)

// Manual Testing:  7 rad/s needed about 505 permille = 50.5% of duty
#define WHEEL_FF_PERMILLE_PER_RAD_S 70.0f

//TESTING PID: to disable set to 0
#define WHEEL_TEST_TARGET_RAD_S 0.0f

//TESTING PWM Direct
#define WHEEL_FIXED_DUTY_TEST     0
#define WHEEL_FIXED_DUTY_PERMILLE 700 

#define RAD_PER_COUNT (6.28318531f / (float)ENCODER_COUNTS_PER_REV)

///////////////////////


typedef struct
{
  encoder_id_t     encoder;
  float            kp;
  float            ki;
  float            kd;
  float            meas_sign;  // -1 backwards, +1 pos
  volatile float   target;     // rad/s, written by the UART task */
  float            integral;   //* rad, the error accumulated over time 
  float            last_meas;  ///rad/s */
  volatile int16_t duty;       // permille, used for logging
} wheel_t;

static wheel_t wheels[MOTOR_COUNT] =
{
  [MOTOR_LEFT]  = { .encoder = ENCODER_LEFT,  .kp = 12.8f, .ki = 8.3f, .kd = 0.1f, .meas_sign = 1.0f },
  [MOTOR_RIGHT] = { .encoder = ENCODER_RIGHT, .kp = 11.5f, .ki = 7.5f, .kd = 0.1f, .meas_sign = -1.0f },
};


////////////////

/* Live from boot, so wheel frames work without an arming step. */
static bool enabled = true;


static float wheel_get_meas_rad_s(motor_id_t motor)
{
  const wheel_t *wheel = &wheels[motor];
  return (float)encoder_get_velocity(wheel->encoder) * RAD_PER_COUNT * wheel->meas_sign;
}


void wheel_control_init(void)
{
  for (motor_id_t i = 0; i < MOTOR_COUNT; i++)
  {
    wheels[i].target = 0.0f;
    wheels[i].integral = 0.0f;
    wheels[i].last_meas = 0.0f;
    wheels[i].duty = 0;
  }

  motor_stop_all();
}

void wheel_control_set_target(motor_id_t motor, float rad_s)
{
  if ((unsigned)motor >= (unsigned)MOTOR_COUNT)
    return;

  wheels[motor].target = rad_s;
}

void wheel_control_set_enabled(uint8_t enable)
{
  enabled = enable;
}

void wheel_control_get_state(motor_id_t motor, wheel_state_t *out)
{
  if (((unsigned)motor >= (unsigned)MOTOR_COUNT) || (out == NULL))
    return;
  

  const wheel_t *wheel = &wheels[motor];

  out->target = wheel->target;
  out->meas = wheel_get_meas_rad_s(motor);
  out->duty = wheel->duty;
}


static float pid_step(wheel_t *wheel, float meas, float dt)
{
  const float error = wheel->target - meas;

  wheel->integral += error * dt;

  const float derivative = (meas - wheel->last_meas) / dt;
  wheel->last_meas = meas;

  float out = (wheel->kp * error) 
            + (wheel->ki * wheel->integral)
            - (wheel->kd * derivative);

  if (out > MOTOR_DUTY_MAX)
  {
    out = MOTOR_DUTY_MAX;
    wheel->integral -= error * dt;  /* undo the wind-up */
  }
  else if (out < -MOTOR_DUTY_MAX)
  {
    out = -MOTOR_DUTY_MAX;
    wheel->integral -= error * dt;
  }

  return out;
}

void wheel_control_update(uint32_t dt_ms)
{
  if (dt_ms == 0U)
  {
    return;
  }

#if WHEEL_FIXED_DUTY_TEST
  /* TEMPORARY TEST: drive every motor at a constant 70 % duty forever, ignoring
     PID, encoder feedback, UART commands and wheel_control_set_enabled(). */
  for (motor_id_t i = 0; i < MOTOR_COUNT; i++)
  {
    wheels[i].duty = WHEEL_FIXED_DUTY_PERMILLE;
    motor_set_duty(i, WHEEL_FIXED_DUTY_PERMILLE);
  }
  return;
#else

#if WHEEL_OPEN_LOOP_TEST
  open_loop_step(dt_ms);
  return;
#endif

  const float dt = (float)dt_ms / 1000.0f;  /* seconds */

  //IF TESTING ENABLED
  if (WHEEL_TEST_TARGET_RAD_S != 0.0f)
  {
    wheel_control_set_target(MOTOR_LEFT,  WHEEL_TEST_TARGET_RAD_S);
    wheel_control_set_target(MOTOR_RIGHT, WHEEL_TEST_TARGET_RAD_S);
  }

  for (motor_id_t i = 0; i < MOTOR_COUNT; i++)
  {
    wheel_t *wheel = &wheels[i];

    const float meas = wheel_get_meas_rad_s(i);

    //disabled/stopped
    if (!enabled || (wheel->target == 0.0f))
    {
      wheel->integral = 0.0f;
      wheel->last_meas = meas;
      wheel->duty = 0;
      motor_set_duty(i, 0);
      continue;
    }

    float duty = (WHEEL_FF_PERMILLE_PER_RAD_S * wheel->target)+ (pid_step(wheel, meas, dt) * PID_CONVERSION_FACTOR);

    wheel->duty = (int16_t)duty;
    motor_set_duty(i, (int16_t)duty);
  }
#endif /* WHEEL_FIXED_DUTY_TEST */
}



//////////////////////

/* Bench test: set to 1 to re-measure the feedforward with a duty ramp. */
#define WHEEL_OPEN_LOOP_TEST 0

#if WHEEL_OPEN_LOOP_TEST
#define OPEN_LOOP_STEP_PERMILLE 50
#define OPEN_LOOP_STEP_MS       600U
#define OPEN_LOOP_MAX_PERMILLE  600

/* Steps the duty up every OPEN_LOOP_STEP_MS, then wraps, for fresh readings. */
static void open_loop_step(uint32_t dt_ms)
{
  static uint32_t elapsed_ms;
  static int16_t  duty;

  elapsed_ms += dt_ms;

  if (elapsed_ms >= OPEN_LOOP_STEP_MS)
  {
    elapsed_ms = 0U;
    duty = (int16_t)((duty >= OPEN_LOOP_MAX_PERMILLE) ? 0 : (duty + OPEN_LOOP_STEP_PERMILLE));
  }

  motor_set_duty(MOTOR_LEFT, duty);
  motor_set_duty(MOTOR_RIGHT, duty);

  SEGGER_RTT_printf(0, "open loop  duty=%d permille  L=%d crad/s  R=%d crad/s\r\n",
                    (int)duty,
                    (int)(wheel_get_meas_rad_s(MOTOR_LEFT) * 100.0f),
                    (int)(wheel_get_meas_rad_s(MOTOR_RIGHT) * 100.0f));
}
#endif
