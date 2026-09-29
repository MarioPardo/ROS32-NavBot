#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#include "cmsis_os.h"

/* AD0 tied low on the breakout, so the 7-bit address is 0x68. */
#define MPU6050_ADDR_7BIT 0x68U
#define MPU6050_ADDR_HAL  (MPU6050_ADDR_7BIT << 1)

typedef enum
{
  IMU_X = 0,   /* forward */
  IMU_Y,       /* left */
  IMU_Z,       /* up */
  IMU_AXIS_COUNT
} imu_axis_t;

typedef struct
{
  int16_t  accel_raw[IMU_AXIS_COUNT];
  int16_t  gyro_raw[IMU_AXIS_COUNT];
  int16_t  temp_raw;
  int32_t  accel_mg[IMU_AXIS_COUNT];    /* milli-g, +-2 g full scale */
  int32_t  gyro_mdps[IMU_AXIS_COUNT];   /* milli-deg/s, +-250 dps full scale */
  int32_t  temp_mc;                     /* milli-degrees C */
  uint32_t seq;                         /* bumped once per good read */
  uint32_t tick_ms;                     /* tick at which this sample was read */
} imu_sample_t;

/* bus_lock guards I2C1; the driver never creates a mutex of its own. */
bool mpu6050_init(osMutexId_t bus_lock);

/* Reads one sample and publishes it. On failure the last good sample stands. */
bool mpu6050_update(void);

/* Consistent copy of the newest sample. Task context only, never an ISR. */
bool mpu6050_get(imu_sample_t *out);

#endif
