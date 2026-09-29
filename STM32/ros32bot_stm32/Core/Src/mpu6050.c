#include <stddef.h>

#include "mpu6050.h"
#include "main.h"
#include "SEGGER_RTT.h"

/* Only the registers this driver touches. */
#define REG_SMPLRT_DIV   0x19U
#define REG_CONFIG       0x1AU
#define REG_GYRO_CONFIG  0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_PWR_MGMT_1   0x6BU
#define REG_WHO_AM_I     0x75U

#define WHO_AM_I_VALUE   0x68U

/* DLPF 3: 44 Hz accel / 42 Hz gyro bandwidth, 1 kHz internal rate. */
#define DLPF_CFG_44HZ    0x03U
/* 1 kHz / (1 + 4) = 200 Hz output, matching the gyroTask period. */
#define SMPLRT_DIV_200HZ 0x04U

/* ACCEL_XOUT_H .. GYRO_ZOUT_L = accel(6) + temp(2) + gyro(6). */
#define BURST_LEN        14U
#define TIMEOUT_MS       100U

extern I2C_HandleTypeDef hi2c1;

static osMutexId_t  bus_lock;
static imu_sample_t sample;

/* Callers must already hold bus_lock; these do no locking themselves. */
static bool reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_HAL, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, TIMEOUT_MS) == HAL_OK;
}

static bool reg_write(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_HAL, reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1U, TIMEOUT_MS) == HAL_OK;
}

/* The MPU6050 puts the high byte on the wire first. */
static int16_t to_i16(uint8_t hi, uint8_t lo)
{
  return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

static int32_t accel_raw_to_mg(int16_t raw)
{
  /* +-2 g full scale is 16384 LSB/g, so raw * 1000 / 16384 = raw * 125 / 2048. */
  return ((int32_t)raw * 125) / 2048;
}

static int32_t gyro_raw_to_mdps(int16_t raw)
{
  /* +-250 dps full scale is 131.072 LSB/dps, so raw * 1000000 / 131072 = * 15625 / 2048. */
  return ((int32_t)raw * 15625) / 2048;
}

static int32_t temp_raw_to_mc(int16_t raw)
{
  /* Datasheet: degC = raw / 340 + 36.53. */
  return ((int32_t)raw * 50) / 17 + 36530;
}

bool mpu6050_init(osMutexId_t lock)
{
  uint8_t who = 0U;
  bool    ok;

  if (lock == NULL)
  {
    return false;
  }

  bus_lock = lock;

  if (osMutexAcquire(bus_lock, osWaitForever) != osOK)
  {
    return false;
  }

  /* Same presence test the imuTest reference uses: a plain address ACK probe. */
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR_HAL, 3U, TIMEOUT_MS) != HAL_OK)
  {
    osMutexRelease(bus_lock);
    SEGGER_RTT_printf(0, "mpu6050: no ACK at 0x%02X\r\n", MPU6050_ADDR_HAL);
    return false;
  }

  (void)reg_read(REG_WHO_AM_I, &who, 1U);

  /* PWR_MGMT_1 = 0 clears SLEEP and picks the internal 8 MHz oscillator. */
  ok = reg_write(REG_GYRO_CONFIG, 0x00U) &&
       reg_write(REG_ACCEL_CONFIG, 0x00U) &&
       reg_write(REG_CONFIG, DLPF_CFG_44HZ) &&
       reg_write(REG_SMPLRT_DIV, SMPLRT_DIV_200HZ) &&
       reg_write(REG_PWR_MGMT_1, 0x00U);

  osMutexRelease(bus_lock);

  /* Reported, not enforced: clones answer with IDs other than 0x68. */
  SEGGER_RTT_printf(0, "mpu6050: who_am_i=0x%02X expect=0x%02X cfg=%s\r\n",
                    who, WHO_AM_I_VALUE, ok ? "ok" : "FAIL");

  return ok;
}

bool mpu6050_update(void)
{
  uint8_t burst[BURST_LEN];
  bool    ok;
  int     i;

  if (bus_lock == NULL)
  {
    return false;
  }

  if (osMutexAcquire(bus_lock, osWaitForever) != osOK)
  {
    return false;
  }

  ok = reg_read(REG_ACCEL_XOUT_H, burst, BURST_LEN);

  if (ok)
  {
    /* Writing straight into sample is safe: the lock holds every reader off. */
    sample.accel_raw[IMU_X] = to_i16(burst[0], burst[1]);
    sample.accel_raw[IMU_Y] = to_i16(burst[2], burst[3]);
    sample.accel_raw[IMU_Z] = to_i16(burst[4], burst[5]);
    sample.temp_raw         = to_i16(burst[6], burst[7]);
    sample.gyro_raw[IMU_X]  = to_i16(burst[8], burst[9]);
    sample.gyro_raw[IMU_Y]  = to_i16(burst[10], burst[11]);
    sample.gyro_raw[IMU_Z]  = to_i16(burst[12], burst[13]);

    for (i = 0; i < IMU_AXIS_COUNT; i++)
    {
      sample.accel_mg[i]  = accel_raw_to_mg(sample.accel_raw[i]);
      sample.gyro_mdps[i] = gyro_raw_to_mdps(sample.gyro_raw[i]);
    }

    sample.temp_mc = temp_raw_to_mc(sample.temp_raw);
    sample.tick_ms = osKernelGetTickCount();
    sample.seq++;
  }

  osMutexRelease(bus_lock);

  return ok;
}

bool mpu6050_get(imu_sample_t *out)
{
  if ((out == NULL) || (bus_lock == NULL))
  {
    return false;
  }

  if (osMutexAcquire(bus_lock, osWaitForever) != osOK)
  {
    return false;
  }

  *out = sample;

  osMutexRelease(bus_lock);

  return true;
}
