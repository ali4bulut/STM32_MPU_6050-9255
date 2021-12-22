/*
 * mpu6050.h
 *
 *  Created on: Sep 2, 2021
 *      Author: alibl
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <math.h>

//#ifndef __STM32F4xx_HAL_I2C_H
//	#include "stm32f4xx_hal_i2c.h"
//#endif

#define MPU6050_ADDR			0xD0

#define WHO_I_AM				0x75
#define PWR_MGMT_1_REG			0x6B
#define SMPLRT_DIV_REG 			0x19
#define ACCEL_CONFIG_REG 		0x1C
#define ACCEL_XOUT_H_REG 		0x3B
#define TEMP_OUT_H_REG 			0x41
#define GYRO_CONFIG_REG 		0x1B
#define GYRO_XOUT_H_REG 		0x43
#define USER_CTRL				0x6A
#define INT_STAUTS				0x3A
#define INT_ENABLE				0x38
#define INT_CONFG				0x37
#define FIFO_EN					0x23

#define FIFO_RW					0x74
#define FIFO_COUNT				0x72

#define I2C_TimeOut				100

#define CALIB_ENABLE			1
#define CALIB_DISABLE			0

#define RAD_TO_DEG				(180 / 3.14159265359f)

typedef struct {
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float P[2][2];
} Kalman_t;

typedef struct {
	volatile int16_t Accel_X_RAW;
	volatile int16_t Accel_Y_RAW;
	volatile int16_t Accel_Z_RAW;
	volatile float Ax;
	volatile float Ay;
	volatile float Az;

	volatile int16_t Gyro_X_RAW;
	volatile int16_t Gyro_Y_RAW;
	volatile int16_t Gyro_Z_RAW;
	volatile float Gx;
	volatile float Gy;
	volatile float Gz;

	float bias_Gx;
	float bias_Gy;
	float bias_Gz;

	float bias_Ax;
	float bias_Ay;
	float bias_Az;

	uint8_t Calib_State;

	volatile int16_t Accel_X_RAW_Calib;
	volatile int16_t Accel_Y_RAW_Calib;
	volatile int16_t Accel_Z_RAW_Calib;

	volatile int16_t Gyro_X_RAW_Calib;
	volatile int16_t Gyro_Y_RAW_Calib;
	volatile int16_t Gyro_Z_RAW_Calib;

	volatile float Temperature;

	volatile float KalmanAngleX;
	volatile float KalmanAngleY;
	volatile float KalmanAngleZ;

} MPU6050_HandleTypeDef;

uint8_t MPU_Init(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu, uint8_t calib_state);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_All(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_Accel_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_Gyro_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_Temp_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_Read_All_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

void MPU6050_FIFO_Read_ALL(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu);

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, MPU6050_HandleTypeDef *mpu);

#endif /* INC_MPU6050_H_ */
