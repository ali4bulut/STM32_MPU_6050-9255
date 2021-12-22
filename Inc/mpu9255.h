/*
 * mpu9255.h
 *
 *  Created on: 19 Eki 2021
 *      Author: alibl
 */

#ifndef INC_MPU9255_H_
#define INC_MPU9255_H_

#include "stm32f4xx_hal.h"
#include "quaternionFilter.h"

#define ACC_RES_16G		(16.0f/32768.0f)
#define ACC_RES_8G		(8.0f/32768.0f)
#define ACC_RES_4G		(4.0f/32768.0f)
#define ACC_RES_2G		(2.0f/32768.0f)

#define GYR_RES_250DPS 	(250.0f/32768.0f)
#define GYR_RES_500DPS 	(500.0f/32768.0f)
#define GYR_RES_1000DPS (1000.0f/32768.0f)
#define GYR_RES_2000DPS (2000.0f/32768.0f)

#define MAG_RES_14B		((10.0f*4912.0f)/8190.0f)
#define MAG_RES_16B		((10.0f*4912.0f)/32760.0f)

#define NORMAL_MODE		0
#define INT_MODE		1
#define FIFO_MODE		2

#define MPU_9255_ID		0x71
#define MPU_9255_ADDR	0xD0	//0x68

#define AK8963_ID		0x48
#define AK8963_ADDR		0x18	//0x0C

//MPU 9255 Registers
#define USER_CTRL		0x6A
#define PWR_MGMT_1		0x6B
#define PWR_MGMT_2		0x6C

#define INT_PIN_CFG		0x37
#define INT_ENABLE		0x38
#define INT_STATUS		0x3A

#define SMPLRT_DIV		0x19
#define CONFIG			0x1A
#define GYRO_CONFIG		0x1B
#define ACCEL_CONFIG	0x1C
#define ACCEL_CONFIG_2	0x1D

#define FIFO_EN			0x23
#define I2C_MST_CTRL	0x24

#define ACCEL_XOUT_H	0x3B
#define TEMP_OUT_H		0x41
#define GYRO_XOUT_H		0x43

#define XG_OFFSET_H		0x13
#define XA_OFFSET_H		0x77

#define FIFO_COUNTH		0x72
#define FIFO_R_W		0x74

#define WHO_AM_I		0x75

//AK8963 Registers
#define AK8963_WMI		0x00

#define AK8963_CNTL1	0x0A

#define AK8963_ASAX		0x10
#define AK8963_ASAY		0x11
#define AK8963_ASAZ		0x12

#define AK8963_ST1		0x02
#define AK8963_ST2		0x09

#define AK8963_HXL		0x03

#define DEG_2_RAD  M_PI / 180.0f

typedef struct {

	I2C_HandleTypeDef *i2c;

	volatile int16_t Accel_Raw_Data[3];
	volatile float Accel_Data[3];

	volatile int16_t Gyro_Raw_Data[3];
	volatile float Gyro_Data[3];

	volatile int16_t Mag_Raw_Data[3];
	volatile float Mag_Data[3];

	float Mag_ASA[3];

	volatile float Temperature;

	float a_bias[3];
	float g_bias[3];
	float m_bias[3];

	float a_Mat[3];
	float b_Mat[3];

	uint8_t Calibrate_Done;
	uint8_t Mode;

	uint8_t RxBuffer[14];

	uint8_t Mag_Ready;

	uint8_t Mag_RxBuffer[7];
	uint8_t st;

} MPU9255_HandleTypeDef;

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2C, MPU9255_HandleTypeDef *mpu, uint8_t mode);

void MPU9255_Read_Raw(MPU9255_HandleTypeDef *mpu);
void MPU9255_Read_Data(MPU9255_HandleTypeDef *mpu);
void MPU9255_Read_Mag(MPU9255_HandleTypeDef *mpu);
void MPU9255_Read_FIFO_ALL(MPU9255_HandleTypeDef *mpu);
void Calibrate_Acc_Gyro(MPU9255_HandleTypeDef *mpu, uint16_t sample);
void Calibrate_Mag(MPU9255_HandleTypeDef *mpu);

void Update_Filter_Data(MPU9255_HandleTypeDef *mpu);

void MPU9255_Read_All_DMA(MPU9255_HandleTypeDef *mpu);
void MPU9255_DMA_Callback(MPU9255_HandleTypeDef *mpu);

#endif /* INC_MPU9255_H_ */
