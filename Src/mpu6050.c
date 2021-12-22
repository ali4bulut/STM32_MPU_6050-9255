/*
 * mpu6050.c
 *
 *  Created on: Sep 2, 2021
 *      Author: alibl
 */
#include "mpu6050.h"

void Kalman_angleSolve(MPU6050_HandleTypeDef *mpu);
void Calibrate_ALL_MPU(MPU6050_HandleTypeDef *mpu, float count);
void Calib_ACCEL(MPU6050_HandleTypeDef *mpu, float count);
void Calib_GYRO(MPU6050_HandleTypeDef *mpu, float count);

float inverse_rsqrt(float number);

static Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

static Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f, };
static Kalman_t KalmanZ = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f, };

uint8_t MPU_Init(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu, uint8_t calib_state) {

	uint8_t check;
	uint8_t data;

	if (HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, WHO_I_AM, 1, &check, 1, 10000) == HAL_OK) {
		if (check == 104) {  		//104 for MPU6050 113 and 115 for MPU9250

			//CLKSEL ile saat seçildi
			data = 0x00; //0x04
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
			//1Khz e ayarlandı
			data = 0x07;	//0x07
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
			//+-2g olarak seçildi
			data = 0;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);
			//+-250 deg/s olarak seçildi
			data = 0;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
			//INT pininin ayarları yapıldı
			data = 0x00;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, INT_CONFG, 1, &data, 1, 100);
			//FIFO için DATA_RDY_EN aktif edildi
			data = 0x01;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, INT_ENABLE, 1, &data, 1, 100);
			//USR_CTRL reg ile FIFO enable edildi ve resetlendi
			data = 0x44;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, USER_CTRL, 1, &data, 1, 100);
			//FIFO ya nelerin aktarılacağı seçildi sıcaklık gyro ve accel verileri reg numara sırasına göre aktarılır
			data = 0xF8;
			HAL_I2C_Mem_Write(I2C, MPU6050_ADDR, FIFO_EN, 1, &data, 1, 100);

			mpu->Calib_State = calib_state;

			mpu->bias_Gx = 0;
			mpu->bias_Gy = 0;
			mpu->bias_Gz = 0;

			mpu->bias_Ax = 0;
			mpu->bias_Ay = 0;
			mpu->bias_Az = 0;

			return 1;

		}
	}

	return 0;
}

void MPU6050_Read_Accel_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t DATA_6BYTE[6];

	HAL_I2C_Mem_Read_IT(I2C, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, DATA_6BYTE, 6);

	mpu->Accel_X_RAW = (uint16_t) ((DATA_6BYTE[0] << 8) | DATA_6BYTE[1]);
	mpu->Accel_Y_RAW = (uint16_t) ((DATA_6BYTE[2] << 8) | DATA_6BYTE[3]);
	mpu->Accel_Z_RAW = (uint16_t) ((DATA_6BYTE[4] << 8) | DATA_6BYTE[5]);

	Calib_ACCEL(mpu, 10000);
}

void MPU6050_Read_Gyro_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t DATA_6BYTE[6];

	HAL_I2C_Mem_Read_IT(I2C, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, DATA_6BYTE, 6);

	mpu->Gyro_X_RAW = (uint16_t) ((DATA_6BYTE[0] << 8) | DATA_6BYTE[1]);
	mpu->Gyro_Y_RAW = (uint16_t) ((DATA_6BYTE[2] << 8) | DATA_6BYTE[3]);
	mpu->Gyro_Z_RAW = (uint16_t) ((DATA_6BYTE[4] << 8) | DATA_6BYTE[5]);

	Calib_GYRO(mpu, 10000);

}

void MPU6050_Read_Temp_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t Rec_Data[2];
	static int16_t temp;

	HAL_I2C_Mem_Read_IT(I2C, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2);

	temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	mpu->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t DATA_6BYTE[6];

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, DATA_6BYTE, 6, I2C_TimeOut);

	mpu->Accel_X_RAW = (uint16_t) ((DATA_6BYTE[0] << 8) | DATA_6BYTE[1]);
	mpu->Accel_Y_RAW = (uint16_t) ((DATA_6BYTE[2] << 8) | DATA_6BYTE[3]);
	mpu->Accel_Z_RAW = (uint16_t) ((DATA_6BYTE[4] << 8) | DATA_6BYTE[5]);

	Calib_ACCEL(mpu, 10000);

}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t DATA_6BYTE[6];

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, DATA_6BYTE, 6, I2C_TimeOut);

	mpu->Gyro_X_RAW = (uint16_t) ((DATA_6BYTE[0] << 8) | DATA_6BYTE[1]);
	mpu->Gyro_Y_RAW = (uint16_t) ((DATA_6BYTE[2] << 8) | DATA_6BYTE[3]);
	mpu->Gyro_Z_RAW = (uint16_t) ((DATA_6BYTE[4] << 8) | DATA_6BYTE[5]);

	Calib_GYRO(mpu, 10000);

}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t Rec_Data[2];
	static int16_t temp;

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, I2C_TimeOut);

	temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	mpu->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

}

void MPU6050_Read_All(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t Data[14];
	static uint16_t temp;

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Data, 14, I2C_TimeOut);

	mpu->Accel_X_RAW = (uint16_t) ((Data[0] << 8) | Data[1]);
	mpu->Accel_Y_RAW = (uint16_t) ((Data[2] << 8) | Data[3]);
	mpu->Accel_Z_RAW = (uint16_t) ((Data[4] << 8) | Data[5]);

	temp = (int16_t) (Data[6] << 8 | Data[7]);

	mpu->Gyro_X_RAW = (uint16_t) ((Data[8] << 8) | Data[9]);
	mpu->Gyro_Y_RAW = (uint16_t) ((Data[10] << 8) | Data[11]);
	mpu->Gyro_Z_RAW = (uint16_t) ((Data[12] << 8) | Data[13]);

	mpu->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

	Calibrate_ALL_MPU(mpu, 10000);

}

void MPU6050_Read_All_IT(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t Data[14];
	static uint16_t temp;

	HAL_I2C_Mem_Read_IT(I2C, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Data, 14);

	mpu->Accel_X_RAW = (uint16_t) ((Data[0] << 8) | Data[1]);
	mpu->Accel_Y_RAW = (uint16_t) ((Data[2] << 8) | Data[3]);
	mpu->Accel_Z_RAW = (uint16_t) ((Data[4] << 8) | Data[5]);

	temp = (int16_t) (Data[6] << 8 | Data[7]);

	mpu->Gyro_X_RAW = (uint16_t) ((Data[8] << 8) | Data[9]);
	mpu->Gyro_Y_RAW = (uint16_t) ((Data[10] << 8) | Data[11]);
	mpu->Gyro_Z_RAW = (uint16_t) ((Data[12] << 8) | Data[13]);

	mpu->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

	Calibrate_ALL_MPU(mpu, 10000);

}

//FIFO kullanmak için GPIO portlarından birini interrupt da kulanman ve FIFO okumasını
//Interruptın içinde yapman gerek.
//Init kısmındada fifo ve interuptları açman gerek.

void MPU6050_FIFO_Read_ALL(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu) {

	static uint8_t FifoLenght[2];
	static uint8_t Data[14];
	static uint16_t lenght;
	static uint16_t temp;

	HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, FIFO_COUNT, 1, FifoLenght, 2, I2C_TimeOut);
	lenght = (uint16_t) ((FifoLenght[0] << 8) | FifoLenght[1]);

	if (lenght >= 14) {

		HAL_I2C_Mem_Read(I2C, MPU6050_ADDR, FIFO_RW, 1, Data, 14, I2C_TimeOut);

		mpu->Accel_X_RAW = (uint16_t) ((Data[0] << 8) | Data[1]);
		mpu->Accel_Y_RAW = (uint16_t) ((Data[2] << 8) | Data[3]);
		mpu->Accel_Z_RAW = (uint16_t) ((Data[4] << 8) | Data[5]);

		temp = (int16_t) (Data[6] << 8 | Data[7]);
		mpu->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);

		mpu->Gyro_X_RAW = (uint16_t) ((Data[8] << 8) | Data[9]);
		mpu->Gyro_Y_RAW = (uint16_t) ((Data[10] << 8) | Data[11]);
		mpu->Gyro_Z_RAW = (uint16_t) ((Data[12] << 8) | Data[13]);

		Calibrate_ALL_MPU(mpu, 10000);

	}
}

float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
	float rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	float S = Kalman->P[0][0] + Kalman->R_measure;
	float K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	float y = newAngle - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	float P00_temp = Kalman->P[0][0];
	float P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}

void Kalman_angleSolve(MPU6050_HandleTypeDef *mpu) {
	static uint32_t timer = 0;

	float dt = (float) (HAL_GetTick() - timer) / 1000;
	timer = HAL_GetTick();
	float roll;
	float roll_sqrt = inverse_rsqrt(mpu->Accel_X_RAW_Calib * mpu->Accel_X_RAW_Calib + mpu->Accel_Z_RAW_Calib * mpu->Accel_Z_RAW_Calib);
	if (roll_sqrt != 0.0) {
		roll = atanf(mpu->Accel_Y_RAW_Calib / roll_sqrt) * RAD_TO_DEG;
	}
	else {
		roll = 0.0;
	}
	float pitch = atan2f(-mpu->Accel_X_RAW_Calib, mpu->Accel_Z_RAW_Calib) * RAD_TO_DEG;
	if ((pitch < -90 && mpu->KalmanAngleY > 90) || (pitch > 90 && mpu->KalmanAngleY < -90)) {
		KalmanY.angle = pitch;
		mpu->KalmanAngleY = pitch;
	}
	else {
		mpu->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, mpu->Gy, dt);
	}
	if (fabs(mpu->KalmanAngleY) > 90)
		mpu->Gx = -mpu->Gx;
	mpu->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, mpu->Gx, dt);

	mpu->KalmanAngleZ = Kalman_getAngle(&KalmanZ, mpu->KalmanAngleZ, mpu->Gz, dt);

}

void Calibrate_ALL_MPU(MPU6050_HandleTypeDef *mpu, float count) {
	static uint32_t cnt = 0;

	if (mpu->Calib_State) {
		if (cnt <= count) {

			mpu->Ax = mpu->Accel_X_RAW / 16384.0;
			mpu->Ay = mpu->Accel_Y_RAW / 16384.0;
			mpu->Az = mpu->Accel_Z_RAW / 16384.0;

			mpu->Gx = mpu->Gyro_X_RAW / 131.0;
			mpu->Gy = mpu->Gyro_Y_RAW / 131.0;
			mpu->Gz = mpu->Gyro_Z_RAW / 131.0;

			mpu->bias_Gx += (mpu->Gyro_X_RAW / count);
			mpu->bias_Gy += (mpu->Gyro_Y_RAW / count);
			mpu->bias_Gz += (mpu->Gyro_Z_RAW / count);

			mpu->bias_Ax += (mpu->Accel_X_RAW / count);
			mpu->bias_Ay += (mpu->Accel_Y_RAW / count);
			mpu->bias_Az += (mpu->Accel_Z_RAW / count);

			cnt++;
		}
		else {

			if (16384.0 - mpu->bias_Az != 0) {
				mpu->bias_Az -= 16384.0;
			}
			else {
				mpu->bias_Az = 0;
			}
			cnt = 0;
			mpu->Calib_State = 0;

		}
	}
	else {

		mpu->Accel_X_RAW_Calib = (mpu->Accel_X_RAW - mpu->bias_Ax);
		mpu->Accel_Y_RAW_Calib = (mpu->Accel_Y_RAW - mpu->bias_Ay);
		mpu->Accel_Z_RAW_Calib = (mpu->Accel_Z_RAW - mpu->bias_Az);

		mpu->Gyro_X_RAW_Calib = (mpu->Gyro_X_RAW - mpu->bias_Gx);
		mpu->Gyro_Y_RAW_Calib = (mpu->Gyro_Y_RAW - mpu->bias_Gy);
		mpu->Gyro_Z_RAW_Calib = (mpu->Gyro_Z_RAW - mpu->bias_Gz);

		mpu->Ax = mpu->Accel_X_RAW_Calib / 16384.0;
		mpu->Ay = mpu->Accel_Y_RAW_Calib / 16384.0;
		mpu->Az = mpu->Accel_Z_RAW_Calib / 16384.0;

		mpu->Gx = mpu->Gyro_X_RAW_Calib / 131.0;
		mpu->Gy = mpu->Gyro_Y_RAW_Calib / 131.0;
		mpu->Gz = mpu->Gyro_Z_RAW_Calib / 131.0;

		// Kalman angle solve
		Kalman_angleSolve(mpu);
	}
}

void Calib_GYRO(MPU6050_HandleTypeDef *mpu, float count) {

	static uint32_t cnt = 0;

	if (mpu->Calib_State) {
		if (cnt <= count) {

			mpu->Gx = mpu->Gyro_X_RAW / 131.0;
			mpu->Gy = mpu->Gyro_Y_RAW / 131.0;
			mpu->Gz = mpu->Gyro_Z_RAW / 131.0;

			mpu->bias_Gx += (mpu->Gyro_X_RAW / count);
			mpu->bias_Gy += (mpu->Gyro_Y_RAW / count);
			mpu->bias_Gz += (mpu->Gyro_Z_RAW / count);

			cnt++;
		}
		else {
			cnt = 0;
			mpu->Calib_State = 0;

		}
	}
	else {

		mpu->Gyro_X_RAW_Calib = (mpu->Gyro_X_RAW - mpu->bias_Gx);
		mpu->Gyro_Y_RAW_Calib = (mpu->Gyro_Y_RAW - mpu->bias_Gy);
		mpu->Gyro_Z_RAW_Calib = (mpu->Gyro_Z_RAW - mpu->bias_Gz);

		mpu->Gx = mpu->Gyro_X_RAW_Calib / 131.0;
		mpu->Gy = mpu->Gyro_Y_RAW_Calib / 131.0;
		mpu->Gz = mpu->Gyro_Z_RAW_Calib / 131.0;

	}

}

void Calib_ACCEL(MPU6050_HandleTypeDef *mpu, float count) {

	static uint32_t cnt = 0;

	if (mpu->Calib_State) {
		if (cnt <= count) {

			mpu->Ax = mpu->Accel_X_RAW / 16384.0;
			mpu->Ay = mpu->Accel_Y_RAW / 16384.0;
			mpu->Az = mpu->Accel_Z_RAW / 16384.0;

			mpu->bias_Ax += (mpu->Accel_X_RAW / count);
			mpu->bias_Ay += (mpu->Accel_Y_RAW / count);
			mpu->bias_Az += (mpu->Accel_Z_RAW / count);

			cnt++;
		}
		else {

			if (16384.0 - mpu->bias_Az != 0) {
				mpu->bias_Az -= 16384.0;
			}
			else {
				mpu->bias_Az = 0;
			}
			cnt = 0;
			mpu->Calib_State = 0;

		}
	}
	else {

		mpu->Accel_X_RAW_Calib = (mpu->Accel_X_RAW - mpu->bias_Ax);
		mpu->Accel_Y_RAW_Calib = (mpu->Accel_Y_RAW - mpu->bias_Ay);
		mpu->Accel_Z_RAW_Calib = (mpu->Accel_Z_RAW - mpu->bias_Az);

		mpu->Ax = mpu->Accel_X_RAW_Calib / 16384.0;
		mpu->Ay = mpu->Accel_Y_RAW_Calib / 16384.0;
		mpu->Az = mpu->Accel_Z_RAW_Calib / 16384.0;

	}
}



float inverse_rsqrt(float number) {
	const float threehalfs = 1.5F;

	float x2 = number * 0.5F;
	float y = number;

	// evil floating point bit level hacking
	long i = *(long*) &y;

	// value is pre-assumed
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;

	// 1st iteration
	y = y * (threehalfs - (x2 * y * y));

	// 2nd iteration, this can be removed
	// y = y * ( threehalfs - ( x2 * y * y ) );

	return 1 / y;
}

