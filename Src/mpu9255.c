/*
 * mpu9255.c
 *https://github.com/hideakitai/MPU9250
 *  Created on: 19 Eki 2021
 *      Author: alibl
 */

#include "mpu9255.h"

static void Write_Calib_Data(MPU9255_HandleTypeDef *mpu);

uint8_t MPU9255_Init(I2C_HandleTypeDef *I2C, MPU9255_HandleTypeDef *mpu, uint8_t mode) {

	mpu->i2c = I2C;

	uint8_t device_ID;
	uint8_t data;
	uint8_t reg_Read;

	//Hard Reset
	data = 0x80;
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, PWR_MGMT_1, 1, &data, 1, 10000);
	HAL_Delay(200);

	//Wake UP
	data = 0x00;
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, PWR_MGMT_1, 1, &data, 1, 10000);
	HAL_Delay(200);

	if (HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, WHO_AM_I, 1, &device_ID, 1, 10000) == HAL_OK) {

		if (device_ID == MPU_9255_ID) {

			//Internal 20Mhz clock source
			data = 0x00;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, PWR_MGMT_1, 1, &data, 1, 10000);
			HAL_Delay(200);

			//DLP 8Khz //100Hz //42Hz
			data = 0x03;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, CONFIG, 1, &data, 1, 10000);
			HAL_Delay(15);

			//Sample Rate Divider 4 for 200hz 0 for 1KHZ 9 for 100Hz
			data = 0x04;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, SMPLRT_DIV, 1, &data, 1, 10000);
			HAL_Delay(15);

			//250dps Gyro
			HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, GYRO_CONFIG, 1, &reg_Read, 1, 10000);
			reg_Read &= ~0xE0;                                     // Clear self-test bits [7:5]
			reg_Read &= ~0x03;                                     // Clear Fchoice bits [1:0]
			reg_Read &= ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
			reg_Read |= (uint8_t) ((0x00) << 3);      					// Set full scale range for the gyro
			reg_Read |= (uint8_t) ((~0x03) & 0x03);   					// Set Fchoice for the gyro
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, GYRO_CONFIG, 1, &reg_Read, 1, 10000);
			HAL_Delay(15);

			//2g Accel
			HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, ACCEL_CONFIG, 1, &reg_Read, 1, 10000);
			reg_Read &= ~0xE0;                                     // Clear self-test bits [7:5]
			reg_Read &= ~0x18;                                     // Clear ACCEL_FS_SEL bits [4:3]
			reg_Read |= (uint8_t) ((0x00) << 3);      					// Set full scale range for the accelerometer
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, ACCEL_CONFIG, 1, &reg_Read, 1, 10000);
			HAL_Delay(15);

			HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, ACCEL_CONFIG_2, 1, &reg_Read, 1, 10000);
			reg_Read &= ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
			reg_Read |= (~(0x01 << 3) & 0x08);    				// Set accel_fchoice_b to 1
			reg_Read |= (uint8_t) ((0x03) & 0x07);  				// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, ACCEL_CONFIG_2, 1, &reg_Read, 1, 10000);
			HAL_Delay(15);

//			//Reset I2C
//			data = 0x01;
//			HAL_I2C_Mem_Write(I2C, MPU_9255_ADDR, USER_CTRL, 1, &data, 1, 10000);
//			HAL_Delay(100);
//
//			//I2C 400Khz
//			data = 0xD;
//			HAL_I2C_Mem_Write(I2C, MPU_9255_ADDR, I2C_MST_CTRL, 1, &data, 1, 10000);
//			HAL_Delay(15);
//
			//Disable I2C Master
			data = 0x00;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, USER_CTRL, 1, &data, 1, 10000);
			HAL_Delay(15);

			if (mode & 0x01) {

				//Bypass Enable and INT pin Confg Enable
				data = 0x02;
				HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, INT_PIN_CFG, 1, &data, 1, 10000);
				HAL_Delay(15);
			}
			else {
				//Bypass Enable and INT pin Confg Disable
				data = 0x22;
				HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, INT_PIN_CFG, 1, &data, 1, 10000);
				HAL_Delay(15);
			}

			if (mode & 0x02) {

				//Enable FIFO
				data = 0x44;
				HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, USER_CTRL, 1, &data, 1, 10000);
				HAL_Delay(15);

				//FIFO Data Structer
				data = 0xF8;
				HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, FIFO_EN, 1, &data, 1, 10000);
				HAL_Delay(15);

				mpu->Mode = 1;

			}

			//Enable Int Pin When Data Ready
			data = 0x01;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, INT_ENABLE, 1, &data, 1, 10000);
			HAL_Delay(15);

			//Enable All Sensors
			data = 0x00;
			HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, PWR_MGMT_2, 1, &data, 1, 10000);
			HAL_Delay(15);

			if (HAL_I2C_Mem_Read(mpu->i2c, AK8963_ADDR, AK8963_WMI, 1, &device_ID, 1, 10000) == HAL_OK) {
				if (device_ID == AK8963_ID) {
					uint8_t recived[3];

					//Power Down mode
					data = 0x00;
					HAL_I2C_Mem_Write(mpu->i2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 10000);
					HAL_Delay(200);

					//Fuse ROM Access mode
					data = 0x1F;
					HAL_I2C_Mem_Write(mpu->i2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 10000);
					HAL_Delay(200);

					HAL_I2C_Mem_Read(mpu->i2c, AK8963_ADDR, AK8963_ASAX, 1, recived, 3, 10000);

					for (int i = 0; i < 3; i++) {
						mpu->Mag_ASA[i] = (float) ((recived[i] - 128.0f) / 256.0f) + 1.0f;
					}

					HAL_Delay(15);

					//Power Down mode
					data = 0x00;
					HAL_I2C_Mem_Write(mpu->i2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 10000);
					HAL_Delay(200);

					//Continuous measurement mode 2 100Hz 16bit output
					data = 0x16;
					HAL_I2C_Mem_Write(mpu->i2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 10000);
					HAL_Delay(200);

				}
			}
			mpu->Calibrate_Done = 0;

			return 1;

		}
	}

	return 0;

}

void Update_Filter_Data(MPU9255_HandleTypeDef *mpu) {

	//Parameter order is important Order : +g[0], -g[1], -g[2], -a[0], +a[1], +a[2], +m[1], -m[0], +m[2]
	MadgwickAHRSupdate(mpu->Gyro_Data[0], -(mpu->Gyro_Data[1]), -(mpu->Gyro_Data[2]), -mpu->Accel_Data[0], mpu->Accel_Data[1], mpu->Accel_Data[2],
			mpu->Mag_Data[1], -mpu->Mag_Data[0], mpu->Mag_Data[2]);
	update_rpy(q0, q1, q2, q3, mpu->Accel_Data[0], mpu->Accel_Data[1], mpu->Accel_Data[2]);
}

void MPU9255_Read_All_DMA(MPU9255_HandleTypeDef *mpu) {

	if (mpu->st == 0) {
		HAL_I2C_Mem_Read_DMA(mpu->i2c, MPU_9255_ADDR, ACCEL_XOUT_H, 1, mpu->RxBuffer, 14);
		mpu->st = 1;
	}
}

void MPU9255_DMA_Callback(MPU9255_HandleTypeDef *mpu) {

	if (mpu->st == 1) {
		uint16_t temp = 0;
		mpu->Accel_Raw_Data[0] = (int16_t) ((mpu->RxBuffer[0] << 8) | mpu->RxBuffer[1]);
		mpu->Accel_Raw_Data[1] = (int16_t) ((mpu->RxBuffer[2] << 8) | mpu->RxBuffer[3]);
		mpu->Accel_Raw_Data[2] = (int16_t) ((mpu->RxBuffer[4] << 8) | mpu->RxBuffer[5]);

		temp = (int16_t) (mpu->RxBuffer[6] << 8 | mpu->RxBuffer[7]);

		mpu->Gyro_Raw_Data[0] = (int16_t) ((mpu->RxBuffer[8] << 8) | mpu->RxBuffer[9]);
		mpu->Gyro_Raw_Data[1] = (int16_t) ((mpu->RxBuffer[10] << 8) | mpu->RxBuffer[11]);
		mpu->Gyro_Raw_Data[2] = (int16_t) ((mpu->RxBuffer[12] << 8) | mpu->RxBuffer[13]);

		mpu->Temperature = (float) ((int16_t) temp / (float) 333.87f + (float) 21.00f);

		for (int i = 0; i < 3; i++) {
			mpu->Accel_Data[i] = (float) mpu->Accel_Raw_Data[i] * ACC_RES_2G * 10.0f; //unit G 1G = 10mG
			mpu->Gyro_Data[i] = (float) mpu->Gyro_Raw_Data[i] * GYR_RES_250DPS * DEG_2_RAD; //unit dps degree per second rad/s
		}

		HAL_I2C_Mem_Read_DMA(mpu->i2c, AK8963_ADDR, AK8963_ST1, 1, &mpu->Mag_Ready, 1);
		mpu->st = 2;
	}
	else if (mpu->st == 2) {
		if ((mpu->Mag_Ready & 0x01)) {
			HAL_I2C_Mem_Read_DMA(mpu->i2c, AK8963_ADDR, AK8963_HXL, 1, mpu->Mag_RxBuffer, 7);
		}
		else {
			HAL_I2C_Mem_Read_DMA(mpu->i2c, AK8963_ADDR, AK8963_ST1, 1, &mpu->Mag_Ready, 1);
		}
		mpu->st = 3;
	}
	else {
		if ((mpu->Mag_Ready & 0x01)) {
			mpu->Mag_Raw_Data[0] = (uint16_t) ((mpu->Mag_RxBuffer[1] << 8) | mpu->Mag_RxBuffer[0]);
			mpu->Mag_Raw_Data[1] = (uint16_t) ((mpu->Mag_RxBuffer[3] << 8) | mpu->Mag_RxBuffer[2]);
			mpu->Mag_Raw_Data[2] = (uint16_t) ((mpu->Mag_RxBuffer[5] << 8) | mpu->Mag_RxBuffer[4]);

			if (mpu->Calibrate_Done) {
				for (int i = 0; i < 3; i++) {
					mpu->Mag_Raw_Data[i] -= mpu->b_Mat[i];
					mpu->Mag_Raw_Data[i] *= mpu->a_Mat[i];
					mpu->Mag_Data[i] = mpu->Mag_Raw_Data[i] * mpu->Mag_ASA[i] * 0.6f * 10.0f; // 1 uT = 10 mG
				}
			}
		}
		mpu->st = 0;
	}

}

void MPU9255_Read_Raw(MPU9255_HandleTypeDef *mpu) {

	static uint8_t Data[14];
	static uint16_t temp;

	HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, ACCEL_XOUT_H, 1, Data, 14, 1000);

	mpu->Accel_Raw_Data[0] = (int16_t) ((Data[0] << 8) | Data[1]);
	mpu->Accel_Raw_Data[1] = (int16_t) ((Data[2] << 8) | Data[3]);
	mpu->Accel_Raw_Data[2] = (int16_t) ((Data[4] << 8) | Data[5]);

	temp = (int16_t) (Data[6] << 8 | Data[7]);

	mpu->Gyro_Raw_Data[0] = (int16_t) ((Data[8] << 8) | Data[9]);
	mpu->Gyro_Raw_Data[1] = (int16_t) ((Data[10] << 8) | Data[11]);
	mpu->Gyro_Raw_Data[2] = (int16_t) ((Data[12] << 8) | Data[13]);

	mpu->Temperature = (float) ((int16_t) temp / (float) 333.87f + (float) 21.00f);

}

void MPU9255_Read_Data(MPU9255_HandleTypeDef *mpu) {
	if (mpu->Mode)
		MPU9255_Read_FIFO_ALL(mpu);
	else
		MPU9255_Read_Raw(mpu);

	for (int i = 0; i < 3; i++) {
		mpu->Accel_Data[i] = (float) mpu->Accel_Raw_Data[i] * ACC_RES_2G * 10.0f; //unit G 1G = 10mG
		mpu->Gyro_Data[i] = (float) mpu->Gyro_Raw_Data[i] * GYR_RES_250DPS * DEG_2_RAD; //unit dps degree per second rad/s
	}
}

void MPU9255_Read_FIFO_ALL(MPU9255_HandleTypeDef *mpu) {

	static uint8_t FifoLenght[2];
	static uint8_t Data[14];
	static int16_t lenght;
	static int16_t temp;

	HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, FIFO_COUNTH, 1, &FifoLenght[0], 2, 10000);
	lenght = (((uint16_t) FifoLenght[0] << 8) | FifoLenght[1]);

	if (lenght >= 14) {
		HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, FIFO_R_W, 1, &Data[0], 14, 10000);

		mpu->Accel_Raw_Data[0] = (int16_t) ((Data[0] << 8) | Data[1]);
		mpu->Accel_Raw_Data[1] = (int16_t) ((Data[2] << 8) | Data[3]);
		mpu->Accel_Raw_Data[2] = (int16_t) ((Data[4] << 8) | Data[5]);

		temp = (int16_t) (Data[6] << 8 | Data[7]);
		mpu->Temperature = (float) ((int16_t) temp / (float) 333.87f + (float) 21.00f);

		mpu->Gyro_Raw_Data[0] = (int16_t) ((Data[8] << 8) | Data[9]);
		mpu->Gyro_Raw_Data[1] = (int16_t) ((Data[10] << 8) | Data[11]);
		mpu->Gyro_Raw_Data[2] = (int16_t) ((Data[12] << 8) | Data[13]);

	}

}

void MPU9255_Read_Mag(MPU9255_HandleTypeDef *mpu) {
	static uint8_t DATA_6BYTE[7];
	static uint8_t dataReady;

	HAL_I2C_Mem_Read(mpu->i2c, AK8963_ADDR, AK8963_ST1, 1, &dataReady, 1, 10000);
	if ((dataReady & 0x01)) {
		HAL_I2C_Mem_Read(mpu->i2c, AK8963_ADDR, AK8963_HXL, 1, DATA_6BYTE, 7, 10000);
		if (!(DATA_6BYTE[6] & 0x08)) {
			mpu->Mag_Raw_Data[0] = (uint16_t) ((DATA_6BYTE[1] << 8) | DATA_6BYTE[0]);
			mpu->Mag_Raw_Data[1] = (uint16_t) ((DATA_6BYTE[3] << 8) | DATA_6BYTE[2]);
			mpu->Mag_Raw_Data[2] = (uint16_t) ((DATA_6BYTE[5] << 8) | DATA_6BYTE[4]);

			if (mpu->Calibrate_Done) {
				for (int i = 0; i < 3; i++) {
					mpu->Mag_Raw_Data[i] -= mpu->b_Mat[i];
					mpu->Mag_Raw_Data[i] *= mpu->a_Mat[i];
					mpu->Mag_Data[i] = mpu->Mag_Raw_Data[i] * mpu->Mag_ASA[i] * 0.6f * 10.0f; // 1 uT = 10 mG
				}
			}

		}
	}
}

void Calibrate_Mag(MPU9255_HandleTypeDef *mpu) {
	uint16_t cnt = 0;
	static int16_t m_max[3] = { -32767 };
	static int16_t m_min[3] = { 32767 };
	static float avg = 0;

	while (cnt < 2000) {
		MPU9255_Read_Mag(mpu);
		for (int i = 0; i < 3; i++) {
			if (mpu->Mag_Raw_Data[i] > m_max[i])
				m_max[i] = mpu->Mag_Raw_Data[i];
			if (mpu->Mag_Raw_Data[i] < m_min[i])
				m_min[i] = mpu->Mag_Raw_Data[i];
		}
		cnt++;
		HAL_Delay(10);
	}
	for (int i = 0; i < 3; i++) {
		mpu->b_Mat[i] = (float) (m_max[i] + m_min[i]) / 2.0f;
		mpu->a_Mat[i] = (float) (m_max[i] - m_min[i]) / 2.0f;
		avg += (float) (mpu->a_Mat[i] / 3.0f);
	}
	for (int j = 0; j < 3; j++) {
		mpu->a_Mat[j] = (float) (avg / mpu->a_Mat[j]);
	}
	mpu->Calibrate_Done = 1;

}

void Calibrate_Acc_Gyro(MPU9255_HandleTypeDef *mpu, uint16_t sample) {
	uint16_t cnt = 0;

	while (cnt < sample) {

		MPU9255_Read_Raw(mpu);
		mpu->a_bias[0] += (float) mpu->Accel_Raw_Data[0];
		mpu->a_bias[1] += (float) mpu->Accel_Raw_Data[1];
		mpu->a_bias[2] += (float) mpu->Accel_Raw_Data[2];

		mpu->g_bias[0] += (float) mpu->Gyro_Raw_Data[0];
		mpu->g_bias[1] += (float) mpu->Gyro_Raw_Data[1];
		mpu->g_bias[2] += (float) mpu->Gyro_Raw_Data[2];

		cnt++;
		HAL_Delay(10);
	}
	mpu->a_bias[0] /= (float) sample;
	mpu->a_bias[1] /= (float) sample;
	mpu->a_bias[2] /= (float) sample;

	mpu->g_bias[0] /= (float) sample;
	mpu->g_bias[1] /= (float) sample;
	mpu->g_bias[2] /= (float) sample;

	if (mpu->a_bias[2] > 0L) {
		mpu->a_bias[2] -= (float) (16384.0f);
	}
	else {
		mpu->a_bias[2] += (float) (16384.0f);
	}
	cnt = 0;

	Write_Calib_Data(mpu);
	HAL_Delay(100);

}

void Write_Calib_Data(MPU9255_HandleTypeDef *mpu) {
	uint16_t abias_reg[3] = { 0, 0, 0 };
	uint8_t data[2];

	HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, XA_OFFSET_H, 1, &data[0], 2, 1000);
	abias_reg[0] = (((uint16_t) data[0] << 8) | data[1]);
	HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, 0x7A, 1, &data[0], 2, 1000);
	abias_reg[1] = (((uint16_t) data[0] << 8) | data[1]);
	HAL_I2C_Mem_Read(mpu->i2c, MPU_9255_ADDR, 0x7D, 1, &data[0], 2, 1000);
	abias_reg[2] = (((uint16_t) data[0] << 8) | data[1]);

	int16_t mask_bit[3] = { 1, 1, 1 };
	for (int i = 0; i < 3; i++) {
		if (abias_reg[i] % 2) {
			mask_bit[i] = 0;
		}
		abias_reg[i] -= (int16_t) mpu->a_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
		if (mask_bit[i]) {
			abias_reg[i] = abias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
		}
		else {
			abias_reg[i] = abias_reg[i] | 0x0001;  // Preserve temperature compensation bit
		}
	}

	uint8_t write_data[6] = { 0 };
	write_data[0] = (abias_reg[0] >> 8) & 0xFF;
	write_data[1] = (abias_reg[0]) & 0xFF;
	write_data[2] = (abias_reg[1] >> 8) & 0xFF;
	write_data[3] = (abias_reg[1]) & 0xFF;
	write_data[4] = (abias_reg[2] >> 8) & 0xFF;
	write_data[5] = (abias_reg[2]) & 0xFF;

	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, XA_OFFSET_H, 1, &write_data[0], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x78, 1, &write_data[1], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x7A, 1, &write_data[2], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x7B, 1, &write_data[3], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x7D, 1, &write_data[4], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x7E, 1, &write_data[5], 1, 1000);

	uint8_t gyro_offset_data[6] = { 0 };
	gyro_offset_data[0] = (-(int16_t) mpu->g_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	gyro_offset_data[1] = (-(int16_t) mpu->g_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
	gyro_offset_data[2] = (-(int16_t) mpu->g_bias[1] / 4 >> 8) & 0xFF;
	gyro_offset_data[3] = (-(int16_t) mpu->g_bias[1] / 4) & 0xFF;
	gyro_offset_data[4] = (-(int16_t) mpu->g_bias[2] / 4 >> 8) & 0xFF;
	gyro_offset_data[5] = (-(int16_t) mpu->g_bias[2] / 4) & 0xFF;

	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, XG_OFFSET_H, 1, &gyro_offset_data[0], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x14, 1, &gyro_offset_data[1], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x15, 1, &gyro_offset_data[2], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x16, 1, &gyro_offset_data[3], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x17, 1, &gyro_offset_data[4], 1, 1000);
	HAL_I2C_Mem_Write(mpu->i2c, MPU_9255_ADDR, 0x18, 1, &gyro_offset_data[5], 1, 1000);
}
