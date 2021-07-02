/*
 * MPU9250.c
 *
 *  	@Created on: Jan 18, 2021
 *      @Author: Talha Sari
 *      @License: GNU General Public License v3
 *      @Brief: This library is created for STM32 development environment and can be used only with STM32F series microcontrollers.
 *      @Note: Requires HAL Libraries!
 */

#include "main.h"
#include "MPU9250.h"
#include "printf_SWO.h"

#define PI	3.14159265359f

/* Define the I2C Handler's name according to I2C port in use */
extern I2C_HandleTypeDef hi2c1;

static inline float deg2rad(float x);

MPU9250_Result_t MPU9250_Init(MPU9250_t *MPU9250, MPU9250_Device_t dev,
		MPU9250_Accel_Scale_t accScale, MPU9250_Gyro_Scale_t gyroScale,
		MPU9250_Mag_Scale_t magScale) {
	uint8_t data;
	MPU9250->I2C_Addr = MPU9250_I2C_ADDR | (uint8_t) dev;
	MPU9250->I2C_Addr_Mag = MPU9250_I2C_ADDR_MAG;

	/* Gyro & Acc Device Connection Check */
	if (isMPU9250Ready(&hi2c1, MPU9250->I2C_Addr) != MPU9250_RESULT_OK)
		return MPU9250_RESULT_NC;

	/* Who Am I Check */
	readByte(&hi2c1, MPU9250->I2C_Addr, WHO_AM_I, &data);
	if (data != 0x71)
		return MPU9250_RESULT_NC;

	/* Device Wake Up */
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x00);
	HAL_Delay(100);

	/* Auto-select best (stable) available clock source */
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x01);
	HAL_Delay(200);

	writeByte(&hi2c1, MPU9250->I2C_Addr, CONFIG, 0x03);

	writeByte(&hi2c1, MPU9250->I2C_Addr, SMPLRT_DIV, 0x04);

	/* Gyro Configuration */
	readByte(&hi2c1, MPU9250->I2C_Addr, GYRO_CONFIG, &data);
	data &= ~0x02; /* [1:0] - Clear Fchoice_b[1:0] bits */
	data &= ~0x18; /* [4:3] - Clear GYRO_FS_SEL[1:0] bits */
	data |= gyroScale;
	writeByte(&hi2c1, MPU9250->I2C_Addr, GYRO_CONFIG, data);

	/* Accel Configuraiton */

	readByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG, &data);
	data &= ~0x18; /* [4:3] - Clear ACCEL_FS_SEL[1:0] bits */
	data |= accScale;
	writeByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG, data);

	/* Accel_2 Configuration */

	readByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG_2, &data);
	data &= ~0x0F; /* Clear ACCEL_CONFIG_2[3:0] Bits */
	data |= 0x03; /* A_DLPFCFG[2:0] bits are set to 011*/
	writeByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG_2, data);

	readByte(&hi2c1, MPU9250->I2C_Addr, LP_ACCEL_ODR, &data);

	printf("LP_ACCEL_ODR = %x\r\n", data);
//	status = HAL_UART_Transmit (&huart4, pTxData, strlen((char *)pTxData), 200);

	writeByte(&hi2c1, MPU9250->I2C_Addr, LP_ACCEL_ODR, 0x02);

	writeByte(&hi2c1, MPU9250->I2C_Addr, INT_PIN_CFG, 0x22);
	writeByte(&hi2c1, MPU9250->I2C_Addr, INT_ENABLE, 0x01);

	/* Magnetometer Device Connection Check */
	if (isMPU9250Ready(&hi2c1, MPU9250->I2C_Addr_Mag) != MPU9250_RESULT_OK)
		return MPU9250_RESULT_NC;

	/* Magnetometer Power Down */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x00);
	HAL_Delay(10);
	/* Magnetometer Fuse ROM Access Mode ON */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x0F);
	HAL_Delay(10);
	/* Magnetometer Power Down */
	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, 0x00);
	HAL_Delay(10);

	writeByte(&hi2c1, MPU9250->I2C_Addr_Mag, CNTL, (1 << 4) | 2);
	HAL_Delay(10);

	/*
	 * Accelerometer Full Scale: 	±16g
	 * Gyroscope Full Scale:		±2000 degree/s
	 * Magnetometer Full Scale:		±4912 uT
	 */

	/* Accelerometer Resolution Multiplicator: LSB / g 			*/
	switch (accScale) {
	case ACCEL_SCALE_2G:
		MPU9250->accMult = 2.0f / 32768.0f;
		break;
	case ACCEL_SCALE_4G:
		MPU9250->accMult = 4.0f / 32768.0f;
		break;
	case ACCEL_SCALE_8G:
		MPU9250->accMult = 8.0f / 32768.0f;
		break;
	case ACCEL_SCALE_16G:
		MPU9250->accMult = 16.0f / 32768.0f;
		break;
	}

	/* Gyroscope Resolution Multiplicator: LSB / (degree/s) 	*/
	switch (gyroScale) {
	case GYRO_SCALE_250dps:
		MPU9250->gyroMult = 250.0f / 32768.0f;
		break;
	case GYRO_SCALE_500dps:
		MPU9250->gyroMult = 500.0f / 32768.0f;
		break;
	case GYRO_SCALE_1000dps:
		MPU9250->gyroMult = 1000.0f / 32768.0f;
		break;
	case GYRO_SCALE_2000dps:
		MPU9250->gyroMult = 2000.0f / 32768.0f;
		break;
	}

	/* Magnetometer Resolution Multiplicator: LSB / 0.15uT 		*/
	switch (magScale) {
	case MAG_SCALE_14bit:
		MPU9250->magMult = 10.0 * 4219.0 / 8190.0;
		break;
	case MAG_SCALE_16bit:
		MPU9250->magMult = 10.0 * 4219.0 / 32760.0;
		break;
	}

	/* Temperature Resolution Multiplicator: LSB / degreeC		*/
	MPU9250->tempMult = 333.87f;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_Calibrate(MPU9250_t *MPU9250, float *dest1 , float *dest2) {
	int8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x80);
	HAL_Delay(100);

	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x01);
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_2, 0x00);
	HAL_Delay(200);

    // Configure device for bias calculation
	writeByte(&hi2c1, MPU9250->I2C_Addr, INT_ENABLE, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, FIFO_EN, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, PWR_MGMT_1, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, I2C_MST_CTRL, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, USER_CTRL, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, I2C_MST_CTRL, 0x0C);
	HAL_Delay(15);


    // Configure MPU9250 gyro and accelerometer for bias calculation
	writeByte(&hi2c1, MPU9250->I2C_Addr, CONFIG, 0x01);
	writeByte(&hi2c1, MPU9250->I2C_Addr, SMPLRT_DIV, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, GYRO_CONFIG, 0x00);
	writeByte(&hi2c1, MPU9250->I2C_Addr, ACCEL_CONFIG, 0x00);


    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g



    // Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(&hi2c1, MPU9250->I2C_Addr, USER_CTRL, 0x40);
	writeByte(&hi2c1, MPU9250->I2C_Addr, FIFO_EN, 0x78);
	HAL_Delay(40);

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(&hi2c1, MPU9250->I2C_Addr, FIFO_EN, 0x00);
	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, ACCEL_XOUT_H, &data[0], 2);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging
	for (ii = 0; ii < packet_count; ii++)
	    {
	        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	        readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag, FIFO_R_W, &data[0], 12);
	         // read data for averaging
	        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
	        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
	        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
	        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
	        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
	        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

	        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	        accel_bias[1] += (int32_t)accel_temp[1];
	        accel_bias[2] += (int32_t)accel_temp[2];
	        gyro_bias[0] += (int32_t)gyro_temp[0];
	        gyro_bias[1] += (int32_t)gyro_temp[1];
	        gyro_bias[2] += (int32_t)gyro_temp[2];
	    }
	    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	    accel_bias[1] /= (int32_t)packet_count;
	    accel_bias[2] /= (int32_t)packet_count;
	    gyro_bias[0] /= (int32_t)packet_count;
	    gyro_bias[1] /= (int32_t)packet_count;
	    gyro_bias[2] /= (int32_t)packet_count;

	    if (accel_bias[2] > 0L)
	        {
	            accel_bias[2] -= (int32_t)accelsensitivity;
	        } // Remove gravity from the z-axis accelerometer bias calculation
	        else
	        {
	            accel_bias[2] += (int32_t)accelsensitivity;
	        }

	        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	        data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	        data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
	        data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	        data[3] = (-gyro_bias[1] / 4) & 0xFF;
	        data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	        data[5] = (-gyro_bias[2] / 4) & 0xFF;

	        dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	          dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	          dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	          int32_t accel_bias_reg[3] = {0, 0, 0};
	          // A place to hold the factory accelerometer trim biases
	          readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag,XA_OFFSET_H,&data[0],2);
	        // Read factory accelerometer trim values
	          accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
	          readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag,YA_OFFSET_H,&data[0],2);

	          accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
	          accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
		          readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag,ZA_OFFSET_H,&data[0],2);
	          accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

	          uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	             uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	             for (ii = 0; ii < 3; ii++)
	             {
	                 if (accel_bias_reg[ii] & mask)
	                     mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	             }

	             accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	             accel_bias_reg[1] -= (accel_bias[1] / 8);
	             accel_bias_reg[2] -= (accel_bias[2] / 8);

	             data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	             data[1] = (accel_bias_reg[0]) & 0xFF;
	             data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	             data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	             data[3] = (accel_bias_reg[1]) & 0xFF;
	             data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	             data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	             data[5] = (accel_bias_reg[2]) & 0xFF;
	             data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers


	             dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	             dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	             dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

	         	return MPU9250_RESULT_OK;

}

MPU9250_Result_t MPU9250_ReadAcc(MPU9250_t *MPU9250) {
	uint8_t data[6];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, ACCEL_XOUT_H, data, 6);

	MPU9250->acc_raw[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Turn the MSB and LSB into a signed 16-bit value
	MPU9250->acc_raw[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
	MPU9250->acc_raw[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
//
//	MPU9250->acc_raw[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
//	MPU9250->acc_raw[1] = ((int16_t)data[2] << 8) | data[3];
//	MPU9250->acc_raw[2] = ((int16_t)data[4] << 8) | data[5];

	MPU9250->acc[0] = (float) MPU9250->acc_raw[0] * MPU9250->accMult;
	MPU9250->acc[1] = (float) MPU9250->acc_raw[1] * MPU9250->accMult;
	MPU9250->acc[2] = (float) MPU9250->acc_raw[2] * MPU9250->accMult;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadGyro(MPU9250_t *MPU9250) {
	uint8_t data[6];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, GYRO_XOUT_H, data, 6);

	MPU9250->gyro_raw[0] = ((int16_t) data[0] << 8) | data[1];
	MPU9250->gyro_raw[1] = ((int16_t) data[2] << 8) | data[3];
	MPU9250->gyro_raw[2] = ((int16_t) data[4] << 8) | data[5];

	MPU9250->gyro[0] = (float) MPU9250->gyro_raw[0] * MPU9250->gyroMult;
	MPU9250->gyro[1] = (float) MPU9250->gyro_raw[1] * MPU9250->gyroMult;
	MPU9250->gyro[2] = (float) MPU9250->gyro_raw[2] * MPU9250->gyroMult;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadMag(MPU9250_t *MPU9250) {
	uint8_t data[7];
	uint8_t check;

	/* Check Mag Data Ready Status */
	readByte(&hi2c1, MPU9250->I2C_Addr_Mag, ST1, &check);

	if (check & 0x01) {
		readMultiBytes(&hi2c1, MPU9250->I2C_Addr_Mag, HXL, data, 7);
		/* Check (ST2 Register) If Magnetic Sensor Overflow Occured */
		if (!(data[6] & 0x08)) {
			MPU9250->mag_raw[0] = ((int16_t) data[1] << 8) | data[0];
			MPU9250->mag_raw[1] = ((int16_t) data[3] << 8) | data[2];
			MPU9250->mag_raw[2] = ((int16_t) data[5] << 8) | data[4];

			MPU9250->mag[0] = deg2rad(
					(float) MPU9250->mag_raw[0] * MPU9250->magMult);
			MPU9250->mag[1] = deg2rad(
					(float) MPU9250->mag_raw[1] * MPU9250->magMult);
			MPU9250->mag[2] = deg2rad(
					(float) MPU9250->mag_raw[2] * MPU9250->magMult);

			return MPU9250_RESULT_OK;
		}
		return MPU9250_RESULT_ERROR;
	}
	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_ReadTemperature(MPU9250_t *MPU9250) {
	uint8_t data[2];

	readMultiBytes(&hi2c1, MPU9250->I2C_Addr, TEMP_OUT_H, data, 2);

	MPU9250->temp_raw = ((int16_t) data[0] << 8) | data[1];

	MPU9250->temp = ((float) MPU9250->temp_raw / MPU9250->tempMult) + 21.0f;

	return MPU9250_RESULT_OK;
}

MPU9250_Result_t MPU9250_DataReady(MPU9250_t *MPU9250) {
	uint8_t data;
	readByte(&hi2c1, MPU9250->I2C_Addr, INT_STATUS, &data);
	if (data & 0x01)
		return MPU9250_RESULT_OK;

	return MPU9250_RESULT_ERROR;
}

HAL_StatusTypeDef writeByte(I2C_HandleTypeDef *hi2c1, uint8_t device_addr,
		uint8_t register_addr, uint8_t data) {
	uint8_t buffer[2];
	buffer[0] = register_addr;
	buffer[1] = data;

	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t) device_addr,
			(uint8_t*) buffer, 2, 1000) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	return HAL_OK;

}

HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c1, uint8_t device_addr,
		uint8_t register_addr, uint8_t *data) {
	/* Transmit Register Address */
	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t) device_addr, &register_addr,
			1, 1000) != HAL_OK) {
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
			}
			return HAL_ERROR;
		}
	}

	/* Receive Register Data */
	if (HAL_I2C_Master_Receive(hi2c1, (uint16_t) device_addr, data, 1, 1000)
			!= HAL_OK) {
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
			}
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef writeMultiBytes(I2C_HandleTypeDef *hi2c1, uint8_t device_addr,
		uint8_t register_addr, uint8_t *data, uint16_t count) {
	if (HAL_I2C_Mem_Write(hi2c1, (uint16_t) device_addr, register_addr,
			register_addr > 0xFF ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT,
			data, count, 1000) != HAL_OK) {
		{
			if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
			}
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef readMultiBytes(I2C_HandleTypeDef *hi2c1, uint8_t device_addr,
		uint8_t register_addr, uint8_t *data, uint16_t count) {
	/* Transmit Register Address */
	if (HAL_I2C_Master_Transmit(hi2c1, (uint16_t) device_addr, &register_addr,
			1, 1000) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}

	/* Receive Multiple Register Data */
	if (HAL_I2C_Master_Receive(hi2c1, (uint16_t) device_addr, data, count, 1000)
			!= HAL_OK) {
		if (HAL_I2C_GetError(hi2c1) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	return HAL_OK;
}

MPU9250_Result_t isMPU9250Ready(I2C_HandleTypeDef *hi2c1, uint8_t device_addr) {
	/* Checks if device is ready to communicate */
	if (HAL_I2C_IsDeviceReady(hi2c1, (uint16_t) device_addr, 2, 5) != HAL_OK)
		return MPU9250_RESULT_NC;

	return MPU9250_RESULT_OK;
}

static inline float deg2rad(float x) {
	return ((PI / 180.0f) * x);
}
