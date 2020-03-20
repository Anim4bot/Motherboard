#include "LSM9DS1.h"



HAL_StatusTypeDef LSM9DS1_Write8(uint8_t Reg, uint8_t data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, &Reg, 1, &data, 1, 50);
	return status;
}


uint8_t LSM9DS1_Read8(uint8_t Reg)
{
	HAL_StatusTypeDef status;
	uint8_t retVal;

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, &Reg, 1, (uint8_t*)retVal, 1 , 50);
	return retVal;
}


uint16_t LSM9DS1_Read16(uint8_t Reg)
{
	HAL_StatusTypeDef status;
	uint8_t retVal[2];

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, &Reg, 1, (uint8_t*)retVal, 2 , 50);
	return (retVal[1] << 8) | retVal[0];
}



void LSM9DS1_Init(void)
{
	HAL_StatusTypeDef status;
	uint8_t RegVal1, RegVal2;
	uint8_t data[2];

	//Gyro
	/*
	LSM9DS1_Write8(CTRL_REG1_G,  0b01100011);	//ODR 119Hz - 245dps
	LSM9DS1_Write8(CTRL_REG2_G,  0x00);
	LSM9DS1_Write8(CTRL_REG3_G,  0x00);
	LSM9DS1_Write8(CTRL_REG4,    0b00111000);
	LSM9DS1_Write8(ORIENT_CFG_G, 0x00);

	//Acc
	LSM9DS1_Write8(CTRL_REG5_XL, 0b01111000);
	//LSM9DS1_Write8(CTRL_REG6_XL, 0b01100000);	//ODR 119Hz - 2g
	LSM9DS1_Write8(CTRL_REG7_XL, 0x00);
	LSM9DS1_Write8(CTRL_REG8,    0x00);
	LSM9DS1_Write8(CTRL_REG9,    0x00);
	LSM9DS1_Write8(CTRL_REG10,   0x00);
*/

	data[0] = CTRL_REG1_G;
	data[1] = 0b01100011;
	status = HAL_I2C_Master_Transmit(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, &data, 2, 50);
	asm("NOP");
	HAL_Delay(50);
	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, CTRL_REG1_G,  1, &RegVal1, 1 , 50);
	asm("NOP");
	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, CTRL_REG6_XL, 1, &RegVal2, 1 , 50);
	asm("NOP");

}


HAL_StatusTypeDef LSM9DS1_ReadAcc(float *xAcc, float *yAcc, float *zAcc)
{
	volatile HAL_StatusTypeDef status;
	uint8_t retVal[2];
	uint8_t test1, test2;
	int16_t tempx, tempy, tempz;

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, WHO_AM_I_XG, 1, (uint8_t*)retVal, 1 , 50);
	asm("NOP");

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_X_L_XL, 1, (uint8_t*)retVal, 2 , 50);
	tempx = ( (retVal[1]<<8) | retVal[0] );
	*xAcc = tempx*SENSITIVITY_ACCELEROMETER_2;

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_Y_L_XL, 1, (uint8_t*)retVal, 2, 10);
	tempy = (int16_t)( (retVal[1]<<8) | retVal[0] );
	*yAcc = tempy*SENSITIVITY_ACCELEROMETER_2;

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_Z_L_XL, 1, (uint8_t*)retVal, 2, 10);
	tempz = (int16_t)( (retVal[1]<<8) | retVal[0] );
	*zAcc = tempz*SENSITIVITY_ACCELEROMETER_2;

	return status;
}


HAL_StatusTypeDef LSM9DS1_ReadAngle(float *rollF, float *pitchF)
{
	HAL_StatusTypeDef status;

	float x, y ,z;
	float pitch, roll;
	float C1, C2;

	C1 = 0.50;
	C2 = (1 - C1);

	LSM9DS1_ReadAcc(&x, &y, &z);

	pitch = atan(y / sqrt(pow(x, 2) + pow(z, 2))) * 180.0 / PI;
	roll = atan(-1 * x / sqrt(pow(y, 2) + pow(z, 2))) * 180.0 / PI;

	// Low-pass filter
	*rollF  = (C1 * (*rollF) ) + (C2 * roll);
	*pitchF = (C1 * (*pitchF)) + (C2 * pitch);

	return status;
}


HAL_StatusTypeDef LSM9DS1_ReadGyro(float *xRot, float *yRot, float *zRot)
{
	HAL_StatusTypeDef status;
	uint8_t data_rec[2];
	int16_t tempx, tempy, tempz;

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_X_L_G, 1, &data_rec, 2 , 50);
	tempx = (int16_t)( (data_rec[1]<<8) | data_rec[0] );
	*xRot = (tempx*SENSITIVITY_GYROSCOPE_245);

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_Y_L_G, 1, &data_rec, 2, 10);
	tempy = (int16_t)( (data_rec[1]<<8) | data_rec[0] );
	*yRot = (tempy*SENSITIVITY_GYROSCOPE_245);

	status = HAL_I2C_Mem_Read(&LSM9DS1_I2C_PORT, LSM9DS1_AG_ADDR, OUT_Z_L_G, 1, &data_rec, 2, 10);
	tempz = (int16_t)( (data_rec[1]<<8) | data_rec[0] );
	*zRot = (tempz*SENSITIVITY_GYROSCOPE_245);

	return status;
}

