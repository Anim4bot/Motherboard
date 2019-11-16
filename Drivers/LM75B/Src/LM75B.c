#include "LM75B.h"


void LM75B_Init(void)
{
	uint8_t data[2];

	data[0] = LM75B_CONF;	// Config Reg
	data[1] = 0x00;			// Normal Op

	HAL_I2C_Master_Transmit(&LM75B_I2C_PORT, LM75B_CHG_ADDR, data, 2, 50);
	HAL_I2C_Master_Transmit(&LM75B_I2C_PORT, LM75B_PSU_ADDR, data, 2, 50);
}


uint16_t LM75B_CHG_Read16(void)
{
	uint8_t retVal[2];
	int16_t data;

	HAL_I2C_Mem_Read(&LM75B_I2C_PORT, LM75B_CHG_ADDR, LM75B_TEMP, 1, (uint8_t*)retVal, 2, 50);

	data = ((retVal[0]<<8) | retVal[1]&0xE0)>>5;
	return data;
}


uint16_t LM75B_PSU_Read16(void)
{
	uint8_t retVal[2];
	uint16_t data;

	HAL_I2C_Mem_Read(&LM75B_I2C_PORT, LM75B_PSU_ADDR, LM75B_TEMP, 1, (uint8_t*)retVal, 2, 50);

	data = ((retVal[0]<<8) | retVal[1]&0xE0)>>5;
	return data;
}


void LM75B_CHG_GetTemp(float *TempVal)
{
	float temp;
	float C1 = 0.50;
	float C2 = (1 - C1);

	temp = LM75B_CHG_Read16() * 0.125;
	*TempVal  = (uint16_t)(C1 * (*TempVal) ) + (C2 * temp);
}


void LM75B_PSU_GetTemp(float *TempVal)
{
	float temp;
	float C1 = 0.50;
	float C2 = (1 - C1);

	temp = LM75B_PSU_Read16() * 0.125;
	*TempVal  = (uint16_t)(C1 * (*TempVal) ) + (C2 * temp);
}


