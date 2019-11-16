#include "APDS9960.h"


void APDS9960_Write8(uint8_t Reg, uint8_t size, uint8_t Val)
{
	uint8_t data[2];

	data[0] = Reg;
	data[1] = Val;

	HAL_I2C_Master_Transmit(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, data, 1, 50);
}


uint8_t APDS9960_Read8(uint8_t Reg)
{
	uint8_t ret;
	HAL_I2C_Master_Receive(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, &ret, 1, 50);
	return ret;
}

uint16_t APDS9960_Read16(uint8_t Reg)
{
	uint8_t ret[2];
	HAL_I2C_Master_Receive(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, &ret, 1, 50);
	return (ret[0] << 8) | ret[1];
}

uint16_t APDS9960_Read16R(uint8_t Reg)
{
	uint8_t ret[2];
	HAL_I2C_Master_Receive(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, &ret, 1, 50);
	return (ret[1] << 8) | ret[0];
}

uint16_t APDS9960_Read32(uint8_t Reg)
{
	uint8_t ret[4];
	HAL_I2C_Master_Receive(&APDS9960_I2C_PORT, APDS9960_I2C_ADDR, &ret, 1, 50);
	 return (ret[0] << 24) | (ret[1] << 16) | (ret[2] << 8) | ret[3];
}




