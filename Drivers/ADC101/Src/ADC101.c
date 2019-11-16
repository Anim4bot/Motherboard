#include "ADC101.h"


HAL_StatusTypeDef ADC101_Init(void)
{
	HAL_StatusTypeDef status;
	uint32_t I2C_Timeout = 0;

	uint8_t RegVal = 0b10000001;

	status = HAL_I2C_Master_Transmit(&ADC101_I2C_PORT, ADC101_ADDR, RegVal, 1, 50);
	return status;
}


HAL_StatusTypeDef ADC101_readIR(uint16_t *ConvVal)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];
	uint16_t temp;
	float C1 = 0.70;
	float C2 = (1 - C1);


	status = HAL_I2C_Mem_Read(&ADC101_I2C_PORT, ADC101_ADDR, 0x00, 1, &data, 2, 50);
	temp = (data[0]<<8 | data[1]);

	*ConvVal  = (uint16_t)(C1 * (*ConvVal) ) + (C2 * temp);

	return status;
}



