#include "LTC4015.h"
#include "LTC4015_formats.h"
//#include "LTC4015_reg_defs.h"


HAL_StatusTypeDef LTC4015_write16(uint8_t Reg, uint16_t Val)
{
	HAL_StatusTypeDef status;
	uint16_t data[3];

	data[0] = Reg;
	data[1] = Val << 8;
	data[2] = Val;

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, data, 2, 50);
	return status;
}


HAL_StatusTypeDef LTC4015_read16(uint8_t Reg, uint16_t *data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, *data, 2, 50);
	return status;
}


void LTC4015_GetChargerState(uint16_t* LTC4015_State)
{
	HAL_StatusTypeDef status;
	uint16_t *ChargerState;
	uint16_t *ChargeStatus;
	uint16_t *LimitsAlert;
	uint8_t DC_IN[2];
	uint16_t InputVoltage;
	uint8_t V_SYS[2];
	uint16_t SysVoltage;
	uint8_t V_BAT[2];
	uint16_t BatVoltage;
	uint8_t I_BAT[2];
	uint16_t BatCurrent;
	uint8_t I_IN[2];
	uint16_t DCCurrent;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x34, 1, *ChargerState, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x35, 1, *ChargeStatus, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x36, 1, *LimitsAlert, 2, 50);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x3B, 1, &DC_IN, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x3C, 1, &V_SYS, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x3A, 1, &V_BAT, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x3D, 1, &I_BAT, 2, 50);
	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, 0x3E, 1, &I_IN, 2, 50);

	InputVoltage = ((DC_IN[1]<<8)  | DC_IN[1])*1.648;
	SysVoltage 	 = ((V_SYS[1]<<8)  | V_SYS[1])*1.648;
	BatVoltage 	 = (((V_BAT[1]<<8) | V_BAT[1])*192.264E-6)*3;
	BatCurrent 	 = (((I_BAT[1]<<8) | I_BAT[1])*1.46487E-6)/20.0E-3;
	DCCurrent 	 = (((I_IN[1]<<8)  | I_IN[1])*1.46487E-6)/4.0E-3;


}

void LTC4015_Init(void)
{
	uint16_t data;
	HAL_StatusTypeDef status;

	//LTC4015_write16(LTC4015_EN_VSYS_LO_ALERT_BF, 0x01);
	//LTC4015_read16(LTC4015_EN_VSYS_LO_ALERT_BF, &data);

	/*
	LTC4015_write16(VBAT_LO_ALERT_LIMIT, ); 	//VBATSENS/cellcount = [VBAT] • 192.264µV for lithium chemistries.
	LTC4015_write16(VBAT_HI_ALERT_LIMIT, );
	LTC4015_write16(VIN_LO_ALERT_LIMIT, );		//Two’s complement ADC measurement result for VIN.	VVIN = [VIN] • 1.648mV
	LTC4015_write16(VIN_HI_ALERT_LIMIT, );
	LTC4015_write16(VSYS_LO_ALERT_LIMIT, );		//Two’s complement ADC measurement result for VSYS.	VSYS = [VSYS] • 1.648mV
	LTC4015_write16(VSYS_HI_ALERT_LIMIT, );
	*/
}
