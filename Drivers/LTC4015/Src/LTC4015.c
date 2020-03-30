#include "LTC4015.h"
#include "LTC4015_formats.h"
#include "Robot.h"


LTC4015_SystemStatus logSystemStatus[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logChargerState[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logChargeStatus[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logLimitAlerts[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};


HAL_StatusTypeDef LTC4015_write16(uint8_t Reg, uint16_t Val)
{
	HAL_StatusTypeDef status;
	uint16_t data[2];

	data[0] = Val<<8;
	data[1] = Val;

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, data, 2, 1000);
	return status;
}


HAL_StatusTypeDef LTC4015_read16(uint8_t Reg, uint16_t *data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, *data, 2, 1000);
	return status;
}


void LTC4015_Init(void)
{
	HAL_StatusTypeDef status;
	float SysCurrent = 0;
	uint8_t received[2];
	uint8_t data[3];

	data[0] = REG_CONFIG_BITS;
	data[1] = 0b00010100;
	data[2] = 0b00000000;

	status = HAL_I2C_Master_Transmit(&LTC4015_I2C_PORT, LTC4015_ADDR, &data, 3, 1000);

}


HAL_StatusTypeDef LTC4015_GetPowerVal(void)
{
	HAL_StatusTypeDef status;

	uint8_t received[2];
	float tempdata = 0;
	float Die_temp = 0;
	float InputVoltage = 0;
	float SysVoltage = 0;
	float BatVoltage = 0;
	float BatCurrent = 0;
	float SysCurrent = 0;

	float C1 = 0.50;
	float C2 = (1 - C1);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, (uint8_t*)received, 2, 1000);
	Die_temp = (((received[1]<<8) | received[0])-12010)/45.6;
	//Die_temp  = (uint16_t)(C1 * (Die_temp) ) + (C2 * dietemptemp);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VIN, 1, (uint8_t*)received, 2, 1000);
	InputVoltage = ((received[1]<<8) | received[0])*1.648;
	//InputVoltage  = (uint16_t)(C1 * (InputVoltage) ) + (C2 * inputvolttemp);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IIN, 1, (uint8_t*)received, 2, 1000);
	SysCurrent = (((received[1]<<8) | received[0])*1.46487E-6)/4.0E-3;
	//SysCurrent  = (uint16_t)(C1 * (SysCurrent) ) + (C2 * syscurrtemp);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VSYS, 1, (uint8_t*)received, 2, 1000);
	SysVoltage = ((received[1]<<8) | received[0])*1.648;
	//SysVoltage  = (uint16_t)(C1 * (SysVoltage) ) + (C2 * sysvolttemp);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VBAT, 1, (uint8_t*)received, 2, 1000);
	BatVoltage = (((received[1]<<8) | received[0])*192.264E-6)*3;
	//BatVoltage  = (uint16_t)(C1 * (BatVoltage) ) + (C2 * batvolttemp);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IBAT, 1, (uint8_t*)received, 2, 1000);
	BatCurrent = (((received[1]<<8) | received[0])*1.46487E-6)/20.0E-3;
	//BatCurrent  = (uint16_t)(C1 * (BatCurrent) ) + (C2 * batcurrtemp);


	Charger.Power.Die_temp = Die_temp;								// Result in °C
	Charger.Power.InputVoltage = ABS(InputVoltage/1000.0);			// in V
	Charger.Power.SysVoltage   = ABS(SysVoltage/1000.0);			// in V
	Charger.Power.SysCurrent   = ABS(SysCurrent);					// in A
	Charger.Power.SysPower     = ABS(SysCurrent*SysVoltage/1000.0);	// Power in Watt
	Charger.Power.BatVoltage   = ABS(BatVoltage);					// in V
	Charger.Power.BatCurrent   = ABS(BatCurrent);					// in A

	return status;
}


// 0x34 - CHARGER_STATE
HAL_StatusTypeDef LTC4015_GetChargerState(LTC4015_ChargerState* chargerState)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGER_STATE, 1, (uint8_t*)received, 2, 1000);
	*chargerState = ((received[1]<<8) | received[0]);

	Charger.ChargerState = *chargerState;
	return status;
}


// 0x35 - CHARGE_STATUS
HAL_StatusTypeDef LTC4015_GetChargeStatus(LTC4015_ChargeStatus* chargeStatus)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGE_STATUS, 1, (uint8_t*)received, 2, 1000);
	*chargeStatus = ((received[1]<<8) | received[0]);

	Charger.ChargeStatus = *chargeStatus;
	return status;
}


// 0x36 - LIMIT_ALERTS
HAL_StatusTypeDef LTC4015_GetLimitAlerts(LTC4015_LimitAlerts* limitAlerts)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_LIMIT_ALERTS, 1, (uint8_t*)received, 2, 1000);
	*limitAlerts = ((received[1]<<8) | received[0]);

	Charger.LimitAlerts = *limitAlerts;
	return status;
}


// 0x37 - CHARGER_STATE_ALERTS
HAL_StatusTypeDef LTC4015_GetChargerStateAlerts(LTC4015_ChargeStateAlerts* stateAlerts)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGER_STATE_ALERTS, 1, (uint8_t*)received, 2, 1000);
	*stateAlerts = ((received[1]<<8) | received[0]);

	Charger.ChargeStateAlerts = *stateAlerts;
	return status;
}


// 0x38 - CHARGE_STATUS_ALERTS
HAL_StatusTypeDef LTC4015_GetChargeStatusAlerts(LTC4015_ChargeStatusAlerts* statusAlerts)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGE_STATUS_ALERTS, 1, (uint8_t*)received, 2, 1000);
	*statusAlerts = ((received[1]<<8) | received[0]);

	Charger.ChargeStatusAlerts = *statusAlerts;
	return status;
}


// 0x39 - SYSTEM_STATUS
HAL_StatusTypeDef LTC4015_GetSystemStatus(LTC4015_SystemStatus* systemStatus)
{
	HAL_StatusTypeDef status;
	uint8_t received[2];

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_SYSTEM_STATUS, 1, (uint8_t*)received, 2, 1000);
	*systemStatus = ((received[1]<<8) | received[0]);

	Charger.SystemStatus = *systemStatus;
	return status;
}


