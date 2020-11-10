#include "LTC4015.h"
#include "LTC4015_formats.h"
#include "Robot.h"

uint8_t ctr = 0;
const uint8_t Samples = 3;

float buffDie_temp[3];
float buffInputVoltage[3];
float buffInputCurrent[3];
float buffSysVoltage[3];
float buffBatVoltage[3];
float buffBatCurrent[3];

LTC4015_SystemStatus logSystemStatus[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logChargerState[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logChargeStatus[5] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LTC4015_ChargerState logLimitAlerts[5]  = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000};


HAL_StatusTypeDef LTC4015_write16(uint8_t Reg, uint16_t Val)
{
	HAL_StatusTypeDef status;
	uint16_t data[2];

	data[0] = Val<<8;
	data[1] = Val;

	status = HAL_I2C_Mem_Write_IT(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, data, 2);
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
	int16_t receivedCurr[1];

	float Die_temp = 0;
	float InputVoltage = 0;
	float SysVoltage = 0;
	float BatVoltage = 0;
	float BatCurrent = 0;
	float InputCurrent = 0;


	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, received, 2, 1000);
	Die_temp = (((received[1]<<8) | received[0])-12010)/45.6;
	buffDie_temp[ctr] = Die_temp;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VIN, 1, received, 2, 1000);
	InputVoltage = ((received[1]<<8) | received[0])*1.648E-3;
	buffInputVoltage[ctr] = InputVoltage;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IIN, 1, receivedCurr, 2, 1000);
	InputCurrent = receivedCurr[0]*(1.46487E-6/LTC4015_RSNSI);
	buffInputCurrent[ctr] = InputCurrent;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VSYS, 1, received, 2, 1000);
	SysVoltage = ((received[1]<<8) | received[0])*1.648E-3;
	buffSysVoltage[ctr] = SysVoltage;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VBAT, 1, received, 2, 1000);
	BatVoltage = (((received[1]<<8) | received[0])*192.264E-6)*3;
	buffBatVoltage[ctr] = BatVoltage;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IBAT, 1, receivedCurr, 2, 1000);
	BatCurrent = receivedCurr[0]*(1.46487E-6/LTC4015_RSNSB);
	buffBatCurrent[ctr] = BatCurrent;

	ctr++;

	if(ctr >= Samples)
	{
		ctr = 0;
		Die_temp     = (buffDie_temp[0] + buffDie_temp[1] + buffDie_temp[2]) / Samples;
		InputVoltage = (buffInputVoltage[0] + buffInputVoltage[1] + buffInputVoltage[2]) / Samples;
		InputCurrent = (buffInputCurrent[0] + buffInputCurrent[1] + buffInputCurrent[2]) / Samples;
		SysVoltage   = (buffSysVoltage[0] + buffSysVoltage[1] + buffSysVoltage[2]) / Samples;
		BatVoltage   = (buffBatVoltage[0] + buffBatVoltage[1] + buffBatVoltage[2]) / Samples;
		BatCurrent   = (buffBatCurrent[0] + buffBatCurrent[1] + buffBatCurrent[2]) / Samples;

		Charger.Power.Die_temp 	   = Die_temp;					// Result in °C
		Charger.Power.InputVoltage = InputVoltage;				// in V
		Charger.Power.SysVoltage   = SysVoltage;				// in V
		Charger.Power.InputCurrent = InputCurrent;				// in A
		Charger.Power.InputPower   = ABS(InputCurrent*InputVoltage);	// Power in Watt
		Charger.Power.BatVoltage   = ABS(BatVoltage);				// in V
		Charger.Power.BatCurrent   = ABS(BatCurrent);				// in A
	}

	return status;
}


void LTC4015_InitValues(void)
{
	Charger.Power.Die_temp 	   = 0;
	Charger.Power.InputVoltage = 0;
	Charger.Power.SysVoltage   = 0;
	Charger.Power.InputCurrent = 0;
	Charger.Power.InputPower   = 0;
	Charger.Power.BatVoltage   = 0;
	Charger.Power.BatCurrent   = 0;
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


