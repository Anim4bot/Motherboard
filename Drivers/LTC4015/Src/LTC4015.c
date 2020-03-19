#include "LTC4015.h"
#include "LTC4015_formats.h"
//#include "LTC4015_reg_defs.h"

Charger_st Charger;

LTC4015_SystemStatus logSystemStatus[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
LTC4015_ChargerState logChargerState[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


HAL_StatusTypeDef LTC4015_write16(uint8_t Reg, uint16_t Val)
{
	HAL_StatusTypeDef status;
	uint16_t data[2];

	data[0] = Val<<8;
	data[1] = Val;

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, data, 2, 50);
	return status;
}


HAL_StatusTypeDef LTC4015_read16(uint8_t Reg, uint16_t *data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, &Reg, 1, *data, 2, 50);
	return status;
}


void LTC4015_Init(void)
{
	HAL_StatusTypeDef status;
	uint16_t RegConfigBits = 0b0000000000010100;

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, (uint16_t*)RegConfigBits, 2, 50);
	//status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, (uint16_t*)RegConfigBits, 2, 50);

}


LTC4015_GlobalState LTC4015_Mngt(void)
{

}


void LTC4015_GetPowerVal(void)
{
	HAL_StatusTypeDef status;

	uint8_t received[2];
	volatile float Die_temp = 0xFFFF;
	volatile float InputVoltage = 0xFFFF;
	volatile float SysVoltage = 0xFFFF;
	volatile float BatVoltage = 0xFFFF;
	volatile float BatCurrent = 0xFFFF;
	volatile float SysCurrent = 0xFFFF;
	volatile uint16_t ChargeTime = 0xFFFF;

	uint16_t RegConfigBits = 0b0000000000010100;
	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, (uint16_t*)RegConfigBits, 2, 50);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, (uint8_t*)received, 2, 50);
	Die_temp = (((received[1]<<8) | received[0])-12010)/45.6;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VIN, 1, (uint8_t*)received, 2, 50);
	InputVoltage = ((received[1]<<8) | received[0])*1.648;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IIN, 1, (uint8_t*)received, 2, 50);
	SysCurrent 	 = (((received[1]<<8) | received[0])*1.46487E-6)/4.0E-3;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VSYS, 1, (uint8_t*)received, 2, 50);
	SysVoltage 	 = ((received[1]<<8) | received[0])*1.648;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VBAT, 1, (uint8_t*)received, 2, 50);
	BatVoltage 	 = (((received[1]<<8) | received[0])*192.264E-6)*3;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IBAT, 1, (uint8_t*)received, 2, 50);
	BatCurrent 	 = (((received[1]<<8) | received[0])*1.46487E-6)/20.0E-3;

	//status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_MAX_CHARGE_TIMER, 1, (uint8_t*)received, 2, 50);
	//ChargeTime 	 = ((received[1]<<8) | received[0]);

	Charger.Power.Die_temp = Die_temp;								// Result in °C
	Charger.Power.InputVoltage = ABS(InputVoltage/1000.0);			// in V
	Charger.Power.SysVoltage   = ABS(SysVoltage/1000.0);			// in V
	Charger.Power.SysCurrent   = ABS(SysCurrent);					// in A
	Charger.Power.SysPower     = ABS(SysCurrent*SysVoltage/1000.0);	// Power in Watt
	Charger.Power.BatVoltage   = ABS(BatVoltage);					// in V
	Charger.Power.BatCurrent   = ABS(BatCurrent);					// in A

}


void LTC4015_GetSystemStatus(LTC4015_SystemStatus* status)
{
	HAL_StatusTypeDef stat;
	uint8_t received[2];
	uint8_t i=0;
	uint16_t tempstatus = 0xFF;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, (uint8_t*)received, 2, 50);
	tempstatus = ((received[1]<<8) | received[0]);

	switch (tempstatus)
	{
		case charger_enabled:
			*status = charger_enabled;
		break;
		case mppt_en_pin:
			*status = mppt_en_pin;
		break;
		case equalize_req:
			*status = equalize_req;
		break;
		case drvcc_good:
			*status = drvcc_good;
		break;
		case cell_count_error:
			*status = cell_count_error;
		break;
		case ok_to_charge:
			*status = ok_to_charge;
		break;
		case no_rt:
			*status = no_rt;
		break;
		case thermal_shutdown:
			*status = thermal_shutdown;
		break;
		case vin_ovlo:
			*status = vin_ovlo;
		break;
		case vin_gt_vbat:
			*status = vin_gt_vbat;
		break;
		case intvcc_gt_4p3v:
			*status = intvcc_gt_4p3v;
		break;
		break;
		case intvcc_gt_2p8v:
			*status = intvcc_gt_2p8v;
		break;
		default:
			*status = NOK;
		break;
	}

	if(*status != logSystemStatus[i])
	{
		logSystemStatus[i] = *status;
		if(i>9) { i=0; }
		i++;
	}
	Charger.SystemStatus = *status;
}


void LTC4015_GetChargerState(LTC4015_ChargerState* state)
{
	HAL_StatusTypeDef stat;
	uint8_t received[2];
	uint8_t i=0;
	uint16_t tempstate;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, (uint8_t*)received, 2, 50);
	tempstate = ((received[1]<<8) | received[0]);

	switch (tempstate)
	{
		case equalize_charge:
			*state = equalize_charge;
		break;
		case absorb_charge:
			*state = absorb_charge;
		break;
		case charger_suspended:
			*state = charger_suspended;
		break;
		case precharge:
			*state = precharge;
		break;
		case cc_cv_charge:
			*state = cc_cv_charge;
		break;
		case ntc_pause:
			*state = ntc_pause;
		break;
		case timer_term:
			*state = timer_term;
		break;
		case c_over_x_term:
			*state = c_over_x_term;
		break;
		case max_charge_time_fault:
			*state = max_charge_time_fault;
		break;
		case bat_missing_fault:
			*state = bat_missing_fault;
		break;
		case bat_short_fault:
			*state = bat_short_fault;
		break;
		default:
			*state = NOK;
		break;
	}

	if(*state != logChargerState[i])
	{
		logChargerState[i] = *state;
		if(i>9) { i=0; }
		i++;
	}

	Charger.ChargeState = *state;
}
