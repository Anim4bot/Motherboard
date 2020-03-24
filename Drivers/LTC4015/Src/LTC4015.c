#include "LTC4015.h"
#include "LTC4015_formats.h"
//#include "LTC4015_reg_defs.h"

Charger_st Charger;

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
	uint8_t data[3];
	uint16_t RegConfigBits = 0b0000000000010100;

	data[0] = REG_CONFIG_BITS;
	data[2] = 0b00000000;
	data[1] = 0b00010100;
	status = HAL_I2C_Master_Transmit(&LTC4015_I2C_PORT, LTC4015_ADDR, &data, 3, 50);

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, (uint16_t*)RegConfigBits, 2, 1000);

	//status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, (uint16_t*)RegConfigBits, 2, 50);

}


LTC4015_GlobalState LTC4015_Mngt(void)
{

}


void LTC4015_GetPowerVal(void)
{
	HAL_StatusTypeDef status;

	uint8_t received[2];
	float Die_temp = 0xFFFF;
	float InputVoltage = 0xFFFF;
	float SysVoltage = 0xFFFF;
	float BatVoltage = 0xFFFF;
	float BatCurrent = 0xFFFF;
	float SysCurrent = 0xFFFF;
	uint16_t ChargeTime = 0xFFFF;

	uint16_t RegConfigBits1 = 0x0014;
	uint16_t RegConfigBits2 = 0x0000;

	uint8_t data[3];

	data[0] = REG_CONFIG_BITS;
	data[2] = 0b00000000;
	data[1] = 0b00010100;
	status = HAL_I2C_Master_Transmit(&LTC4015_I2C_PORT, LTC4015_ADDR, &data, 3, 1000);

	//status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, 0x0000, 2, 1000);

	status = HAL_I2C_Mem_Write(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CONFIG_BITS, 1, 0x0014, 2, 1000);

	HAL_Delay(10);
	osDelay(10);

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_DIE_TEMP, 1, (uint8_t*)received, 2, 1000);
	Die_temp = (((received[1]<<8) | received[0])-12010)/45.6;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VIN, 1, (uint8_t*)received, 2, 1000);
	InputVoltage = ((received[1]<<8) | received[0])*1.648;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IIN, 1, (uint8_t*)received, 2, 1000);
	SysCurrent 	 = (((received[1]<<8) | received[0])*1.46487E-6)/4.0E-3;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VSYS, 1, (uint8_t*)received, 2, 1000);
	SysVoltage 	 = ((received[1]<<8) | received[0])*1.648;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_VBAT, 1, (uint8_t*)received, 2, 1000);
	BatVoltage 	 = (((received[1]<<8) | received[0])*192.264E-6)*3;

	status = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_IBAT, 1, (uint8_t*)received, 2, 1000);
	BatCurrent 	 = (((received[1]<<8) | received[0])*1.46487E-6)/20.0E-3;


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
	uint16_t tempstatus = 0x0000;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_SYSTEM_STATUS, 1, (uint8_t*)received, 2, 1000);
	*status = ((received[1]<<8) | received[0]);

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
		case intvcc_gt_4p3:
			*status = intvcc_gt_4p3;
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
	uint16_t tempstate = 0x0000;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGER_STATE, 1, (uint8_t*)received, 2, 1000);
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

	Charger.ChargerState = *state;
}


void LTC4015_GetChargeStatus(LTC4015_ChargeStatus* status)
{
	HAL_StatusTypeDef stat;
	uint8_t received[2];
	uint8_t i=0;
	uint16_t tempstate = 0x0000;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_CHARGE_STATUS, 1, (uint8_t*)received, 2, 1000);
	tempstate = ((received[1]<<8) | received[0]);

	switch (tempstate)
	{
		case vin_uvcl_active:
			*status = vin_uvcl_active;
		break;
		case iin_limit_active:
			*status = iin_limit_active;
		break;
		case constant_current:
			*status = constant_current;
		break;
		case constant_voltage:
			*status = constant_voltage;
		break;
		default:
			*status = NOK;
		break;
	}

	if(*status != logChargeStatus[i])
	{
		logChargerState[i] = *status;
		if(i>9) { i=0; }
		i++;
	}

	Charger.ChargeStatus = *status;
}



void LTC4015_GetLimitAlerts(LTC4015_LimitAlerts* alert)
{
	HAL_StatusTypeDef stat;
	uint8_t received[2];
	uint8_t i=0;
	uint16_t temp = 0x0000;

	stat = HAL_I2C_Mem_Read(&LTC4015_I2C_PORT, LTC4015_ADDR, REG_SYSTEM_STATUS, 1, (uint8_t*)received, 2, 1000);
	temp = ((received[1]<<8) | received[0]);

	switch (temp)
	{
		case meas_sys_valid_alert:
			*alert = meas_sys_valid_alert;
		break;
		case qcount_lo_alert:
			*alert = qcount_lo_alert;
		break;
		case qcount_hi_alert:
			*alert = qcount_hi_alert;
		break;
		case vbat_lo_alert:
			*alert = vbat_lo_alert;
		break;
		case vbat_hi_alert:
			*alert = vbat_hi_alert;
		break;
		case vin_lo_alert:
			*alert = vin_lo_alert;
		break;
		case vin_hi_alert:
			*alert = vin_hi_alert;
		break;
		case vsys_lo_alert:
			*alert = vsys_lo_alert;
		break;
		case vsys_hi_alert:
			*alert = vsys_hi_alert;
		break;
		case iin_hi_alert:
			*alert = iin_hi_alert;
		break;
		case ibat_lo_alert:
			*alert = ibat_lo_alert;
		break;
		break;
		case die_temp_hi_alert:
			*alert = die_temp_hi_alert;
		break;
		case bsr_hi_alert:
			*alert = bsr_hi_alert;
		break;
		case ntc_ratio_hi_alert:
			*alert = ntc_ratio_hi_alert;
		break;
		case ntc_ratio_lo_alert:
			*alert = ntc_ratio_lo_alert;
		break;
		default:
			*alert = NOK;
		break;
	}

	if(*alert != logSystemStatus[i])
	{
		logLimitAlerts[i] = *alert;
		if(i>9) { i=0; }
		i++;
	}
	Charger.LimitAlerts = *alert;
}
