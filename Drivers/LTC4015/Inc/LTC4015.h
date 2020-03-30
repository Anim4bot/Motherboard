#ifndef LTC4015_H_
#define LTC4015_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern I2C_HandleTypeDef hi2c2;

#define LTC4015_I2C_PORT	hi2c2
#define LTC4015_ADDR 		0xD0

#define REG_VBAT_LO_ALERT_LIMIT 		0x01 //R/W 15:0 Battery voltage low alert limit, signed, same format as VBAT (0x3A)
#define REG_REG_VBAT_HI_ALERT_LIMIT 	0x02 //R/W 15:0 Battery voltage high alert limit, signed, same format as VBAT (0x3A)
#define REG_REG_VIN_LO_ALERT_LIMIT 		0x03 //R/W 15:0 Input voltage low alert limit, signed, same format as VIN (0x3B)
#define REG_REG_VIN_HI_ALERT_LIMIT 		0x04 //R/W 15:0 Input voltage high alert limit, signed, same format as VIN (0x3B)
#define REG_REG_VSYS_LO_ALERT_LIMIT 	0x05 //R/W 15:0 Output voltage low alert limit, signed, same format as VSYS (0x3C)
#define REG_VSYS_HI_ALERT_LIMIT 		0x06 //R/W 15:0 Output voltage high alert limit, signed, same format as VSYS (0x3C)
#define REG_IIN_HI_ALERT_LIMIT 			0x07 //R/W 15:0 Input current high alert limit, signed, same format as IIN (0x3D)
#define REG_IBAT_LO_ALERT_LIMIT 		0x08 //R/W 15:0 Charge current low alert limit, signed, same format as IBAT (0x3E)
#define REG_DIE_TEMP_HI_ALERT_LIMIT 	0x09 //R/W 15:0 Die temperature high alert limit, signed, same format as DIE_TEMP (0x3F)
#define REG_BSR_HI_ALERT_LIMIT 			0x0A //R/W 15:0 Battery series resistance high alert limit, signed, same format as BSR (0x41)
#define REG_NTC_RATIO_HI_ALERT_LIMIT	0x0B //R/W 15:0 Thermistor ratio high (cold battery) alert limit, signed, same format as NTC_RATIO (0x40)
#define REG_NTC_RATIO_LO_ALERT_LIMIT 	0x0C //R/W 15:0 Thermistor ratio low (hot battery) alert limit, signed, same format as NTC_RATIO (0x40)
#define REG_EN_LIMIT_ALERTS 			0x0D //R/W 15:0 Enable limit monitoring and alert notification via SMBALERT en_meas_sys_valid_alert 0x0D R/W 15 enable meas_sys_valid_alert (0x36)
#define REG_EN_CHARGER_STATE_ALERTS 	0x0E //R/W 15:0 Enable charger state alert notification via SMBALERT
#define REG_EN_CHARGE_STATUS_ALERTS 	0x0F //R/W 15:0 Enable charge status alert notification via SMBALERT
#define REG_QCOUNT_LO_ALERT_LIMIT 		0x10 //R/W 15:0 Coulomb counter QCOUNT low alert limit, same format as QCOUNT (0x13)
#define REG_QCOUNT_HI_ALERT_LIMIT 		0x11 //R/W 15:0 Coulomb counter QCOUNT high alert limit, same format as QCOUNT (0x13)
#define REG_QCOUNT_PRESCALE_FACTOR 		0x12 //R/W 15:0 Coulomb counter prescale factor
#define REG_QCOUNT 						0x13 //R/W 15:0 Coulomb counter value
#define REG_CONFIG_BITS 				0x14 //R/W 15:0 Configuration Settings 0x0000
#define REG_IIN_LIMIT_SETTING 			0x15 //R/W 5:0 Input current limit setting = (IIN_LIMIT_SETTING + 1) • 500µV / RSNSI
#define REG_VIN_UVCL_SETTING 			0x16 //R/W 7:0 UVCLFB input undervoltage limit = (VIN_UVCL_SETTING + 1) • 4.6875mV
#define REG_ARM_SHIP_MODE 				0x19 //R/W 15:0 Write 0x534D to arm ship mode. Once armed, ship mode cannot be disarmed
#define REG_ICHARGE_TARGET 				0x1A //R/W2 4:0 Maximum charge current target = (ICHARGE_TARGET + 1) • 1mV/RSNSB
#define REG_VCHARGE_SETTING 			0x1B //R/W2 5:0 Charge voltage target. See detailed description for equations.
#define REG_C_OVER_X_THRESHOLD 			0x1C //R/W2 15:0 Two’s complement Low IBAT threshold for C/x termination
#define REG_MAX_CV_TIME 				0x1D //R/W2 15:0 Time in seconds with battery charger in the CV state before timer termination occurs (lithium chemistries only)
#define REG_MAX_CHARGE_TIME 			0x1E //R/W2 15:0 Time in seconds before a max_charge_time fault is declared. Set to zero to disable max_charge_time fault
#define REG_JEITA_T1 					0x1F //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 2 and 1 (off) 0x3F00
#define REG_JEITA_T2 					0x20 //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 3 and 2 0x372A
#define REG_JEITA_T3	 				0x21 //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 4 and 3 0x1F27
#define REG_JEITA_T4 					0x22 //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 5 and 4 0x1BCC
#define REG_JEITA_T5 					0x23 //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 6 and 5 0x18B9
#define REG_JEITA_T6 					0x24 //R/W2 15:0 Value of NTC_RATIO for transition between JEITA regions 7 (off) and 6 0x136D
#define REG_VCHARGE_JEITA_6_5 			0x25 //R/W2 9:0 VCHARGE values for JEITA temperature regions 6 and 5 See Note 1
#define REG_VCHARGE_JEITA_4_3_2 		0x26 //R/W2 14:0 VCHARGE values for JEITA temperature regions 4, 3, and 2 See Note 1
#define REG_ICHARGE_JEITA_6_5 			0x27 //R/W2 9:0 ICHARGE_TARGET values for JEITA temperature regions 6 and 5 0x01EF
#define REG_ICHARGE_JEITA_4_3_2	 		0x28 //R/W2 14:0 ICHARGE_TARGET value for JEITA temperature regions 4, 3, and 2 0x7FEF
#define REG_CHARGER_CONFIG_BITS 		0x29 //R/W2 2:0 Battery charger configuration settings, bits 15:3 are reserved. See Note 1
#define REG_VABSORB_DELTA				0x2A //R/W2 5:0 LiFePO4/lead-acid absorb voltage adder, bits 15:6 are reserved. See Note 1
#define REG_MAX_ABSORB_TIME 			0x2B //R/W2 15:0 Maximum time for LiFePO4/lead-acid absorb charge See Note 1
#define REG_VEQUALIZE_DELTA 			0x2C //R/W2 5:0 Lead-acid equalize charge voltage adder, bits 15:6 are reserved. 0x002A
#define REG_EQUALIZE_TIME 				0x2D //R/W2 15:0 Lead-acid equalization time 0x0E10
#define REG_LIFEP04_RECHARGE_THRESHOLD 	0x2E //R/W 15:0 LiFeP04 recharge threshold 0x4410

#define REG_MAX_CHARGE_TIMER 		0x30 //R 15:0 For lithium chemistries, indicates the time (in sec) that the battery has been charging 64
#define REG_CV_TIMER 				0x31 //R 15:0 For lithium chemistries, indicates the time (in sec) that the battery has been in constant-voltage regulation 64
#define REG_ABSORB_TIMER 			0x32 //R 15:0 For LiFePO4 and lead-acid batteries, indicates the time (in sec) that the battery has been in absorb phase 64
#define REG_EQUALIZE_TIMER 			0x33 //R 15:0 For lead-acid batteries, indicates the time (in sec) that the battery has been in EQUALIZE phase
#define REG_CHARGER_STATE			0x34 //R 15:0 Real time battery charger state indicator. Individual bits are mutually exclusive. Bits 15:11 are reserved.
#define REG_CHARGE_STATUS 			0x35 //R 15:0 Charge status indicator. Individual bits are mutually exclusive. Only active in charging states
#define REG_LIMIT_ALERTS			0x36 //R 15:0 Limit alert register.Individual bits are enabled by EN_LIMIT_ALERTS (0x0D).Writing 0 to any bit clears that alert.Once set, alert bits remain high until cleared or disabled.
#define REG_CHARGER_STATE_ALERTS 	0x37 //R 15:0 Charger state alert register.Individual bits are enabled by EN_CHARGER_STATE_ALERTS (0x0E).Writing 0 to any bit clears that alert. Once set, alert bits remain high until cleared or disabled.
#define REG_CHARGE_STATUS_ALERTS 	0x38 //R 5:0 Alerts that CHARGE_STATUS indicators have occurred Individual bits are enabled by EN_CHARGE_STATUS_ALERTS (0x0F) Writing 0 to any bit clears that alert. Once set, alert bits remain high until cleared or disabled.
#define REG_SYSTEM_STATUS 			0x39 //R 15:0 Real time system status indicator bits
#define REG_VBAT 					0x3A //R 15:0 Two’s complement ADC measurement result for the BATSENS pin.VBATSENS/cellcount = [VBAT] • 192.264µV for lithium chemistries.VBATSENS/cellcount = [VBAT] • 128.176µV for lead-acid.
#define REG_VIN 					0x3B //R 15:0 Two’s complement ADC measurement result for VIN.VVIN = [VIN] • 1.648mV
#define REG_VSYS 					0x3C //R 15:0 Two’s complement ADC measurement result for VSYS. VSYS = [VSYS] • 1.648mV
#define REG_IBAT 					0x3D //R 15:0 Two’s complement ADC measurement result for (VCSP – VCSN). Charge current (into the battery) is represented as a positive number. Battery current = [IBAT] • 1.46487µV/RSNSB
#define REG_IIN 					0x3E //R 15:0 Two’s complement ADC measurement result for (VCLP – VCLN). Input current = [IIN] • 1.46487µV/RSNSI
#define REG_DIE_TEMP				0x3F //R 15:0 Two’s complement ADC measurement result for die temperature. Temperature = (DIE_TEMP – 12010)/45.6°C
#define REG_NTC_RATIO 				0x40 //R 15:0 Two’s complement ADC measurement result for NTC thermistor ratio. RNTC = NTC_RATIO • RNTCBIAS/(21845.0 – NTC_RATIO)
#define REG_BSR 					0x41 //R 15:0 Calculated battery series resistance.For lithium chemistries, series resistance/cellcount = BSR • RSNSB/500.0 For lead-acid chemistries, series resistance/cellcount = BSR • RSNSB/750.0
#define REG_JEITA_REGION 			0x42 //R 2:0 JEITA temperature region of the NTC thermistor (Li Only). Active only when EN_JEITA=1
#define REG_CHEM_CELLS 				0x43 //R 11:0 Readout of CHEM and CELLS pin settings 70
#define REG_ICHARGE_DAC 			0x44 //R 4:0 Charge current control DAC control bits
#define REG_VCHARGE_DAC 			0x45 //R 5:0 Charge voltage control DAC control bits
#define REG_IIN_LIMIT_DAC			0x46 //R 5:0 Input current limit control DAC control word
#define REG_VBAT_FILT 				0x47 //R 15:0 Digitally filtered two’s complement ADC measurement result for battery voltage
#define REG_ICHARGE_BSR 			0x48 //R 15:0 This 16-bit two's complement word is the value of IBAT (0x3D) used in calculating BSR.
#define REG_MEAS_SYS_VALID 			0x4A //R 0 Measurement valid bit, bit 0 is a 1 when the telemetry(ADC) system is ready


//EN_LIMIT_ALERTS
#define en_meas_sys_valid_aler	0x8000	//R/W 15 enable meas_sys_valid_alert (0x36)
#define en_qcount_low_alert  	0x2000 	//R/W 13 enable qcount_low_alert (0x36)
#define en_qcount_high_alert 	0x1000 	//R/W 12 enable qcount_high_alert (0x36)
#define en_vbat_lo_alert 	 	0x0800 	//R/W 11 enable vbat_lo_alert (0x36)
#define en_vbat_hi_alert 	 	0x0400 	//R/W 10 enable vbat_hi_alert (0x36)
#define en_vin_lo_alert 	 	0x0200 	//R/W 9 enable vin_lo_alert (0x36)
#define en_vin_hi_alert 		0x0100 	//R/W 8 enable vin_hi_alert (0x36)
#define en_vsys_lo_alert 		0x0080 	//R/W 7 enable vsys_lo_alert (0x36)
#define en_vsys_hi_alert		0x0040 	//R/W 6 enable vsys_hi_alert (0x36)
#define en_iin_hi_alert 		0x0020 	//R/W 5 enable iin_hi_alert (0x36)
#define en_ibat_lo_alert 		0x0010 	//R/W 4 enable ibat_lo_alert (0x36)
#define en_die_temp_hi_alert 	0x0008 	//R/W 3 enable_die_temp_hi_alert (0x36)
#define en_bsr_hi_alert 		0x0004 	//R/W 2 enable bsr_hi_alert (0x36)
#define en_ntc_ratio_hi_alert 	0x0002 	//R/W 1 enable ntc_ratio_hi alert (cold battery; 0x36)
#define en_ntc_ratio_lo_alert 	0x0001 	//R/W 0 enable ntc_ratio_lo_alert (hot battery; 0x36)

//EN_CHARGER_STATE_ALERTS
#define en_equalize_charge_alert 		0x0400 //R/W 10 enable lead-acid equalize_charge_ alert (0x37)
#define en_absorb_charge_alert 			0x0200 //R/W 9 enable absorb_charge_alert (0x37)
#define en_charger_suspended_alert 		0x0100 //R/W 8 enable charger_suspended_alert (0x37)
#define en_precharge_alert 				0x0080 //R/W 7 enable precharge_alert (0x37)
#define en_cc_cv_charge_alert 			0x0040 //R/W 6 enable cc_cv_charge_alert (0x37)
#define en_ntc_pause_alert 				0x0020 //R/W 5 enable ntc_pause_alert (0x37)
#define en_timer_term_alert 			0x0010 //R/W 4 enable timer_term_alert (0x37)
#define en_c_over_x_term_alert			0x0008 //R/W 3 enable c_over_x_term alert (0x37)
#define en_max_charge_time_fault_alert 	0x0004 //R/W 2 enable max_charge_time_fault alert (0x37)
#define en_bat_missing_fault_alert 		0x0002 //R/W 1 enable bat_missing_fault alert (0x37)
#define en_bat_short_fault_alert 		0x0001 //R/W 0 enable bat_short_fault alert (0x37)

//EN_CHARGE_STATUS_ALERTS
#define en_vin_uvcl_active_alert 	0x0008 //R/W 3 enable vin_uvcl_active_alert (VIN undervoltage current limit; 0x38)
#define en_iin_limit_active_alert 	0x0004 //R/W 2 enable iin_limit_active_alert (IIN current limit; 0x38)
#define en_constant_current_alert 	0x0002 //R/W 1 enable constant_current_alert (0x38)
#define en_constant_voltage_alert 	0x0001 //R/W 0 enable constant_voltage_alert (0x38)

//CONFIG_BITS
#define suspend_charger 		0x0100 //R/W 8 suspend battery charger operation 0
#define run_bsr 				0x0020 //R/W 5 perform a battery series resistance measurement
#define force_meas_sys_on 		0x0010 //R/W 4 force measurement system to operate
#define mppt_en_i2c 			0x0008 //R/W 3 enable maximum power point tracking
#define en_qcount 				0x0004 //R/W 2 enable coulomb counter

//CHARGER_CONFIG_BITS
#define en_c_over_x_term 		0x0004	//2 enable C/x termination See Note 1
#define en_lead_acid_temp_comp 	0x0002	//1 enable lead-acid charge voltage temperature compensation See Note 1
#define en_jeita 				0x0001	//0 enable jeita temperature profile



typedef enum
{
	equalize_charge 		= 0x0400, 	// 0x0400	//10 indicates battery charger is in lead-acid equalization charge state
	absorb_charge 			= 0x0200, 	// 0x0200	//9 indicates battery charger is in absorb charge state
	charger_suspended		= 0x0100, 	// 0x0100	//8 indicates battery charger is in charger suspended state
	precharge				= 0x0080, 	// 0x0080	//7 indicates battery charger is in precondition charge state 64
	cc_cv_charge			= 0x0040, 	// 0x0040	//6 indicates battery charger is in constant-current constant-voltage state
	ntc_pause				= 0x0020, 	// 0x0020	//5 indicates battery charger is in thermistor pause state
	timer_term				= 0x0010, 	// 0x0010	//4 indicates battery charger is in timer termination state
	c_over_x_term			= 0x0008, 	// 0x0008	//3 indicates battery charger is in C/x termination state
	max_charge_time_fault	= 0x0004, 	// 0x0004	//2 indicates battery charger is in max_charge_time_fault state
	bat_missing_fault		= 0x0002, 	// 0x0002	//1 indicates battery charger is in missing battery fault state
	bat_short_fault			= 0x0001	// 0x0001	//0 indicates battery charger is in shorted battery fault state
}
LTC4015_ChargerState;


typedef enum
{
	vin_uvcl_active		= 0x0008, 	//3 indicates the input undervoltage control loop is actively controlling power delivery based on VIN_UVCL_SETTING (0x16) 65
	iin_limit_active	= 0x0004, 	//2 indicates the input current limit control loop is actively controlling power delivery based on IIN_LIMIT_DAC (0x46) 65
	constant_current	= 0x0002, 	//1 indicates the charge current control loop is actively controlling power delivery based on ICHARGE_DAC (0x44) 65
	constant_voltage	= 0x0001 	//0 indicates the battery voltage control loop is actively controlling power delivery based on VCHARGE_DAC (0x45)
}
LTC4015_ChargeStatus;


typedef enum
{
	meas_sys_valid_alert	= 0x8000, 	// 0x8000	//15 indicates that measurement system results have become valid.
	qcount_lo_alert			= 0x2000, 	// 0x2000	//13 indicates QCOUNT has fallen below QCOUNT_LO_ALERT_LIMIT (0x10)
	qcount_hi_alert			= 0x1000, 	// 0x1000	//12 indicates QCOUNT has exceeded QCOUNT_HI_ALERT_LIMIT (0x11)
	vbat_lo_alert			= 0x0800,	// 0x0800	//11 indicates VBAT has fallen below VBAT_LO_ALERT_LIMIT (0x01)
	vbat_hi_alert			= 0x0400,	// 0x0400	//10 indicates VBAT has exceeded VBAT_HI_ALERT_LIMIT (0x02)
	vin_lo_alert			= 0x0200,	// 0x0200	//9 indicates VIN has fallen below VIN_LO_ALERT_LIMIT (0x03)
	vin_hi_alert			= 0x0100,	// 0x0100 	//8 indicates VIN has exceeded VIN_HI_ALERT_LIMIT (0x04)
	vsys_lo_alert			= 0x0080,	// 0x0080 	//7 indicates VSYS has fallen below VSYS_LO_ALERT_LIMIT (0x05)
	vsys_hi_alert			= 0x0040, 	// 0x0040	//6 indicates VSYS has exceeded VIN_HI_ALERT_LIMIT (0x06)
	iin_hi_alert			= 0x0020, 	// 0x0020	//5 indicates IIN has exceeded IIN_HI_ALERT_LIMIT (0x07)
	ibat_lo_alert			= 0x0010, 	// 0x0010	//4 indicates IBAT has fallen below IBAT_LO_ALERT_LIMIT (0x08)
	die_temp_hi_alert		= 0x0008, 	// 0x0008	//3 indicates DIE_TEMP has exceeded DIE_TEMP_HI_ALERT_LIMIT (0x09)
	bsr_hi_alert			= 0x0004, 	// 0x0004	//2 indicates BSR has exceeded BSR_HI_ALERT_LIMIT (0x0A)
	ntc_ratio_hi_alert		= 0x0002,	// 0x0002 	//1 indicates NTC_RATIO has exceeded NTC_RATIO_HI_ALERT_LIMIT(cold battery; 0x0B)
	ntc_ratio_lo_alert		= 0x0001 	// 0x0001	//0 indicates NTC_RATIO has exceeded NTC_RATIO_LO_ALERT_LIMIT(hot battery; 0x0C)
}
LTC4015_LimitAlerts;


typedef enum
{
	equalize_charge_alert		= 0x0400,	// 0x0400	//10 alert indicates charger has entered lead-acid equalize_charge state (0x34)
	absorb_charge_alert			= 0x0200,	// 0x0200	//9 alert indicates charger has entered absorb_charge state (0x34)
	charger_suspended_alert		= 0x0100,	// 0x0100	//8 alert indicates charger has entered charger_suspended(off) state (0x34)
	precharge_alert				= 0x0080,	// 0x0080	//7 alert indicates charger has entered precharge charge state (0x34)
	cc_cv_charge_alert			= 0x0040,	// 0x0040	//6 alert indicates charger has entered cc_cv_charge state (constant-current constant-voltage; 0x34)
	ntc_pause_alert				= 0x0020,	// 0x0020	//5 alert indicates charger has entered ntc_pause state (0x34)
	timer_term_alert			= 0x0010,	// 0x0010	//4 alert indicates charger has entered timer_term state (0x34)
	c_over_x_term_alert			= 0x0008,	// 0x0008	//3 alert indicates charger has entered c_over_x term state (C/x termination; 0x34)
	max_charge_time_fault_alert	= 0x0004,	// 0x0004	//2 alert indicates charger has entered max_charge_time_fault state (0x34)
	bat_missing_fault_alert		= 0x0002,	// 0x0002	//1 alert indicates charger has entered bat_missing_fault state (0x34)
	bat_short_fault_alert		= 0x0001 	// 0x0001	//0 alert indicates charger has entered bat_short_fault state (0x34)
}
LTC4015_ChargeStateAlerts;


typedef enum
{
	vin_uvcl_active_alert	= 0x0008, 	// 0x0008	//3 alert indicates vin_uvcl_active state entered (VIN undervoltage currentlimit, 0x35)
	iin_limit_active_alert	= 0x0004,	// 0x0004	//2 alert indicates iin_limit_active state entered (VIN current limit; 0x35)
	constant_current_alert	= 0x0002, 	// 0x0002	//1 alert indicates constant_current state entered (0x35)
	constant_voltage_alert	= 0x0001 	// 0x0001	//0 alert indicates constant_voltage state entered (0x35)
}
LTC4015_ChargeStatusAlerts;


typedef enum
{
	charger_enabled		= 0x2000, 	// 0x2000	//13 indicates that the battery charger is active
	mppt_en_pin			= 0x0800, 	// 0x0800	//11 indicates the mppt_en pin is set to enable maximum power point tracking
	equalize_req		= 0x0400,	// 0x0400	//10 indicates a rising edge has been detected at the EQ pin, and an lead-acid equalize charge is queued
	drvcc_good			= 0x0200, 	// 0x0200	//9 indicates DRVCC voltage is above switching charger undervoltage lockout level (4.3V typical)
	cell_count_error	= 0x0100, 	// 0x0080	//8 indicates an invalid combination of CELLS pin settings
	ok_to_charge		= 0x0040, 	// 0x0040	//6 indicates all system conditions are met to allow battery charger operation
	no_rt				= 0x0020, 	// 0x0020	//5 indicates no resistor has been detected at the rt pin
	thermal_shutdown	= 0x0010, 	// 0x0010	//4 indicates die temperature is greater than thermal shutdown level (160°C typical)
	vin_ovlo			= 0x0008, 	// 0x0008	//3 indicates vin voltage is greater than overvoltage lockout level (38.6V typical)
	vin_gt_vbat			= 0x0004, 	// 0x0004	//2 indicates vin voltage is sufficiently greater than batsens for switching charger operation (200mV typical)
	intvcc_gt_4p3		= 0x0002, 	// 0x0002	//1 indicates INTVCC voltage is above switching charger undervoltage lockout level (4.3V typ)
	intvcc_gt_2p8v		= 0x0001 	// 0x0001	//0 indicates INTVCC voltage is greater than measurement system lockout level (2.8V typical)
}
LTC4015_SystemStatus;





#endif /* LTC4015_H_ */
