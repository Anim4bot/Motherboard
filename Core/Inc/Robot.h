#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "DRS0101.h"
#include "SSD1306.h"
#include "SSD1320.h"
#include "APDS9960.h"
#include "LM75B.h"
#include "LTC4015.h"
#include "ADC101.h"
#include "LSM9DS1.h"
#include "Servos.h"
#include "EyeAnimations.h"
#include "Kinematics.h"



/* SENSORS DEFINITIONS
*********************************************/
typedef struct
{
	uint8_t Up;
	uint8_t Down;
	uint8_t Left;
	uint8_t Right;
	uint8_t Luminosity;
}
Gesture_st;

typedef struct
{
	float Pitch;
	float Roll;
}
IMU_st;

typedef struct
{
	IMU_st IMU;
	Gesture_st Gesture;
	uint16_t dist_IR;
	uint16_t Fan_Speed;
	float TempCharger;
	float TempPSU;
	float TempAverage;
}
Sensors_st;
Sensors_st Sensor;



/* HOOD DEFINITIONS
*********************************************/
typedef enum
{
	Close  = 0,
	Open   = 1
}
HoodStatus_em;
HoodStatus_em Hood;


/* HEAD DEFINITIONS
*********************************************/
typedef struct
{
	uint16_t EarL_pos;
	uint16_t EarR_pos;
	uint16_t NeckPith_pos;
	uint16_t NeckPan_pos;
}
Head_st;
Head_st Head;



/* Flex OLED DEFINITIONS
*********************************************/
typedef enum
{
	Battery  = 0,
	Sensors  = 1,
	SPI_1	 = 2,
	Modes    = 3

}
Flex_Oled_Menu_em;

typedef struct
{
	Flex_Oled_Menu_em OLED_Menu;
	uint8_t OLED_Contrast;
	uint8_t LogoDelay;

}
OLED_st;
OLED_st OLED;



/* CHARGER DEFINITIONS
*********************************************/
typedef struct
{
	float Die_temp;
	float InputVoltage;
	float SysVoltage;
	float InputCurrent;
	float InputPower;
	float BatVoltage;
	float BatCurrent;
	float BatPower;
}
LTC4015_Power_st;


typedef enum
{
	Discharging,
	Charging,
	Charged,
	Alert,
	TempHigh,
	Pause
}
LTC4015_GlobalState;

typedef struct
{
	LTC4015_Power_st Power;
	LTC4015_SystemStatus SystemStatus;
	LTC4015_ChargerState ChargerState;
	LTC4015_ChargeStatus ChargeStatus;
	LTC4015_LimitAlerts LimitAlerts;
	LTC4015_ChargeStatusAlerts ChargeStatusAlerts;
	LTC4015_ChargeStateAlerts ChargeStateAlerts;
	LTC4015_GlobalState GlobalState;
}
Charger_st;
Charger_st Charger;



/* KINEMATICS DEFINITIONS
*********************************************/
typedef struct
{
	  int   Xspeed;
	  int   Yspeed;
	  int   Rspeed;
	  float TransX;
	  float TransY;
	  float TransZ;
	  float RotX;
	  float RotY;
	  float RotZ;
}
BodyCmd_st;

typedef struct
{
	  float Init_X;
	  float Init_Y;
	  float Init_Z;
	  float Pos_X;
	  float Pos_Y;
	  float Pos_Z;
}
LegPos_st;



/* RASPBERRY PI STATE
*********************************************/
typedef enum
{
	RPi_OFF = 0,
	RPi_ON  = 1
}
RPiState_enum;


/* ROBOT GLOBAL STRUCTURE
*********************************************/
typedef struct
{
	Eyes_st Eyes;
	Head_st Head;
	OLED_st OLED;
	HoodStatus_em Hood;
	Sensors_st Sensors;
	Charger_st Charger;
	BodyCmd_st BodyCmd;
	LegPos_st LegPos;
	RPiState_enum RPiState;

}
Robot_st;
Robot_st Robot;



#endif /* INC_ROBOT_H_ */
