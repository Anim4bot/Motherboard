#ifndef SERVOS_H_
#define SERVOS_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define INIT_TIME_MS		500


#define PITCH_MAX_UP    	(PitchNeutral+90)
#define PITCH_MAX_DOWN    	(PitchNeutral-50)
#define YAW_MAX_LEFT 	  	(YawNeutral+300)
#define YAW_MAX_RIGHT 	  	(YawNeutral-300)


#define EAR_L_MIN_POS    	1900
#define EAR_L_MAX_POS    	1100
#define EAR_R_MIN_POS 	  	1100
#define EAR_R_MAX_POS 	  	1900



typedef enum
{
	PitchNeutral = 1553, 	//more = UP
	YawNeutral   = 1255		//more = LeftDir
}
NeckPos_enum;



typedef enum
{
	EarLeft = 0,
	EarRight =1
}
EarSide_enum;



typedef enum
{
	EarL_Up 		= 1100,
	EarL_MidUp 		= 1300,
	EarL_Middle		= 1500,
	EarL_MidDown	= 1700,
	EarL_Down		= 1900,

	EarR_Up 		= 1900,
	EarR_MidUp 		= 1700,
	EarR_Middle		= 1500,
	EarR_MidDown	= 1300,
	EarR_Down		= 1100
}
EarPos_enum;



void NeckServos_Init(void);
void EarsServos_Init(void);
void HoodServos_Init(void);

void Ears_SetPosition(uint16_t NewPosL, uint16_t NewPosR, AnimSpeed_enum speed);

#endif /* SERVOS_H_ */
