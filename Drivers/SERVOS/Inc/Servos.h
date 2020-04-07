#ifndef SERVOS_H_
#define SERVOS_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define INIT_TIME_MS		500

/* Head servos positions after switch ON */
#define EAR_L_INIT_POS    	75
#define EAR_R_INIT_POS 	  	75
#define NECKYAW_INIT_POS  	425
#define NECKPITCH_INIT_POS  500

/* Head servos positions before switch OFF  */
#define EAR_L_PARK_POS    	75
#define EAR_R_PARK_POS 	  	75
#define NECKYAW_PARK_POS  	425
#define NECKPITCH_PARK_POS  500



typedef enum
{
	YawHight   = 1400,
	YawNeutral = 1260, //more = LeftDir
	YawLow	   = 1600,

	PitchNeutral = 1550 //more = UP
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
