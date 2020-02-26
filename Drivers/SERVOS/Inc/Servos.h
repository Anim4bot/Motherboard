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

/*
#define PITCH_MIN_PULSE		25
#define PITCH_CTR_PULSE		75
#define PITCH_MAX_PULSE		125

#define YAW_MIN_PULSE		25
#define YAW_CTR_PULSE		75
#define YAW_MAX_PULSE		125

*/


#endif /* SERVOS_H_ */
