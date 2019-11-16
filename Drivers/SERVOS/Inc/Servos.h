#ifndef SERVOS_H_
#define SERVOS_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define PITCH_MIN_PULSE		25
#define PITCH_CTR_PULSE		75
#define PITCH_MAX_PULSE		125

#define YAW_MIN_PULSE		25
#define YAW_CTR_PULSE		75
#define YAW_MAX_PULSE		125

#define INIT_TIME_MS		500

#endif /* SERVOS_H_ */
