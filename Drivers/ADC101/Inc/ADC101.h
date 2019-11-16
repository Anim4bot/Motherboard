#ifndef ADC101_H_
#define ADC101_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern I2C_HandleTypeDef  hi2c2;

#define ADC101_I2C_PORT		hi2c2
#define ADC101_ADDR 		0xA8 //A8




#endif /* ADC101_H_ */
