#ifndef LM75B_H_
#define LM75B_H_


#include "stm32f4xx_hal.h"
#include "PlatformType.h"

extern I2C_HandleTypeDef  hi2c2;

#define LM75B_I2C_PORT		hi2c2
#define LM75B_CHG_ADDR 	 	0x90
#define LM75B_PSU_ADDR 		0x92


#define LM75B_CONF       0x01
#define LM75B_TEMP       0x00
#define LM75B_TOS        0x03
#define LM75B_THYST      0x02

#define LM75_CONF_SHUTDOWN  	0x0
#define LM75_CONF_OS_COMP_INT 	0x1
#define LM75_CONF_OS_POL 		0x2
#define LM75_CONF_OS_F_QUE 		0x3


void LM75B_Init(void);
uint16_t LM75B_CHG_Read16(void);
uint16_t LM75B_PSU_Read16(void);
void LM75B_BAT_GetTemp(float *temp);
void LM75B_BAT_GetTemp(float *temp);

#endif /* LM75B_H_ */
