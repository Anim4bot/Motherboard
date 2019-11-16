#ifndef INC_PLATFORMTYPE_H_
#define INC_PLATFORMTYPE_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


#define ON  	1
#define HIGH 	1
#define TRUE	1

#define OFF 	0
#define LOW  	0
#define FALSE	0

#define ABS(x)      ((x) < 0 ? -(x) : (x))
#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))

#define PI	(3.141592)


#define PWM_BUZZ	TIM2->CCR3
#define PWM_FAN		TIM2->CCR2
#define PWM_LED_PI	TIM1->CCR3

#define EAR_L_PULSE  	TIM3->CCR1
#define EAR_R_PULSE 	TIM3->CCR2

#define HEAD_PITCH_PULSE  	TIM4->CCR3
#define HEAD_YAW_PULSE   	TIM4->CCR2
#define HOOD_PULSE  	  	TIM4->CCR4


/* VISU PARAM ------------------------------------------------*/
#define set_LED_ACT(x) if(x) \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

#define set_LED_ERR(x) if(x) \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

#define set_LED_B0(x) if(x) \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);



/* POWER PARAM -----------------------------------------------*/
#define set_MAIN_SWITCH(x) if(x) \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);

#define set_PSU_DRS0101(x) if(x) \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

#define set_PSU_5V(x) if(x) \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

#define set_PSU_3V3(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);


#define set_PWR_SERVO_NECK(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

#define set_PWR_SERVO_EARS(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

#define set_PWR_SERVO_HOOD(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);



/* OLEDS PARAM -----------------------------------------------*/
#define set_EYES_RST(x) if(x) \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); \
else \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

#define set_OLED_SYS(x) if(x) \
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

#define set_OLED_SYS_CS(x) if(x) \
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);



/* BLUETOOTH PARAM -------------------------------------------*/
#define set_BT_CS(x) if(x) \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

#define set_BT_RST(x) if(x) \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);



/* RPI PARAM -------------------------------------------*/
#define set_RPI_PWR(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

#define set_PI_SIG_OFF(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);

#define set_PI_SIG_ON(x) if(x) \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); \
else \
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);




#define RPI_SHTDWN_MAX_TIMEOUT	5
#define RPI_SHTDWN_TIME 		15000
#define I2C_MAX_TIMEOUT 		250



typedef enum
{
	Ready   = 0x00U,
	Error   = 0x01U,
	Busy	= 0x02U,
	Timeout = 0x03U
}
DeviceStatus_enum;


typedef enum
{
	I2C2_Busy  = 0x00U,
	I2C2_Free  = 0x01U,
	I2C2_Undef = 0x02U
}
I2C2_Bus_enum;

typedef struct
{
	GPIO_PinState SW1;
	GPIO_PinState SW_HOOD;
	GPIO_PinState SW_PI_BOOT;
	GPIO_PinState RPI_RUNNING;
	GPIO_PinState SIG_HOOD1;
	GPIO_PinState SIG_HOOD2;
	GPIO_PinState SIG_SYS;
	GPIO_PinState SIG_DOCK;
	GPIO_PinState CHARGER_FAULT;
}
Input_st;


typedef struct
{
	uint8_t Up;
	uint8_t Down;
	uint8_t Left;
	uint8_t Right;
}
Gesture_st;


typedef struct
{
	float Pitch;
	float Roll;
	float TempIMU;
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


typedef enum
{
	Standby  = 0,
	Normal   = 1,
	Debug    = 2,
	Charging = 3,
	PwrMngt  = 4,
	Sensors  = 5
}
Flex_Oled_Menu_em;


typedef enum
{
	Closed  = 0,
	Opened   = 1,
	Undef    = 2
}
HoodStatus_em;



#endif /* INC_PLATFORMTYPE_H_ */
