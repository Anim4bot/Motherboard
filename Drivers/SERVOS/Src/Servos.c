#include "Servos.h"



void NeckServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 400);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);

	set_PWR_SERVO_NECK(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_NECK(OFF);
}


void EarsServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 75);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 75);

	set_PWR_SERVO_EARS(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_EARS(OFF);
}


void HoodServos_Init(void)
{
	uint16_t i;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 75);

	set_PWR_SERVO_HOOD(ON);
	for(i = 0; i<= 75; i++)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, i);
		HAL_Delay(15);
	}
	set_PWR_SERVO_HOOD(OFF);
}


void Hood_set(uint8_t position)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	if(position == OPEN)
	{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 75);
	set_PWR_SERVO_HOOD(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_HOOD(OFF);
	}
	else if(position == CLOSE)
	{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 100);
	set_PWR_SERVO_HOOD(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_HOOD(OFF);
	}
	else
	{
		//error
	}

}



void Head_Init(uint8_t speed)
{
	uint16_t i;
	uint16_t temp;

	/*
	set_PWR_SERVO_NECK(ON);
	for(i = ; i<= ; i++)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, i);
		HAL_Delay(15);
	}
	set_PWR_SERVO_NECK(OFF);
	*/
}

void Head_ParkPos(uint8_t speed)
{
	uint16_t i;
	uint16_t temp;

	/*
	set_PWR_SERVO_NECK(ON);
	for(i = ; i<= ; i++)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, i);
		HAL_Delay(speed);
	}
	set_PWR_SERVO_NECK(OFF);
	*/
}


void Head_setPosition(uint8_t pitch, uint8_t yaw, uint8_t speed)
{
	uint16_t i;
	uint16_t temp;

	/*
	set_PWR_SERVO_NECK(ON);
	for(i = ; i<= ; i++)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, i);
		HAL_Delay(15);
	}
	set_PWR_SERVO_NECK(OFF);
	*/
}
