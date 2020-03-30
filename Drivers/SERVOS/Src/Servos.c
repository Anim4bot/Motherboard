#include "Servos.h"
#include "Robot.h"

Head_st Head;

void NeckServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NECKYAW_INIT_POS);		//Yaw
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NECKPITCH_INIT_POS);	//Pitch
	Head.NeckYaw_pos  = NECKYAW_INIT_POS;
	Head.NeckPith_pos = NECKPITCH_INIT_POS;

	set_PWR_SERVO_NECK(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_NECK(OFF);
}


void Head_setPosition(uint8_t pitch, uint8_t yaw, uint8_t speed)
{
	uint16_t i;
	uint16_t temp;

	/*
	 * if current pos > desired pos --> steps to do = current pos - desired pos et..
	 *
	set_PWR_SERVO_NECK(ON);
	for(i = ; i<= ; i++)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, i);
		HAL_Delay(15);
	}
	set_PWR_SERVO_NECK(OFF);
	*/
}


void EarsServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, EAR_L_INIT_POS);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, EAR_R_INIT_POS);
	Head.EarL_pos = EAR_L_INIT_POS;
	Head.EarR_pos = EAR_R_INIT_POS;

	set_PWR_SERVO_EARS(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_EARS(OFF);
}


void HoodServos_Init(void)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 75);

	set_PWR_SERVO_HOOD(ON);
	osDelay(INIT_TIME_MS);
	set_PWR_SERVO_HOOD(OFF);
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
		HAL_Delay(15);
	}
	set_PWR_SERVO_NECK(OFF);
	*/
}
