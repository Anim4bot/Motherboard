#include "Servos.h"
#include "Robot.h"



void NeckServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	set_PWR_SERVO_NECK(ON);

	HEAD_PITCH_PULSE	= PitchNeutral;
	HEAD_YAW_PULSE   	= YawNeutral;

	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NECKYAW_INIT_POS);		//Yaw
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NECKPITCH_INIT_POS);		//Pitch

	Head.NeckYaw_pos  = PitchNeutral;
	Head.NeckPith_pos = PitchNeutral;

	osDelay(2000);
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

	//EAR_L_PULSE = EarL_Down;
	//EAR_R_PULSE = EarR_Down;

	Head.EarL_pos = EarL_Down;
	Head.EarR_pos = EarR_Down;
}


void Ears_SetPosition(uint16_t NewPosL, uint16_t NewPosR, AnimSpeed_enum speed)
{
	uint16_t speedConst = 7500;
	uint16_t CurrPosLeft=0, CurrPosRight=0;
	uint32_t l=0, r=0, i=0;

	CurrPosLeft  = Head.EarL_pos;
	CurrPosRight = Head.EarR_pos;

	if( (NewPosL>CurrPosLeft) && ((NewPosR<CurrPosRight)) )			// DOWN - DOWN
	{
		for(l=CurrPosLeft, r=CurrPosRight ; l<=NewPosL && r>=NewPosR ; l++, r--	)
		{
			EAR_L_PULSE = l;
			EAR_R_PULSE = r;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_EARS(ON);
		}
	}
	else if( (NewPosL<CurrPosLeft) && ((NewPosR>CurrPosRight)) )	// UP - UP
	{
		for(l=CurrPosLeft, r=CurrPosRight ; l>=NewPosL && r<=NewPosR ; l--, r++	)
		{
			EAR_L_PULSE = l;
			EAR_R_PULSE = r;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_EARS(ON);
		}
	}

	else if( (NewPosL>CurrPosLeft) && ((NewPosR>CurrPosRight)) )	// DOWN - UP
	{
		for(l=CurrPosLeft, r=CurrPosRight ; l<=NewPosL && r<=NewPosR ; l++, r++	)
		{
			EAR_L_PULSE = l;
			EAR_R_PULSE = r;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_EARS(ON);
		}
	}
	else if( (NewPosL<CurrPosLeft) && ((NewPosR<CurrPosRight)) )	// UP - DOWN
	{
		for(l=CurrPosLeft, r=CurrPosRight ; l>=NewPosL && r>=NewPosR ; l--, r--	)
		{
			EAR_L_PULSE = l;
			EAR_R_PULSE = r;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_EARS(ON);
		}
	}
	else
	{
		EAR_L_PULSE = CurrPosLeft;
		EAR_R_PULSE = CurrPosRight;
	}

	Head.EarL_pos = NewPosL;
	Head.EarR_pos = NewPosR;

	osDelay(750);
	set_PWR_SERVO_EARS(OFF);

}

void Ear_SetPosition(EarSide_enum side, uint16_t NewPos, AnimSpeed_enum speed)
{
	uint16_t ctr=0, i=0;
	uint16_t CurrentPos=0;

	set_PWR_SERVO_EARS(ON);

	if(side == EarLeft)
	{
		CurrentPos = Head.EarL_pos;

		if(NewPos > CurrentPos)
		{
			for(ctr=CurrentPos ; ctr<=NewPos ; ctr++)
			{
				EAR_L_PULSE = ctr;
				for(i=0;i<=(speed*10000);i++) {asm("NOP");}
			}
		}
		else if (NewPos < CurrentPos)
		{
			for(ctr=CurrentPos ; ctr>=NewPos ; ctr--)
			{
				EAR_L_PULSE = ctr;
				for(i=0;i<=(speed*10000);i++) {asm("NOP");}
			}
		}
		else
		{
			asm("NOP");
		}

		Head.EarL_pos = NewPos;
	}
	else
	{
		CurrentPos = Head.EarR_pos;

		if(NewPos > CurrentPos)
		{
			for(ctr=CurrentPos ; ctr>=NewPos ; ctr--)
			{
				EAR_R_PULSE = ctr;
				for(i=0;i<=(speed*10000);i++) {asm("NOP");}
			}
		}
		else if (NewPos < CurrentPos)
		{
			for(ctr=CurrentPos ; ctr<=NewPos ; ctr++)
			{
				EAR_R_PULSE = ctr;
				for(i=0;i<=(speed*10000);i++) {asm("NOP");}
			}
		}
		else
		{
			asm("NOP");
		}

		Head.EarR_pos = NewPos;
	}

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
