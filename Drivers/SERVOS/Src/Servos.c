#include "Servos.h"
#include "Robot.h"



void NeckServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	HEAD_PITCH_PULSE	= PitchNeutral;
	HEAD_YAW_PULSE   	= YawNeutral;

	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, NECKPITCH_INIT_POS);		//Pitch
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, NECKYAW_INIT_POS);		//Yaw

	set_PWR_SERVO_NECK(ON);
	HEAD_PITCH_PULSE = PitchNeutral;
	HEAD_YAW_PULSE = YawNeutral;
	osDelay(350);
	set_PWR_SERVO_NECK(OFF);

	Head.NeckPith_pos = PitchNeutral;
	Head.NeckYaw_pos  = YawNeutral;
}


void Head_SetPosition(uint16_t NewPosPitch, uint16_t NewPosYaw, AnimSpeed_enum speed)
{
	uint16_t speedConst = 10000;
	uint16_t CurrPosPitch=0, CurrPosYaw=0;
	uint32_t p=0, y=0, i=0;

	CurrPosPitch  = Head.NeckPith_pos;
	CurrPosYaw 	  = Head.NeckYaw_pos;

	//if(NewPosPitch > PITCH_MAX_UP) 	 {NewPosPitch = PITCH_MAX_UP;}
	//if(NewPosPitch < PITCH_MAX_DOWN) {NewPosPitch = PITCH_MAX_DOWN;}
	if(NewPosYaw > PITCH_MAX_UP) 	 {NewPosYaw = YAW_MAX_LEFT;}
	if(NewPosYaw < YAW_MAX_RIGHT) 	 {NewPosYaw = YAW_MAX_RIGHT;}

	if( (NewPosPitch>CurrPosPitch) && (NewPosYaw<CurrPosYaw) )			// UP - RIGHT
	{
		for(p=CurrPosPitch, y=CurrPosYaw ; p<=NewPosPitch || y>=NewPosYaw ; p++, y--)
		{
			HEAD_PITCH_PULSE = p;
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else if( (NewPosPitch>CurrPosPitch) && (NewPosYaw>CurrPosYaw) )		// UP - LEFT
	{
		for(p=CurrPosPitch, y=CurrPosYaw ; p<=NewPosPitch || y<=NewPosYaw ; p++, y++)
		{
			HEAD_PITCH_PULSE = p;
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else if( (NewPosPitch<CurrPosPitch) && (NewPosYaw<CurrPosYaw) )		// DOWN - RIGHT
	{
		for(p=CurrPosPitch, y=CurrPosYaw ; p>=NewPosPitch || y>=NewPosYaw ; p--, y--)
		{
			HEAD_PITCH_PULSE = p;
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosPitch<CurrPosPitch) && (NewPosYaw>CurrPosYaw) )		// DOWN - LEFT
	{
		for(p=CurrPosPitch, y=CurrPosYaw ; p>=NewPosPitch || y<=NewPosYaw ; p--, y++)
		{
			HEAD_PITCH_PULSE = p;
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosPitch==CurrPosPitch) && (NewPosYaw<CurrPosYaw) )		// RIGHT
	{
		for(y=CurrPosYaw ; y>=NewPosYaw ; y--)
		{
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosPitch==CurrPosPitch) && (NewPosYaw>CurrPosYaw) )		// LEFT
	{
		for(y=CurrPosYaw ; y<=NewPosYaw ; y++)
		{
			HEAD_YAW_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosPitch>CurrPosPitch) && (NewPosYaw==CurrPosYaw) )		// UP
	{
		for (p=CurrPosPitch ; p<=NewPosPitch ; p++)
		{
			HEAD_PITCH_PULSE = p;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosPitch<CurrPosPitch) && (NewPosYaw==CurrPosYaw) )		// DOWN
	{
		for(p=CurrPosPitch ; p>=NewPosPitch ; p--)
		{
			HEAD_PITCH_PULSE = p;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else
	{
		Head.NeckPith_pos = CurrPosPitch;
		Head.NeckYaw_pos  = CurrPosYaw;
	}


	Head.NeckPith_pos = NewPosPitch;
	Head.NeckYaw_pos  = NewPosYaw;

	osDelay(350);
	set_PWR_SERVO_NECK(OFF);

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
