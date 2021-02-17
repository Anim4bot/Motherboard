#include "Servos.h"
#include "Robot.h"

uint16_t NewPosTilt = TiltNeutral;
uint16_t NewPosPan = PanNeutral;

void NeckServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	//Pan
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	//Tilt

	HEAD_TILT_PULSE	= TiltNeutral;
	HEAD_PAN_PULSE   	= PanNeutral;

	set_PWR_SERVO_NECK(ON);
	HEAD_TILT_PULSE = TiltNeutral;
	HEAD_PAN_PULSE = PanNeutral;
	osDelay(350);
	set_PWR_SERVO_NECK(OFF);

	Head.NeckPith_pos = TiltNeutral;
	Head.NeckPan_pos  = PanNeutral;
}


void Head_SetPosition(uint16_t NewPosTilt, uint16_t NewPosPan, AnimSpeed_enum speed)
{
	uint16_t speedConst = 10000;
	uint16_t CurrPosTilt=0, CurrPosPan=0;
	uint32_t p=0, y=0, i=0;

	CurrPosTilt  = Head.NeckPith_pos;
	CurrPosPan 	  = Head.NeckPan_pos;

	if(NewPosTilt > TILT_MAX_UP) 	 {NewPosTilt = TILT_MAX_UP;}
	if(NewPosTilt < TILT_MAX_DOWN) {NewPosTilt = TILT_MAX_DOWN;}
	if(NewPosPan > PAN_MAX_LEFT) 	 {NewPosPan = PAN_MAX_LEFT;}
	if(NewPosPan < PAN_MAX_RIGHT) 	 {NewPosPan = PAN_MAX_RIGHT;}

	if( (NewPosTilt>CurrPosTilt) && (NewPosPan<CurrPosPan) )			// UP - RIGHT
	{
		for(p=CurrPosTilt, y=CurrPosPan ; p<=NewPosTilt || y>=NewPosPan ; p++, y--)
		{
			HEAD_TILT_PULSE = p;
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else if( (NewPosTilt>CurrPosTilt) && (NewPosPan>CurrPosPan) )		// UP - LEFT
	{
		for(p=CurrPosTilt, y=CurrPosPan ; p<=NewPosTilt || y<=NewPosPan ; p++, y++)
		{
			HEAD_TILT_PULSE = p;
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else if( (NewPosTilt<CurrPosTilt) && (NewPosPan<CurrPosPan) )		// DOWN - RIGHT
	{
		for(p=CurrPosTilt, y=CurrPosPan ; p>=NewPosTilt || y>=NewPosPan ; p--, y--)
		{
			HEAD_TILT_PULSE = p;
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosTilt<CurrPosTilt) && (NewPosPan>CurrPosPan) )		// DOWN - LEFT
	{
		for(p=CurrPosTilt, y=CurrPosPan ; p>=NewPosTilt || y<=NewPosPan ; p--, y++)
		{
			HEAD_TILT_PULSE = p;
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosTilt==CurrPosTilt) && (NewPosPan<CurrPosPan) )		// RIGHT
	{
		for(y=CurrPosPan ; y>=NewPosPan ; y--)
		{
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosTilt==CurrPosTilt) && (NewPosPan>CurrPosPan) )		// LEFT
	{
		for(y=CurrPosPan ; y<=NewPosPan ; y++)
		{
			HEAD_PAN_PULSE = y;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosTilt>CurrPosTilt) && (NewPosPan==CurrPosPan) )		// UP
	{
		for (p=CurrPosTilt ; p<=NewPosTilt ; p++)
		{
			HEAD_TILT_PULSE = p;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}

	else if( (NewPosTilt<CurrPosTilt) && (NewPosPan==CurrPosPan) )		// DOWN
	{
		for(p=CurrPosTilt ; p>=NewPosTilt ; p--)
		{
			HEAD_TILT_PULSE = p;
			for(i=0;i<=(speed*speedConst);i++) {asm("NOP");}
			set_PWR_SERVO_NECK(ON);
		}
	}
	else
	{
		Head.NeckPith_pos = CurrPosTilt;
		Head.NeckPan_pos  = CurrPosPan;
	}


	Head.NeckPith_pos = NewPosTilt;
	Head.NeckPan_pos  = NewPosPan;

	osDelay(350);
	set_PWR_SERVO_NECK(OFF);

}


void EarsServos_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

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
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HOOD_PULSE = HoodOpen;

	set_PWR_SERVO_HOOD(ON);
	HOOD_PULSE = HoodOpen;
	osDelay(350);
	set_PWR_SERVO_HOOD(OFF);
}


void Hood_Set(HoodStatus_em stat)
{
	switch (stat)
	{
		case Close:
			set_PWR_SERVO_HOOD(ON);
			HOOD_PULSE = HoodClose;
			osDelay(500);
			set_PWR_SERVO_HOOD(OFF);
			Hood = Close;
		break;

		case Open:
			set_PWR_SERVO_HOOD(ON);
			HOOD_PULSE = HoodOpen;
			osDelay(500);
			set_PWR_SERVO_HOOD(OFF);
			Hood = Open;
		break;

		default:
		break;
	}
}


void Tracking_ArUco(uint8_t ID, uint8_t dir_x, uint8_t pixel_x, uint8_t dir_y, uint8_t pixel_y, uint8_t dist)
{
	int16_t newPix_x = 0;
	int16_t newPix_y = 0;

	if(dir_x == 0) { newPix_x = pixel_x;} 	// positive value
	if(dir_x == 1) { newPix_x = -pixel_x;}	// negative value

	if(dir_y == 0) { newPix_y = pixel_y;} 	// positive value
	if(dir_y == 1) { newPix_y = -pixel_y;}  // negative value


	if(newPix_x >= 75)	//on the Right
		NewPosPan -=10;
	if(newPix_x <= -75) //on the Left
		NewPosPan +=10;

	if(newPix_y >= 75)	//to the Bottom
		NewPosTilt -=10;
	if(newPix_y <= -75)	//to the Top
		NewPosTilt +=10;

	//Head_SetPosition(NewPosTilt, NewPosPan, 10);





/*
	if(dir_x == 0) // positive
		NewPosPan = PanNeutral + ((uint8_t)pixel_x/4);
	else
		NewPosPan = PanNeutral - ((uint8_t)pixel_x/4);	//pixel x coord negative so move servo Right

	if(dir_y == 0) // positive
		NewPosTilt = TiltNeutral - ((uint8_t)pixel_y/4);
	else
		NewPosTilt = TiltNeutral + ((uint8_t)pixel_y/4); //pixel y coord negative so move servo Up
*/

}
