#include "PlatformType.h"
#include "EyeAnimations.h"
#include "Robot.h"


uint8_t Pupil_x		= 22;
uint8_t Pupil_y 	= 8;
uint8_t Pupil_width = 20;
uint8_t Pupil_heigh = 30;


void Eyes_WakingUp(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = WakingUp;

	for(i=0 ; i<=30 ; i++)
	{
		SSD1306_fillRoundRectLeft(22, y_offset, 20, i, 4, White);
		SSD1306_fillRoundRectRight(22, y_offset, 20, i, 4, White);
		osDelay(speed);
	}
	osDelay(200);
	SSD1306_fillRoundRectLeft(22, y_offset, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, y_offset, 20, 30, 4, White);
	osDelay(200);
	SSD1306_fillRoundRectLeft(22, y_offset, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, y_offset, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, y_offset, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, y_offset, 20, 30, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, y_offset, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, y_offset, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, y_offset, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, y_offset, 20, 30, 4, White);

}


void Eyes_GoingToSleep(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = GoingToSleep;

	for(i=30 ; i>=1 ; i--)
	{
		SSD1306_fillRoundRectLeft(22, y_offset, 20, i, 4, White);
		SSD1306_fillRoundRectRight(22, y_offset, 20, i, 4, White);
		osDelay(speed);
	}
}



void Eyes_Sleeping(void)
{
	//SSD1306_dim()
	osDelay(100);

	Robot.Eyes.Expression = Sleeping;

	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 0, 4, White);
}

void Eyes_Sleepy(void)
{

	Robot.Eyes.Expression = Sleepy;
}

void Eyes_Neutral(void)
{
	Robot.Eyes.Expression = Neutral;

	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 30, 4, White);
}

void Eyes_Blink(void)
{

	Robot.Eyes.Expression = Blink;

	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 30, 4, White);
	/*
	SSD1306_SetContrastLeft(0);
	SSD1306_SetContrastRight(0);
	osDelay(100);
	SSD1306_SetContrastLeft(0x80);
	SSD1306_SetContrastRight(0x80);
	*/
}

void Eyes_BlinkHigh(void)
{

}

void Eyes_BlinkLow(void)
{

}

void Eyes_BlinkLeft(void)
{
	Robot.Eyes.Expression = BlinkLeft;
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
}

void Eyes_BlinkRight(void)
{
	Robot.Eyes.Expression = BlinkRight;
	osDelay(100);
	SSD1306_fillRoundRectRight(22, 8, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectRight(22, 8, 20, 30, 4, White);
}

void Eyes_Happy(void)
{

}

void Eyes_Sad(void)
{

}

void Eyes_Worried(void)
{

}

void Eyes_Focused(void)
{

}

void Eyes_Annoyed(void)
{

}

void Eyes_Surprised(void)
{

}

void Eyes_Skeptic(void)
{

}

void Eyes_Squint(void)
{

}

void Eyes_Frustrated(void)
{

}

void Eyes_Unimpressed(void)
{

}

void Eyes_Angry(void)
{

}

void Eyes_Furious(void)
{

}

void Eyes_Scared(void)
{

}

void Eyes_Awe(void)
{

}





