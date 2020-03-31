#include "PlatformType.h"
#include "EyeAnimations.h"
#include "Robot.h"

//SSD1306_fillRoundRectLeft(x, y, width, height, radius, color)

uint8_t Pupil_x	 	 = 22;
uint8_t Pupil_y 	 = 8;
uint8_t Pupil_width  = 24;
uint8_t Pupil_height = 32;
uint8_t Pupil_radius = 4;


void Eyes_WakingUp(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = WakingUp;

	for(i=0 ; i<=Pupil_height ; i++)
	{
		SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		osDelay(speed);
	}
	osDelay(200);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	osDelay(200);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);

}


void Eyes_GoingToSleep(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = GoingToSleep;

	for(i=Pupil_height ; i>=2 ; i--)
	{
		SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		osDelay(speed);
	}
}



void Eyes_Sleeping(uint8_t speed)
{
	osDelay(100);

	Robot.Eyes.Expression = Sleeping;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 2, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 2, Pupil_radius, White);

	SSD1306_dim(Robot.Eyes.Contrast, Robot.Eyes.Contrast+50, speed);
	SSD1306_dim(Robot.Eyes.Contrast, Robot.Eyes.Contrast-50, speed);
	osDelay(1000);

}

void Eyes_Sleepy(void)
{

	Robot.Eyes.Expression = Sleepy;
}

void Eyes_Neutral(void)
{
	Robot.Eyes.Expression = Neutral;

	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
}

void Eyes_Blink(void)
{

	Robot.Eyes.Expression = Blink;

	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
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
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
}

void Eyes_BlinkRight(void)
{
	Robot.Eyes.Expression = BlinkRight;
	osDelay(100);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
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





