#include "PlatformType.h"
#include "EyeAnimations.h"
#include "Robot.h"

//SSD1306_fillRoundRectLeft(x, y, width, height, radius, color)


const uint8_t Pupil_width = 24;
const uint8_t Pupil_height = 32;
const uint8_t Pupil_radius = 4;

uint8_t Pupil_x = 22;
uint8_t Pupil_y = 8;


void Eyes_Dim(uint8_t val)
{
	SSD1306_dim(val, val, 1);
}

void Eyes_WakingUp(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = WakingUp;

	for(i=0 ; i<=Pupil_height ; i++)
	{
		SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_UpdateLeft();
		SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_UpdateRight();
		osDelay(speed);
	}
	osDelay(200);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
	osDelay(200);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateRight();
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateRight();
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	osDelay(100);
	Eyes_Neutral();
}


void Eyes_GoingToSleep(AnimSpeed_enum speed)
{
	uint8_t i;
	uint8_t y_offset = 8;

	Robot.Eyes.Expression = GoingToSleep;

	for(i=Pupil_height ; i>=2 ; i--)
	{
		SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_UpdateLeft();
		SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, i, Pupil_radius, White);
		SSD1306_UpdateRight();
		osDelay(speed);
	}
}


void Eyes_Sleeping(uint8_t speed)
{
	osDelay(100);

	Robot.Eyes.Expression = Sleeping;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 3, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 3, Pupil_radius, White);
	SSD1306_UpdateRight();
}

void Eyes_Sleepy(void)
{

	Robot.Eyes.Expression = Sleepy;
}


void Eyes_Follow(int8_t x, int8_t y)
{
	Robot.Eyes.Expression = Follow;


	SSD1306_fillRoundRectLeft(Pupil_x+x, Pupil_y+y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x+x, Pupil_y+y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
}


void Eyes_Neutral(void)
{
	Robot.Eyes.Expression = Neutral;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
}

void Eyes_Blink(void)
{

	Robot.Eyes.Expression = Blink;

	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	SSD1306_UpdateRight();
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
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
	SSD1306_UpdateLeft();
	osDelay(100);
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
}

void Eyes_BlinkRight(void)
{
	Robot.Eyes.Expression = BlinkRight;
	osDelay(100);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, 0, Pupil_radius, White);
	osDelay(100);
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
}

void Eyes_Happy(void)
{
	Robot.Eyes.Expression = Happy;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height-6, Black);
	SSD1306_UpdateLeft();

	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height-6, Black);
	SSD1306_UpdateRight();

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





