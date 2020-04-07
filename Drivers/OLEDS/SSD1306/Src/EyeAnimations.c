#include "PlatformType.h"
#include "EyeAnimations.h"
#include "Robot.h"

//SSD1306_fillRoundRectLeft(x, y, width, height, radius, color)


const uint8_t Pupil_width = 24;
const uint8_t Pupil_height = 32;
const uint8_t Pupil_radius = 4;

uint8_t Pupil_x = 22;
uint8_t Pupil_y = 8;



void Eyes_SetExpression(Expressions_enum val, AnimSpeed_enum speed)
{
	switch (val)
	{
		case WakingUp:
			Eyes_WakingUp(speed);
		break;
		case GoingToSleep:
			Eyes_GoingToSleep(speed);
		break;
		case Sleeping:
			Eyes_Sleeping(speed);
		break;
		case Sleepy:
			Eyes_Sleepy();
		break;
		case Neutral:
			Eyes_Neutral();
		break;
		case Blink:
			Eyes_Blink();
		break;
		case BlinkLeft:
			Eyes_BlinkLeft();
		break;
		case BlinkRight:
			Eyes_BlinkRight();
		break;
		case Happy:
			Eyes_Happy();
		break;
		case Sad:
			Eyes_Sad();
		break;
		case Worried:
			Eyes_Worried();
		break;
		case Focused:
			Eyes_Focused();
		break;
		case Annoyed:
			Eyes_Annoyed();
		break;
		case Surprised:
			Eyes_Surprised();
		break;
		case Skeptic:
			Eyes_Skeptic();
		break;
		case Squint:
			Eyes_Squint();
		break;
		case Frustrated:
			Eyes_Frustrated();
		break;
		case Unimpressed:
			Eyes_Unimpressed();
		break;
		case Angry:
			Eyes_Angry();
		break;
		case Furious:
			Eyes_Furious();
		break;
		case Scared:
			Eyes_Scared();
		break;
		case Awe:
			Eyes_Awe();
		break;
		default:
			Eyes_Neutral();
		break;
	}
}



void Eyes_Dim(uint8_t val)
{
	SSD1306_dim(val, val, 1);
}


void Eyes_Follow(int8_t x, int8_t y)
{
	Robot.Eyes.Expression = Follow;


	SSD1306_fillRoundRectLeft(Pupil_x+x, Pupil_y+y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x+x, Pupil_y+y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();
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
	uint8_t i;
	Robot.Eyes.Expression = Happy;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	for(i=0 ; i<=Pupil_height-6 ; i++)
	{
		SSD1306_fillRectLeft(Pupil_x, Pupil_y, Pupil_width, i, Black);
		SSD1306_UpdateLeft();
		SSD1306_fillRectRight(Pupil_x, Pupil_y, Pupil_width, i, Black);
		SSD1306_UpdateRight();
	}

/*
	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height-6, Black);
	SSD1306_UpdateLeft();

	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_fillRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height-6, Black);
	SSD1306_UpdateRight();
	*/

}

void Eyes_Sad(void)
{
	int8_t i;
	Robot.Eyes.Expression = Sad;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	//SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	//SSD1306_UpdateRight();

	for(i=14 ; i>=1 ; i--)
	{
		SSD1306_LineLeft(20, 48-i, 48, 38-i, Black);
		SSD1306_LineRight(20, 38-i, 48, 48-i, Black);
	}
	SSD1306_UpdateLeft();
	SSD1306_UpdateRight();

}

void Eyes_Worried(void)
{

}

void Eyes_Focused(void)
{
	uint8_t i;
	Robot.Eyes.Expression = Focused;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	for(i=0 ; i<=9 ; i++)
	{
		SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y+i, Pupil_width, Pupil_height-2*i, Pupil_radius, White);
		SSD1306_UpdateLeft();
		SSD1306_fillRoundRectRight(Pupil_x, Pupil_y+i, Pupil_width, Pupil_height-2*i, Pupil_radius, White);
		SSD1306_UpdateRight();
		osDelay(1);
	}
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
	uint8_t i;
	Robot.Eyes.Expression = Frustrated;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	for(i=Pupil_height ; i>=Pupil_height-16 ; i--)
	{
		SSD1306_fillRectLeft(Pupil_x, i, Pupil_width, Pupil_height, Black);
		SSD1306_UpdateLeft();
		SSD1306_fillRectRight(Pupil_x, i, Pupil_width, Pupil_height, Black);
		SSD1306_UpdateRight();
	}
}

void Eyes_Unimpressed(void)
{
	uint8_t i;
	Robot.Eyes.Expression = Unimpressed;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	for(i=Pupil_height ; i>=Pupil_height-16 ; i--)
	{
		SSD1306_fillRectLeft(Pupil_x, i, Pupil_width, Pupil_height, Black);
		SSD1306_UpdateLeft();
		if(i>=Pupil_height-10)
		{
			SSD1306_fillRectRight(Pupil_x, i, Pupil_width, Pupil_height, Black);
			SSD1306_UpdateRight();
		}

		osDelay(1);
	}
}

void Eyes_Angry(void)
{
	uint8_t i;
	Robot.Eyes.Expression = Angry;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	//SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	//SSD1306_UpdateRight();

	for(i=16 ; i>=1 ; i--)
	{
		SSD1306_LineLeft(20, 42-i, 48, 48-i, Black);
		SSD1306_LineRight(20, 48-i, 48, 42-i, Black);
	}
	SSD1306_UpdateLeft();
	SSD1306_UpdateRight();
}

void Eyes_Furious(void)
{
	uint8_t i;
	Robot.Eyes.Expression = Furious;

	SSD1306_fillRoundRectLeft(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	//SSD1306_UpdateLeft();
	SSD1306_fillRoundRectRight(Pupil_x, Pupil_y, Pupil_width, Pupil_height, Pupil_radius, White);
	SSD1306_UpdateRight();

	for(i=14 ; i>=1 ; i--)
	{
		SSD1306_LineLeft(20, 37-i, 48, 48-i, Black);
		SSD1306_LineRight(20, 48-i, 48, 37-i, Black);
	}
	SSD1306_UpdateLeft();
	SSD1306_UpdateRight();
}

void Eyes_Scared(void)
{

}

void Eyes_Awe(void)
{

}





