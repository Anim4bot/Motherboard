#include "PlatformType.h"
#include "SSD1306.h"

void Eyes_WakeUp(void)
{
	uint8_t i;
	uint8_t y_offset = 8;

	for(i=0 ; i<=30 ; i++)
	{
		SSD1306_fillRoundRectLeft(22, y_offset, 20, i, 4, White);
		SSD1306_fillRoundRectRight(22, y_offset, 20, i, 4, White);
		osDelay(1);
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

void Eyes_Sleeping(void)
{
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 0, 4, White);
}

void Eyes_Sleepy(void)
{

}

void Eyes_Neutral(void)
{
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 30, 4, White);
}

void Eyes_Blink(void)
{
	SSD1306_SetContrastLeft(0);
	SSD1306_SetContrastRight(0);
	osDelay(100);
	SSD1306_SetContrastLeft(0x80);
	SSD1306_SetContrastRight(0x80);
}

void Eyes_BlinkHigh(void)
{

}

void Eyes_BlinkLow(void)
{
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
	SSD1306_fillRoundRectRight(22, 8, 20, 30, 4, White);
}

void Eyes_BlinkLeft(void)
{
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 0, 4, White);
	osDelay(100);
	SSD1306_fillRoundRectLeft(22, 8, 20, 30, 4, White);
}

void Eyes_BlinkRight(void)
{
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





