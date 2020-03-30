#include "SSD1320.h"
#include "Robot.h"

uint8_t previousMode = 0, currentMode = 0;
uint8_t color = 1;


//uint8_t screenMemory[FLEX_OLED_BUFF_SIZE] = {0};
static SSD1320_t SSD1320;

const unsigned char  *fontsPointer[] =
{
	font5x7,
	font8x16,
};

const uint8_t MenuCubeSize = 8;

//LCD Memory organized in 20 bytes (160 columns) and 32 rows = 640 bytes
static uint8_t screenMemory [640] = {0};


void Flex_OLED_Rst(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	osDelay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}


HAL_StatusTypeDef ssd1320_WriteCommand(uint8_t cmd)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_StatusTypeDef status;

	GPIO_InitStruct.Pin = FLEX_OLED_SCK_Pin|FLEX_OLED_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	set_OLED_SYS_CS(LOW);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);		//MOSI
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);		//CLK
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);		//CLK


    GPIO_InitStruct.Pin = FLEX_OLED_SCK_Pin|FLEX_OLED_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	status = HAL_SPI_Transmit(&hspi2, (uint8_t*)&cmd, 1, 1000);

	set_OLED_SYS_CS(HIGH);

	return status;
}


HAL_StatusTypeDef ssd1320_WriteData(uint8_t cmd)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	HAL_StatusTypeDef status;

	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	set_OLED_SYS_CS(LOW);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);		//MOSI
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);		//CLK
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);		//CLK
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);		//MOSI

    GPIO_InitStruct.Pin = FLEX_OLED_SCK_Pin|FLEX_OLED_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	status = HAL_SPI_Transmit(&hspi2, &cmd, sizeof(cmd), 10);

	set_OLED_SYS_CS(HIGH);

	return status;
}


void Flex_OLED_Init(void)
{
	set_OLED_SYS(OFF);
	osDelay(5);
	set_OLED_SYS(ON);
	osDelay(5);

	ssd1320_WriteCommand(DISPLAYOFF);       	// 0xAE - Display off
	ssd1320_WriteCommand(SETDISPLAYCLOCKDIV); 	// 0xD5 - Clock divide ratio/osc. freq
	ssd1320_WriteCommand(0xC2);					// 0xC2 - osc clock=0xC divide ratio = 0x2

	ssd1320_WriteCommand(SETMULTIPLEX); 		// 0xA8 - Multiplex ratio
	ssd1320_WriteCommand(0x1F);                 // 0x1F - 31

	ssd1320_WriteCommand(SETDISPLAYOFFSET); 	// 0xD3 - Display offset
	ssd1320_WriteCommand(0x60);                 // 0x60 - 96

	ssd1320_WriteCommand(SETSTARTLINE); 		// 0xA2 - Start line
	ssd1320_WriteCommand(0x00);                 // 0x00 - Line 0

	ssd1320_WriteCommand(SETSEGREMAP);  		// 0xA0 - Segment re-map

	ssd1320_WriteCommand(COMSCANINC); 			// 0xC0 - COM Output scan direction

	ssd1320_WriteCommand(SETCOMPINS); 			// 0xDA - seg pins hardware config
	ssd1320_WriteCommand(0x12);					// 0x12 -

	ssd1320_WriteCommand(SETCONTRAST);    		// 0x81 - Contrast control
	ssd1320_WriteCommand(0x5A);                 // 0x5A - value between 0x00 and 0xFF

	ssd1320_WriteCommand(SETPHASELENGTH); 		// 0xD9 - Pre-charge period
	ssd1320_WriteCommand(0x22);                 // 0x22

	ssd1320_WriteCommand(SETVCOMDESELECT);   	// 0xDB - VCOMH Deselect level
	ssd1320_WriteCommand(0x30);                 // 0x30

	ssd1320_WriteCommand(SELECTIREF);     		// 0xAD - Internal IREF Enable
	ssd1320_WriteCommand(0x10);                 // 0x10

	ssd1320_WriteCommand(MEMORYMODE); 			// 0x20 - Memory addressing mode
	ssd1320_WriteCommand(0x00);                 // 0x00 - Horizontal

	// disable internal charge pump
	ssd1320_WriteCommand(SETCHARGEPMP1); 		// 0x8D - Internal charge pump
	ssd1320_WriteCommand(0x01);                 // 0x01
	ssd1320_WriteCommand(SETCHARGEPMP2); 		// 0xAC - Internal charge pump
	ssd1320_WriteCommand(0x00);                 // 0x00

	// set entire display on/off
	ssd1320_WriteCommand(RESETALLON);      		// 0xA4 - Display on

	// set normal/inverse display
	ssd1320_WriteCommand(RESETINVERT);  		// 0xA6 - Normal display (not inverted)



	//Set the row and column limits for this display and set the RAM pointer on the display to 0,0
	ssd1320_WriteCommand(SETCOLUMN); 					// Set column address
	ssd1320_WriteCommand(0); 							// Set start address
	ssd1320_WriteCommand((FLEX_OLED_WIDTH / 2) - 1); 	// There are 160 pixels but each byte is 2 pixels. We want addresses 0 to 79.

	//Set the row and column limits for this display and set the RAM pointer on the display to 0,0
	ssd1320_WriteCommand(SETROW); 						// Set row address
	ssd1320_WriteCommand(0); 							// Set start address
	ssd1320_WriteCommand(FLEX_OLED_WIDTH - 1); 			// Set end address: Display has 32 rows of pixels.

	// display on
	ssd1320_WriteCommand(DISPLAYON);         	// 0xAF - Display on

	Flex_OLED_setFontType(font5x7_t);
}


void Flex_OLED_dim(uint8_t start, uint8_t end, uint8_t speed)
{
	uint8_t i=0;
	uint8_t dimVal;

	dimVal = ABS(end-start);

	if(start <= end)
	{
		for(i=start ; i<=dimVal ; i++)
		{
			ssd1320_WriteCommand(SETCONTRAST);
			ssd1320_WriteCommand(dimVal);
			osDelay(speed);
		}
	}
	if(start >= end)
	{
		for(i=start ; i>=dimVal ; i--)
		{
			ssd1320_WriteCommand(SETCONTRAST);
			ssd1320_WriteCommand(dimVal);
			osDelay(speed);
		}
	}
}

uint8_t flipByte(uint8_t c)
{
  c = ((c >> 1) & 0x55) | ((c << 1) & 0xAA);
  c = ((c >> 2) & 0x33) | ((c << 2) & 0xCC);
  c = (c >> 4) | (c << 4) ;

  return c;
}


void Flex_OLED_Invert(uint8_t inv)
{
	if(inv == 1)
	{
		ssd1320_WriteCommand(INVERTDISPLAY);
	}
	else
	{
		ssd1320_WriteCommand(RESETINVERT);
	}
}


void Flex_OLED_setContrast(uint8_t contrast)
{
	ssd1320_WriteCommand(SETCONTRAST);
	ssd1320_WriteCommand(contrast);
}


void Flex_OLED_scrollStop(void)
{
	ssd1320_WriteCommand(DEACTIVATESCROLL);
}


void Flex_OLED_scrollRight(uint8_t start, uint8_t stop)
{
	if (stop < start) // stop must be larger or equal to start
	return;
	Flex_OLED_scrollStop();   // need to disable scrolling before starting to avoid memory corruption
	ssd1320_WriteCommand(RIGHTHORIZONTALSCROLL);
	ssd1320_WriteCommand(0x00); //Byte A - Dummy 0x00
	ssd1320_WriteCommand(start); //Byte B - Define start page address
	ssd1320_WriteCommand(0x07); //Byte C - Set scroll speed to 2 frames (time interval in number of frames 5/64/128/256/3/4/25/2)
	ssd1320_WriteCommand(stop); //Byte D - Define end page address
	ssd1320_WriteCommand(0x00); //Byte E - Dummy 0x00
	ssd1320_WriteCommand(0xFF); //Byte F - Dummy 0xFF
	ssd1320_WriteCommand(ACTIVATESCROLL);
}


void Flex_OLED_scrollLeft(uint8_t start, uint8_t stop)
{
	if (stop < start) // Stop must be larger or equal to start
	return;

	Flex_OLED_scrollStop();   // Must disable scrolling before starting to avoid memory corruption

	ssd1320_WriteCommand(LEFTHORIZONTALSCROLL);
	ssd1320_WriteCommand(0x00); //Dummy byte
	ssd1320_WriteCommand(0x00); //Dummy byte
	ssd1320_WriteCommand(start); //Define starting page address

	ssd1320_WriteCommand(32); //Number of rows to scroll. You scan scroll part of the display

	ssd1320_WriteCommand(stop); //Define end page address
	ssd1320_WriteCommand(0x00);
	ssd1320_WriteCommand(0xFF); //Speed?
	ssd1320_WriteCommand(ACTIVATESCROLL);
}


uint8_t Flex_OLED_clearDisplay(uint8_t mode)
{
	uint8_t ready = 0;
	uint16_t rows;
	uint16_t columns;

	ready = 0;

	if (mode == CLEAR_DISPLAY || mode == CLEAR_ALL) //Clear the RAM on the display
	{
		//Return CGRAM pointer to 0,0
		ssd1320_WriteCommand(SETCOLUMN); 					// Set column address
		ssd1320_WriteCommand(0); 							// Set start address
		ssd1320_WriteCommand((FLEX_OLED_WIDTH / 2) - 1); 	// There are 160 pixels but each byte is 2 pixels. We want addresses 0 to 79.
		ssd1320_WriteCommand(SETROW); 						// Set row address
		ssd1320_WriteCommand(0); 							// Set start address
		ssd1320_WriteCommand(FLEX_OLED_HEIGHT - 1); 		// Set end address: Display has 32 rows of pixels.

		//Display is 160 pixels long and 32 pixels wide
		//Each byte paints two sequential pixels
		//Each 4-bit nibble is the 4-bit grayscale for that pixel
		//There are only 80 columns because each byte has 2 pixels
		for (rows = 0 ; rows < FLEX_OLED_HEIGHT ; rows++)
		{
			for (columns = 0 ; columns < (FLEX_OLED_WIDTH / 2) ; columns++)
			{
				ssd1320_WriteData(0x00);
			}
		}

		if (mode == CLEAR_ALL)
		{
			memset(screenMemory, 0, (FLEX_OLED_HEIGHT * FLEX_OLED_WIDTH / 8)); //Clear the local buffer as well
		}
	}
	else //Clear the local buffer
	{
		memset(screenMemory, 0, (FLEX_OLED_HEIGHT * FLEX_OLED_WIDTH / 8));   // (32 x 160/8) = 640 bytes in the screenMemory buffer
	}

	ready = 1;
	HAL_Delay(10);

	return ready;
}


void Flex_OLED_setCursor(uint8_t x, uint8_t y)
{
	SSD1320.cursorX = x;
	SSD1320.cursorY = y;
}


void Flex_OLED_Update(void)
{
	uint16_t rows;
	uint16_t columns;
	uint8_t bitNumber;
	uint8_t originalByte;
	uint8_t newByte = 0;

	//Return CGRAM pointer to 0,0
	ssd1320_WriteCommand(SETCOLUMN); 					// Set column address
	ssd1320_WriteCommand(0); 							// Set start address
	ssd1320_WriteCommand((FLEX_OLED_WIDTH / 2) - 1); 	// There are 160 pixels but each byte is 2 pixels. We want addresses 0 to 79.
	ssd1320_WriteCommand(SETROW); 						// Set row address
	ssd1320_WriteCommand(0); 							// Set start address
	ssd1320_WriteCommand(FLEX_OLED_HEIGHT - 1); 		// Set end address: Display has 32 rows of pixels.

	for (rows = 0 ; rows < 32 ; rows++)
	{
		for (columns = 0 ; columns < 20 ; columns++)
		{
			originalByte = screenMemory[(int)rows * 20 + columns];
			for (bitNumber = 8 ; bitNumber > 0 ; bitNumber -= 2)
			{
				newByte = 0;
				//Because our buffer is too small to contain 4-bit grayscale,
				//we extrapolate 1 bit onto 4 bits so we pull in two bits to make a byte.
				//We look at each bit in the byte and change it to 0 = 0x00 and 1 = 0x0F.
				if ( (originalByte & (1 << (bitNumber - 1))) != 0) newByte |= 0x0F;
				if ( (originalByte & (1 << (bitNumber - 2))) != 0) newByte |= 0xF0;

				ssd1320_WriteData(newByte);
			}
		}
	}
	osDelay(100);
}


void Flex_OLED_setPixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode)
{
  if ((x < 0) || (x >= FLEX_OLED_WIDTH) || (y < 0) || (y >= FLEX_OLED_HEIGHT))
  {
    return;
  }

  int byteNumber = y * (FLEX_OLED_WIDTH / 8) + (x / 8);

  if (mode == XOR)
  {
    screenMemory[byteNumber] ^= (1 << (7 - (x % 8)));
  }
  else //mode = NORM
  {
    if (color == WHITE)
    {
      screenMemory[byteNumber] |= (1 << (7 - (x % 8)));
    }
    else
    {
      screenMemory[byteNumber] &= ~(1 << (7 - (x % 8)));
    }
  }
}



void Flex_OLED_write(uint8_t c, uint8_t mode)
{

	if (c == '\n')
	{
		SSD1320.cursorY += SSD1320.fontHeight;
		SSD1320.cursorX = 0;
	}
	else if (c == '\r')
	{
	// skip
	}
	else
	{
		Flex_OLED_drawChar(SSD1320.cursorX, SSD1320.cursorY, c, 1, mode);
		SSD1320.cursorX += SSD1320.fontWidth + 1;
		if ((SSD1320.cursorX > (FLEX_OLED_WIDTH - SSD1320.fontWidth)))
		{
			SSD1320.cursorY += SSD1320.fontHeight;
			SSD1320.cursorX = 0;
		}
	}
}


void Flex_OLED_String(char* str, uint8_t mode)
{
	uint8_t i;

	while (*str)
	{
		Flex_OLED_write(*str, mode);
		str++;
	}

}


void Flex_OLED_setFontType(fontType_enum type)
{
	switch(type)
	{
		case font5x7_t:
			SSD1320.fontType = type;
			SSD1320.fontWidth = 5;
			SSD1320.fontHeight = 8;
			SSD1320.fontStartChar = 0;
			SSD1320.fontTotalChar = 255;
			SSD1320.fontMapWidth = 12*100 + 75;
		break;

		case font8x16_t:
			SSD1320.fontType = type;
			SSD1320.fontWidth = 8;
			SSD1320.fontHeight = 16;
			SSD1320.fontStartChar = 32;
			SSD1320.fontTotalChar = 96;
			SSD1320.fontMapWidth = 2*100 + 56;
		break;
		default:
			//do nothing
		break;
	}
}


void Flex_OLED_drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode)
{

  uint8_t rowsToDraw, row, tempC;
  uint8_t i, j, temp;
  uint16_t charPerBitmapRow, charColPositionOnBitmap, charRowPositionOnBitmap, charBitmapStartPosition;

  if ((c < SSD1320.fontStartChar) || (c > (SSD1320.fontStartChar + SSD1320.fontTotalChar - 1))) // no bitmap available for the required c
    return;

  tempC = c - SSD1320.fontStartChar; //Turn user's character into a byte number

  // each row (in datasheet is called a page) is 8 bits high, 16 bit high character will have 2 rows to be drawn
  rowsToDraw = SSD1320.fontHeight / 8; // 8 is LCD's page size, see datasheet
  if (rowsToDraw < 1) rowsToDraw = 1;

  // The following draw function can draw anywhere on the screen, but SLOW pixel by pixel draw
  if (rowsToDraw == 1)
  {
    for  (i = 0 ; i < SSD1320.fontWidth + 1 ; i++)
    {
      if (i == SSD1320.fontWidth) // this is done in a weird way because for 5x7 font, there is no margin, this code add a margin after col 5
        temp = 0;
      else
        temp = pgm_read_byte(fontsPointer[SSD1320.fontType] + FONTHEADERSIZE + (tempC * SSD1320.fontWidth) + i);

      //0x7F is the first vertical line of the lowercase letter h
      //The fonts are coming in upside down?
      temp = flipByte(temp);

      //Step through this line of the character checking each bit and setting a pixel
      for (j = 0 ; j < 8 ; j++)
      {
        if (temp & 0x01) {
        	Flex_OLED_setPixel(x + i, y + j, color, mode);
        }
        else {
        	Flex_OLED_setPixel(x + i, y + j, !color, mode);
        }

        temp >>= 1;
      }
    }
    return;
  }

  // Font height over 8 bit
  // Take character "0" ASCII 48 as example
  charPerBitmapRow = (SSD1320.fontMapWidth / SSD1320.fontWidth); // 256/8 = 32 char per row
  charColPositionOnBitmap = tempC % charPerBitmapRow; // = 16
  charRowPositionOnBitmap = (uint16_t)(tempC / charPerBitmapRow); // = 1
  charBitmapStartPosition = (charRowPositionOnBitmap * SSD1320.fontMapWidth * (SSD1320.fontHeight / 8)) + (charColPositionOnBitmap * SSD1320.fontWidth) ;

  for (row = 0 ; row < rowsToDraw ; row++) {
    for (i = 0 ; i < SSD1320.fontWidth ; i++) {
      temp = pgm_read_byte(fontsPointer[SSD1320.fontType] + FONTHEADERSIZE + (charBitmapStartPosition + i + (row * SSD1320.fontMapWidth)));

      //The fonts are coming in upside down
      //Additionally, the large font #1 has padding at the (now) bottom that causes problems
      //The fonts really need to be updated
      temp = flipByte(temp);

      for (j = 0 ; j < 8 ; j++) {
        if (temp & 0x01) {
        	Flex_OLED_setPixel(x + i, y + j + ((rowsToDraw - 1 - row) * 8), color, mode);
        }
        else {
        	Flex_OLED_setPixel(x + i, y + j + ((rowsToDraw - 1 - row) * 8), !color, mode);
        }
        temp >>= 1;
      }
    }
  }
}



void Flex_OLED_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode)
{
	uint8_t steep = ABS(y1 - y0) > ABS(x1 - x0);
	uint8_t dx, dy;
	int8_t err;
	int8_t ystep;


	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}


	dx = x1 - x0;
	dy = ABS(y1 - y0);
	err = dx / 2;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0 < x1; x0++)
	{
		if (steep)
		{
			Flex_OLED_setPixel(y0, x0, color, mode);
		}
		else
		{
			Flex_OLED_setPixel(x0, y0, color, mode);
		}
		err -= dy;

		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}


void Flex_OLED_lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode)
{
	Flex_OLED_Line(x, y, x + width, y, color, mode);
}


void Flex_OLED_lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode)
{
	Flex_OLED_Line(x, y, x, y + height, color, mode);
}


void Flex_OLED_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode)
{
	uint8_t tempHeight;

	Flex_OLED_lineH(x, y, width, color, mode);
	Flex_OLED_lineH(x, y + height - 1, width, color, mode);

	tempHeight = height - 2;

	if (tempHeight < 1) return;

	Flex_OLED_lineV(x, y + 1, tempHeight, color, mode);
	Flex_OLED_lineV(x + width - 1, y + 1, tempHeight, color, mode);
}


void Flex_OLED_rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode)
{
  for (int i = x; i < x + width; i++)
  {
	  Flex_OLED_lineV(i, y, height, color, mode);
  }
}





uint8_t Flex_OLED_StartupAnimation(uint8_t speed)
{
	uint8_t loop, i, ready;
	uint8_t LogoDelay = 10;
	int ctr;

	ready = 0;

	Flex_OLED_setContrast(0);

	for (ctr = 0 ; ctr < sizeof(AnimabotLogo2) ; ctr++)
	{
	  ssd1320_WriteData(AnimabotLogo2[ctr]); //Write byte directly to display
	}

	for (i=0 ; i<Robot.OLED.OLED_Contrast ; i++)
	{
		Flex_OLED_setContrast(i);
		osDelay(speed);
	}

	osDelay(500);
	for (i=Robot.OLED.OLED_Contrast ; i>1 ; i--)
	{
		Flex_OLED_setContrast(i);
		osDelay(speed);
	}

	Flex_OLED_clearDisplay(CLEAR_ALL);
	osDelay(300);
	ready = 1;

	return ready;
}





void Flex_OLED_Menu_Modes()
{
	int ctr;
	uint8_t buff1[16];

	Flex_OLED_setPixel(1, 1, WHITE, NORM);
	Flex_OLED_setPixel(148, 1, WHITE, NORM);
	Flex_OLED_setPixel(1, 31, WHITE, NORM);
	Flex_OLED_setPixel(148, 31, WHITE, NORM);

	Flex_OLED_Update();
}


void Flex_OLED_Menu_Sensors(void)
{
	uint8_t buff1[48], buff2[48], buff3[48];
	uint8_t xPos = 16;

	Flex_OLED_setCursor(xPos,22);
	sprintf(buff1, "Pitch: %+.2d   PSU: %+.2d  ", (int16_t)Sensor.IMU.Pitch, (int16_t)Sensor.TempPSU);
	Flex_OLED_String(buff1, NORM);

	Flex_OLED_setCursor(xPos,12);
	sprintf(buff2, "Roll : %+.2d   CHG: %+.2d  ", (int16_t)Sensor.IMU.Roll, (int16_t)Sensor.TempCharger);
	Flex_OLED_String(buff2, NORM);

	Flex_OLED_setCursor(xPos,2);
	sprintf(buff3, "IR: %.4d     FAN: %.3d  ", (int16_t)Sensor.dist_IR, (int16_t)Sensor.Fan_Speed);
	Flex_OLED_String(buff3, NORM);

	Flex_OLED_Update();
	osDelay(50);

}


void Flex_OLED_Menu_Battery(void)
{
	uint8_t xOffset = 80;
	uint8_t loop, i, j;
	uint8_t buff1[32], buff2[32], buff3[32];
	volatile uint8_t test[2];
	volatile uint16_t tdata;



	Flex_OLED_setContrast(50);

	if( (Charger.Power.InputVoltage > 11.50) && (Charger.Power.InputVoltage < 14.00) )
	{
		currentMode = 1;

		Flex_OLED_setCursor(60,22);
		sprintf(buff1,"Vbat: %.2fV", Charger.Power.BatVoltage);
		Flex_OLED_String(buff1, NORM);
		Flex_OLED_setCursor(60,12);
		sprintf(buff2,"Ibat: %.2fA", Charger.Power.BatCurrent);
		Flex_OLED_String(buff2, NORM);
		Flex_OLED_setCursor(60,2);
		sprintf(buff2,"Temp: %.2d", (uint16_t)Charger.Power.Die_temp);
		Flex_OLED_String(buff2, NORM);


		Flex_OLED_rect(4, 4, 32, 24, WHITE, NORM);			// battery housing
		Flex_OLED_rectFill(36, 10, 3, 12, WHITE, NORM);		// battery housing head

		Flex_OLED_Update();

		for(loop = 0; loop<1 ; loop++)
		{
			Flex_OLED_rectFill(6, 6, 28, 20, BLACK, NORM);
			for (i=0 ; i<=28 ; i++)
			{
				Flex_OLED_rectFill(6, 6, i, 20, WHITE, NORM);
				Flex_OLED_Update();
				osDelay(5);
			}
		}

	}
	else
	{
		currentMode = 0;
		Flex_OLED_setCursor(10,22);
		sprintf(buff1,"VBATT : %.2fV", Charger.Power.SysVoltage);
		Flex_OLED_String(buff1, NORM);
		Flex_OLED_setCursor(10,12);
		sprintf(buff2,"IBATT : %.3fA", Charger.Power.SysCurrent);
		Flex_OLED_String(buff2, NORM);
		Flex_OLED_setCursor(10,2);
		sprintf(buff3,"Power : %.2fW", Charger.Power.SysPower);
		Flex_OLED_String(buff3, NORM);

		color = !color;
		Flex_OLED_setPixel(149, 1, color, NORM);
		Flex_OLED_setPixel(149, 31, color, NORM);

		Flex_OLED_Update();
	}

	if(previousMode != currentMode)
	{
		Flex_OLED_clearDisplay(CLEAR_ALL);
		Flex_OLED_Update();
		osDelay(100);
	}

	previousMode = currentMode;
}


