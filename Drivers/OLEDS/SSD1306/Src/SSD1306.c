#include "SSD1306.h"


uint8_t SSD1306_Buff_L[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
uint8_t SSD1306_Buff_R[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

SSD1306_LEFT_st SSD1306_Left;
SSD1306_RIGHT_st SSD1306_Right;


HAL_StatusTypeDef SSD1306_WriteCmdRight(uint8_t command)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_RIGHT, 0x00, 1, &command, 1, 100);
	return status;
}


HAL_StatusTypeDef SSD1306_WriteCmdLeft(uint8_t command)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_LEFT, 0x00, 1, &command, 1, 100);
	return status;
}


HAL_StatusTypeDef SSD1306_Init(void)
{	
	HAL_StatusTypeDef status;
	uint32_t I2C_TimeoutL = 0;
	uint32_t I2C_TimeoutR = 0;

	set_EYES_RST(ON);
	set_EYES_RST(OFF);

	HAL_Delay(100);
	
	/* Init LCD */

	while(HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_LEFT, 3, 100) != HAL_OK)
	{
		set_LED_ERR(ON);
		status = HAL_ERROR;
		asm("NOP");

		I2C_TimeoutL ++;
		if(I2C_TimeoutL > I2C_MAX_TIMEOUT) break;
	}

	while(HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_RIGHT, 3, 100) != HAL_OK)
	{
		set_LED_ERR(ON);
		status = HAL_ERROR;
		asm("NOP");

		I2C_TimeoutR ++;
		if(I2C_TimeoutR > I2C_MAX_TIMEOUT) break;
	}

	set_LED_ERR(OFF);

	status = SSD1306_WriteCmdLeft(0xAE); //display off
	status = SSD1306_WriteCmdLeft(0x20); //Set Memory Addressing Mode
	status = SSD1306_WriteCmdLeft(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	status = SSD1306_WriteCmdLeft(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	status = SSD1306_WriteCmdLeft(0xC0); //Set COM Output Scan Direction		//C8 normal
	status = SSD1306_WriteCmdLeft(0xA1); //--set segment re-map 0 to 127		//A1 normal
	status = SSD1306_WriteCmdLeft(0x00); //---set low column address
	status = SSD1306_WriteCmdLeft(0x10); //---set high column address
	status = SSD1306_WriteCmdLeft(0x40); //--set start line address
	status = SSD1306_WriteCmdLeft(0x81); //--set contrast control register
	status = SSD1306_WriteCmdLeft(0xFF);
	status = SSD1306_WriteCmdLeft(0xA6); //--set normal display
	status = SSD1306_WriteCmdLeft(0xA8); //--set multiplex ratio(1 to 64)
	status = SSD1306_WriteCmdLeft(0x3F); //
	status = SSD1306_WriteCmdLeft(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	status = SSD1306_WriteCmdLeft(0xD3); //-set display offset
	status = SSD1306_WriteCmdLeft(0x10); //-not offset
	status = SSD1306_WriteCmdLeft(0xD5); //--set display clock divide ratio/oscillator frequency
	status = SSD1306_WriteCmdLeft(0xF0); //--set divide ratio
	status = SSD1306_WriteCmdLeft(0xD9); //--set pre-charge period
	status = SSD1306_WriteCmdLeft(0x22); //
	status = SSD1306_WriteCmdLeft(0xDA); //--set com pins hardware configuration
	status = SSD1306_WriteCmdLeft(0x12);
	status = SSD1306_WriteCmdLeft(0xDB); //--set vcomh
	status = SSD1306_WriteCmdLeft(0x20); //0x20,0.77xVcc
	status = SSD1306_WriteCmdLeft(0x8D); //--set DC-DC enable
	status = SSD1306_WriteCmdLeft(0x14); //
	status = SSD1306_WriteCmdLeft(0xAF); //--turn on SSD1306 panel

	status = SSD1306_WriteCmdRight(0xAE); //display off
	status = SSD1306_WriteCmdRight(0x20); //Set Memory Addressing Mode
	status = SSD1306_WriteCmdRight(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	status = SSD1306_WriteCmdRight(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	status = SSD1306_WriteCmdRight(0xC0); //Set COM Output Scan Direction		//C8 normal
	status = SSD1306_WriteCmdRight(0xA1); //--set segment re-map 0 to 127		//A1 normal
	status = SSD1306_WriteCmdRight(0x00); //---set low column address
	status = SSD1306_WriteCmdRight(0x10); //---set high column address
	status = SSD1306_WriteCmdRight(0x40); //--set start line address
	status = SSD1306_WriteCmdRight(0x81); //--set contrast control register
	status = SSD1306_WriteCmdRight(0xFF);
	status = SSD1306_WriteCmdRight(0xA6); //--set normal display
	status = SSD1306_WriteCmdRight(0xA8); //--set multiplex ratio(1 to 64)
	status = SSD1306_WriteCmdRight(0x3F); //
	status = SSD1306_WriteCmdRight(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	status = SSD1306_WriteCmdRight(0xD3); //-set display offset
	status = SSD1306_WriteCmdRight(0x10); //-not offset
	status = SSD1306_WriteCmdRight(0xD5); //--set display clock divide ratio/oscillator frequency
	status = SSD1306_WriteCmdRight(0xF0); //--set divide ratio
	status = SSD1306_WriteCmdRight(0xD9); //--set pre-charge period
	status = SSD1306_WriteCmdRight(0x22); //
	status = SSD1306_WriteCmdRight(0xDA); //--set com pins hardware configuration
	status = SSD1306_WriteCmdRight(0x12);
	status = SSD1306_WriteCmdRight(0xDB); //--set vcomh
	status = SSD1306_WriteCmdRight(0x20); //0x20,0.77xVcc
	status = SSD1306_WriteCmdRight(0x8D); //--set DC-DC enable
	status = SSD1306_WriteCmdRight(0x14); //
	status = SSD1306_WriteCmdRight(0xAF); //--turn on SSD1306 panel

	// Clear screen
	SSD1306_FillLeft(Black);
	SSD1306_FillRight(Black);
	
	// Set default values for screen object
	SSD1306_Left.CurrentX = 0;
	SSD1306_Left.CurrentY = 0;
	SSD1306_Right.CurrentX = 0;
	SSD1306_Right.CurrentY = 0;
	
	SSD1306_Left.Initialized = 1;
	SSD1306_Right.Initialized = 1;

	// Flush buffer to screen
	status = SSD1306_UpdateLeft();
	status = SSD1306_UpdateRight();
	
	return status;
}


void SSD1306_SetContrastLeft(uint8_t contrast)
{
	SSD1306_WriteCmdLeft(0x81);
	SSD1306_WriteCmdLeft(contrast);
	SSD1306_UpdateLeft();
}


void SSD1306_SetContrastRight(uint8_t contrast)
{
	SSD1306_WriteCmdRight(0x81);
	SSD1306_WriteCmdRight(contrast);
	SSD1306_UpdateRight();
}


void SSD1306_dim(uint8_t start, uint8_t end, uint8_t speed)
{
	uint8_t i=0;
	uint8_t dimVal;

	dimVal = ABS(end-start);

	if(start <= end)
	{
		for(i=start ; i<=dimVal ; i++)
		{
			SSD1306_WriteCmdRight(0x81);
			SSD1306_WriteCmdRight(i);
			SSD1306_WriteCmdLeft(0x81);
			SSD1306_WriteCmdLeft(i);
			SSD1306_UpdateRight();
			SSD1306_UpdateLeft();
			osDelay(speed);
		}
	}
	if(start >= end)
	{
		for(i=start ; i>=dimVal ; i--)
		{
			SSD1306_WriteCmdRight(0x81);
			SSD1306_WriteCmdRight(i);
			SSD1306_WriteCmdLeft(0x81);
			SSD1306_WriteCmdLeft(i);
			SSD1306_UpdateRight();
			SSD1306_UpdateLeft();
			osDelay(speed);
		}
	}
}


//  Fill the whole screen with the given color
void SSD1306_FillLeft(SSD1306_COLOR color)
{
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buff_L); i++)
	{
		SSD1306_Buff_L[i] = (color == Black) ? 0x00 : 0xFF;
	}
}


void SSD1306_FillRight(SSD1306_COLOR color)
{
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buff_R); i++)
	{
		SSD1306_Buff_R[i] = (color == Black) ? 0x00 : 0xFF;
	}
}


//  Write the screenbuffer with changed to the screen
HAL_StatusTypeDef SSD1306_UpdateLeft(void)
{
	HAL_StatusTypeDef status;
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		SSD1306_WriteCmdLeft(0xB0 + i);
		SSD1306_WriteCmdLeft(0x00);
		SSD1306_WriteCmdLeft(0x10);

		status = HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_LEFT, 0x40, 1, &SSD1306_Buff_L[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
	return status;
}

HAL_StatusTypeDef SSD1306_UpdateRight(void)
{
	HAL_StatusTypeDef status;
	uint8_t i;
	
	for (i = 0; i < 8; i++)
	{
		SSD1306_WriteCmdRight(0xB0 + i);
		SSD1306_WriteCmdRight(0x00);
		SSD1306_WriteCmdRight(0x10);

		status = HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, SSD1306_RIGHT, 0x40, 1, &SSD1306_Buff_R[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
	}
	return status;
}


void SSD1306_DrawPixelLeft(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	x = x + 32;
	y = y + 16;

	if( (x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT) )
	{
		// Don't write outside the buffer
		return;
	}

	// Check if pixel should be inverted
	if (SSD1306_Left.Inverted)
	{
		color = (SSD1306_COLOR)!color;
	}

	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buff_L[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
	}
	else
	{
		SSD1306_Buff_L[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

void SSD1306_DrawPixelRight(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	x = x + 32;
	y = y + 16;

	if( (x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT) )
	{
		// Don't write outside the buffer
		return;
	}
	
	// Check if pixel should be inverted
	if (SSD1306_Right.Inverted)
	{
		color = (SSD1306_COLOR)!color;
	}
	
	// Draw in the right color
	if (color == White)
	{
		SSD1306_Buff_R[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
	} 
	else 
	{
		SSD1306_Buff_R[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}


char SSD1306_WriteCharLeft(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;

	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306_Left.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306_Left.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}

	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				SSD1306_DrawPixelLeft(SSD1306_Left.CurrentX + j, (SSD1306_Left.CurrentY + i), (SSD1306_COLOR) color);
			}
			else
			{
				SSD1306_DrawPixelLeft(SSD1306_Left.CurrentX + j, (SSD1306_Left.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}

	// The current space is now taken
	SSD1306_Left.CurrentX += Font.FontWidth;

	// Return written char for validation
	return ch;
}

char SSD1306_WriteCharRight(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;
	
	// Check remaining space on current line
	if (SSD1306_WIDTH <= (SSD1306_Right.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT <= (SSD1306_Right.CurrentY + Font.FontHeight))
	{
		// Not enough space on current line
		return 0;
	}
	
	// Use the font to write
	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000) 
			{
				SSD1306_DrawPixelRight(SSD1306_Right.CurrentX + j, (SSD1306_Right.CurrentY + i), (SSD1306_COLOR) color);
			} 
			else 
			{
				SSD1306_DrawPixelRight(SSD1306_Right.CurrentX + j, (SSD1306_Right.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	
	// The current space is now taken
	SSD1306_Right.CurrentX += Font.FontWidth;
	
	// Return written char for validation
	return ch;
}


char SSD1306_WriteStringLeft(char* str, FontDef Font, SSD1306_COLOR color)
{
	while (*str) 
	{
		if (SSD1306_WriteCharLeft(*str, Font, color) != *str)
		{
			// Char could not be written
			return *str;
		}
		str++;
	}
	return *str;
}


char SSD1306_WriteStringRight(char* str, FontDef Font, SSD1306_COLOR color)
{
	while (*str)
	{
		if (SSD1306_WriteCharRight(*str, Font, color) != *str)
		{
			// Char could not be written
			return *str;
		}
		str++;
	}
	return *str;
}


void SSD1306_SetCursorLeft(uint8_t x, uint8_t y)
{
	SSD1306_Left.CurrentX = x;
	SSD1306_Left.CurrentY = y;
}


void SSD1306_SetCursorRight(uint8_t x, uint8_t y)
{
	SSD1306_Right.CurrentX = x;
	SSD1306_Right.CurrentY = y;
}


void SSD1306_LineLeft(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
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
			SSD1306_DrawPixelLeft(y0, x0, color);
		}
		else
		{
			SSD1306_DrawPixelLeft(x0, y0, color);
		}
		err -= dy;

		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}

void SSD1306_LineRight(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
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
			SSD1306_DrawPixelRight(y0, x0, color);
		}
		else
		{
			SSD1306_DrawPixelRight(x0, y0, color);
		}
		err -= dy;

		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}



/**************************************************************************/
/*!
   @brief      Draw PROGMEM-resident XBitMap Files (*.xbm), exported from GIMP.
   Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
   C Array can be directly used with this function.
   There is no RAM-resident version of this function; if generating bitmaps
   in RAM, use the format defined by drawBitmap() and call that instead.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
*/
/**************************************************************************/


void SSD1306_DrawBitmapLeft(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;
	int16_t i, j;

	SSD1306_FillLeft(Black);

	for(j=0; j<h; j++, y++)
	{
		for(i=0; i<w; i++ )
		{
			if(i & 7) byte >>= 1;
			else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
			// Nearly identical to drawBitmap(), only the bit order
			// is reversed here (left-to-right = LSB to MSB):
			if(byte & 0x01) SSD1306_DrawPixelLeft(x+i, y, color);
		}
	}
	SSD1306_UpdateLeft();
}


void SSD1306_DrawBitmapRight(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;
    int16_t i, j;

    SSD1306_FillRight(Black);

    for(j=0; j<h; j++, y++)
    {
        for(i=0; i<w; i++ )
        {
            if(i & 7) byte >>= 1;
            else      byte   = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            // Nearly identical to drawBitmap(), only the bit order
            // is reversed here (left-to-right = LSB to MSB):
            if(byte & 0x01) SSD1306_DrawPixelRight(x+i, y, color);
        }
    }
    SSD1306_UpdateRight();
}


void SSD1306_drawFastVLineLeft(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	SSD1306_LineLeft(x, y, x, y+h-1, color);
}


void SSD1306_drawFastVLineRight(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	SSD1306_LineRight(x, y, x, y+h-1, color);
}

void SSD1306_fillRectLeft(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	int16_t i;

    for (i=x; i<x+w; i++)
    {
    	SSD1306_drawFastVLineLeft(i, y, h, color);
    }
}

void SSD1306_fillRectRight(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	int16_t i;

    for (i=x; i<x+w; i++)
    {
    	SSD1306_drawFastVLineRight(i, y, h, color);
    }
}


void SSD1306_fillCircleLeft(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1))
        {
            if(corners & 1) SSD1306_drawFastVLineLeft(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) SSD1306_drawFastVLineLeft(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py)
        {
            if(corners & 1) SSD1306_drawFastVLineLeft(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) SSD1306_drawFastVLineLeft(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void SSD1306_fillCircleRight(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1))
        {
            if(corners & 1) SSD1306_drawFastVLineRight(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) SSD1306_drawFastVLineRight(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py)
        {
            if(corners & 1) SSD1306_drawFastVLineRight(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) SSD1306_drawFastVLineRight(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}


void SSD1306_fillRoundRectLeft(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis

    SSD1306_FillLeft(Black);

    if(r > max_radius) r = max_radius;
    // smarter version

    SSD1306_fillRectLeft(x+r, y, w-2*r, h, color);
    // draw four corners
    SSD1306_fillCircleLeft(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    SSD1306_fillCircleLeft(x+r    , y+r, r, 2, h-2*r-1, color);

    SSD1306_UpdateLeft();
}

void SSD1306_fillRoundRectRight(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis

    SSD1306_FillRight(Black);

    if(r > max_radius) r = max_radius;
    // smarter version

    SSD1306_fillRectRight(x+r, y, w-2*r, h, color);
    // draw four corners
    SSD1306_fillCircleRight(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    SSD1306_fillCircleRight(x+r    , y+r, r, 2, h-2*r-1, color);

    SSD1306_UpdateRight();
}

