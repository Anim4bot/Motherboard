#ifndef SSD1320_H_
#define SSD1320_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"
#include "string.h"
#include "font_8x16.h"
#include "font_5x7.h"
#include "AnimabotLogo.h"

extern SPI_HandleTypeDef hspi2;


#define swap(a, b) { uint8_t t = a; a = b; b = t; }
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

#define FLEX_OLED_WIDTH			160
#define FLEX_OLED_HEIGHT 		32
#define FLEX_OLED_BUFF_SIZE 	((FLEX_OLED_WIDTH*FLEX_OLED_HEIGHT)/8)

#define BLACK 0
#define WHITE 1

#define FONTHEADERSIZE    	6
#define TOTALFONTS    		2

#define NORM        0
#define XOR         1

#define CLEAR_ALL         0
#define CLEAR_DISPLAY     1
#define CLEAR_BUFFER      2

#define MEMORYMODE          0x20
#define SETCOLUMN           0x21
#define SETROW              0x22
#define SETPORTRAIT         0x25
#define SETCONTRAST         0x81
#define SETCHARGEPMP1       0x8D
#define SETSEGREMAP         0xA0
#define SETSTARTLINE        0xA2
#define RESETALLON          0xA4
#define DISPLAYALLON        0xA5
#define RESETINVERT         0xA6
#define INVERTDISPLAY       0xA7
#define SETMULTIPLEX        0xA8
#define SETCHARGEPMP2       0xAC
#define SELECTIREF          0xAD
#define DISPLAYOFF          0xAE
#define DISPLAYON           0xAF
#define SETPRECHARGE        0xBC
#define SETGSTABLE          0xBE
#define SETDEFAULTTABLE     0xBF
#define COMSCANINC          0xC0
#define COMSCANDEC          0xC8
#define SETDISPLAYOFFSET    0xD3
#define SETDISPLAYCLOCKDIV  0xD5
#define SETPHASELENGTH      0xD9
#define SETCOMPINS          0xDA
#define SETVCOMDESELECT     0xDB
#define SETCOMMANDLOCK      0xFD

// Scroll - It's not documented in the SSD1320 doc but we
// guessed at it from the SSD1306
#define ACTIVATESCROLL                0x2F
#define DEACTIVATESCROLL              0x2E
#define SETVERTICALSCROLLAREA         0xA3
#define RIGHTHORIZONTALSCROLL         0x26
#define LEFTHORIZONTALSCROLL          0x27
#define VERTICALRIGHTHORIZONTALSCROLL 0x29
#define VERTICALLEFTHORIZONTALSCROLL  0x2A

typedef enum CMD
{
	  CMD_CLEAR 		= 0,
	  CMD_INVERT		= 1,
	  CMD_CONTRAST		= 2,
	  CMD_DISPLAY		= 3,
	  CMD_SETCURSOR		= 4,
	  CMD_PIXEL			= 5,
	  CMD_LINE			= 6,
	  CMD_LINEH			= 7,
	  CMD_LINEV			= 8,
	  CMD_RECT			= 9,
	  CMD_RECTFILL		= 10,
	  CMD_CIRCLE		= 11,
	  CMD_CIRCLEFILL	= 12,
	  CMD_DRAWCHAR		= 13,
	  CMD_DRAWBITMAP	= 14,
	  CMD_GETLCDWIDTH	= 15,
	  CMD_GETLCDHEIGHT	= 16,
	  CMD_SETCOLOR		= 17,
	  CMD_SETDRAWMODE	= 18
}
commCommand_t;


typedef enum
{
	font5x7_t  = 0,
	font8x16_t = 1
}
fontType_enum;

typedef struct
{
	uint16_t cursorX;
	uint16_t cursorY;
	uint16_t fontMapWidth;
	uint8_t foreColor;
	uint8_t drawMode;
	uint8_t fontWidth;
	uint8_t fontHeight;
	uint8_t fontType;
	uint8_t fontStartChar;
	uint8_t fontTotalChar;

}
SSD1320_t;



// RAW LCD functions
HAL_StatusTypeDef ssd1320_WriteCommand(uint8_t cmd);
HAL_StatusTypeDef ssd1320_WriteData(uint8_t cmd);

//static const unsigned char *fontsPointer[];


void Flex_OLED_Init(void);
void Flex_OLED_setContrast(uint8_t contrast);
void Flex_OLED_setCursor(uint8_t x, uint8_t y);
void Flex_OLED_setFontType(fontType_enum type);

uint8_t Flex_OLED_clearDisplay(uint8_t mode);
void Flex_OLED_Update(void);
void Flex_OLED_Invert(uint8_t inv);

void Flex_OLED_scrollStop(void);
void Flex_OLED_scrollRight(uint8_t start, uint8_t stop);
void Flex_OLED_scrollLeft(uint8_t start, uint8_t stop);

void  Flex_OLED_drawChar(uint8_t x, uint8_t y, uint8_t c, uint8_t color, uint8_t mode);
void Flex_OLED_setPixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode);
void Flex_OLED_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color, uint8_t mode);
void Flex_OLED_lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, uint8_t mode);
void Flex_OLED_lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, uint8_t mode);
void Flex_OLED_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);
void Flex_OLED_rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color , uint8_t mode);

uint8_t Flex_OLED_StartupAnimation(void);
//void Flex_OLED_Menus_Sensors(Sensors_st sensor);



#endif /* SSD1320_INC_SSD1320_H_ */
