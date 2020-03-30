#ifndef SSD0306_H_
#define SSD0306_H_

#include "stm32f4xx_hal.h"
#include "PlatformType.h"
#include "fonts.h"

extern I2C_HandleTypeDef	hi2c1;

#define SSD1306_I2C_PORT	hi2c1
#define SSD1306_LEFT        0x7A
#define SSD1306_RIGHT       0x78

/* real resolution 64x48 */
#define SSD1306_WIDTH       96
#define SSD1306_HEIGHT      64


#define swap(a, b) { uint8_t t = a; a = b; b = t; }
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

//  Enumeration for screen colors
typedef enum
{
	Black = 0x00, // Black color, no pixel
	White = 0x01  // Pixel is set. Color depends on LCD
}
SSD1306_COLOR;

//  Struct to store transformations
typedef struct
{
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
}
SSD1306_LEFT_st;

typedef struct
{
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
}
SSD1306_RIGHT_st;



extern unsigned char EyeL[];
extern unsigned char EyeR[];

extern unsigned char EyeL_v1[];
extern unsigned char EyeR_v1[];



//  Function definitions

HAL_StatusTypeDef SSD1306_WriteCmdRight(uint8_t command);
HAL_StatusTypeDef SSD1306_WriteCmdLeft(uint8_t command);
uint8_t SSD1306_Init(void);

void SSD1306_SetContrastLeft(uint8_t contrast);
void SSD1306_SetContrastRight(uint8_t contrast);
void SSD1306_dim(uint8_t start, uint8_t end, uint8_t speed);

void SSD1306_FillLeft(SSD1306_COLOR color);
void SSD1306_FillRight(SSD1306_COLOR color);

HAL_StatusTypeDef SSD1306_UpdateLeft(void);
HAL_StatusTypeDef SSD1306_UpdateRight(void);

void SSD1306_SetCursorLeft(uint8_t x, uint8_t y);
void SSD1306_SetCursorRight(uint8_t x, uint8_t y);


void SSD1306_DrawPixelLeft(uint8_t x, uint8_t y, SSD1306_COLOR color);
void SSD1306_DrawPixelRight(uint8_t x, uint8_t y, SSD1306_COLOR color);

char SSD1306_WriteCharLeft(char ch, FontDef Font, SSD1306_COLOR color);
char SSD1306_WriteCharRight(char ch, FontDef Font, SSD1306_COLOR color);
char SSD1306_WriteStringLeft(char* str, FontDef Font, SSD1306_COLOR color);
char SSD1306_WriteStringRight(char* str, FontDef Font, SSD1306_COLOR color);

void SSD1306_LineLeft(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void SSD1306_LineRight(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

void SSD1306_DrawBitmapLeft(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
void SSD1306_DrawBitmapRight(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);


void SSD1306_drawFastVLineLeft(int16_t x, int16_t y, int16_t h, uint16_t color);
void SSD1306_drawFastVLineRight(int16_t x, int16_t y, int16_t h, uint16_t color);
void SSD1306_fillRectLeft(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void SSD1306_fillRectLeft(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void SSD1306_fillRectRight(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void SSD1306_fillCircleLeft(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void SSD1306_fillCircleRight(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void SSD1306_fillRoundRectLeft(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void SSD1306_fillRoundRectRight(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);

#endif
