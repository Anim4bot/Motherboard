#ifndef OLEDS_SSD1320_INC_FONT_7SEG_H_
#define OLEDS_SSD1320_INC_FONT_7SEG_H_

static const uint8_t font7Seg [] =
{
	// first row defines - FONTWIDTH, FONTHEIGHT, ASCII START CHAR, TOTAL CHARACTERS, FONT MAP WIDTH HIGH, FONT MAP WIDTH LOW (2,56 meaning 256)
	10, 16, 46, 12, 1, 20,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x78, 0xFC, 0x02, 0x03, 0x03, 0x03, 0x03, 0x02, 0xFC, 0x78, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x7E, 0x00, 0x00, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02,
	0xFC, 0x78, 0x00, 0x00, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0xFC, 0x78, 0x7E, 0xFF, 0x00, 0x80,
	0x80, 0x80, 0x80, 0x00, 0xFF, 0x7E, 0x78, 0xFC, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0x00, 0x00,
	0x78, 0xFC, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0x00, 0x00, 0x00, 0x02, 0x03, 0x03, 0x03, 0x03,
	0x03, 0x02, 0xFC, 0x78, 0x78, 0xFC, 0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0xFC, 0x78, 0x78, 0xFC,
	0x02, 0x83, 0x83, 0x83, 0x83, 0x02, 0xFC, 0x78, 0x00, 0x00, 0x00, 0x60, 0xF0, 0xF0, 0x60, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x3F, 0x40, 0xC0,
	0xC0, 0xC0, 0xC0, 0x40, 0x3F, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x7E,
	0x1C, 0x3E, 0x41, 0xC1, 0xC1, 0xC1, 0xC1, 0x41, 0x00, 0x00, 0x00, 0x00, 0x41, 0xC1, 0xC1, 0xC1,
	0xC1, 0x41, 0x3E, 0x1C, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0x7E, 0x00, 0x00,
	0x41, 0xC1, 0xC1, 0xC1, 0xC1, 0x41, 0x3E, 0x1C, 0x1C, 0x3E, 0x41, 0xC1, 0xC1, 0xC1, 0xC1, 0x41,
	0x3E, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x7E, 0x1C, 0x3E, 0x41, 0xC1,
	0xC1, 0xC1, 0xC1, 0x41, 0x3E, 0x1C, 0x00, 0x00, 0x41, 0xC1, 0xC1, 0xC1, 0xC1, 0x41, 0x3E, 0x1C
};


#endif /* OLEDS_SSD1320_INC_FONT_7SEG_H_ */
