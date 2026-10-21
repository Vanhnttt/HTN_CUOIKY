#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"

/* ---------------- Cấu hình màn hình ---------------- */
#define LCD_W        160
#define LCD_H        128
#define LCD_MADCTL   0x60
#define LCD_XOFF     0
#define LCD_YOFF     0

/* ---------------- Màu sắc RGB565 (Byte-swapped cho SPI MSB first) ---------------- */
#define RGB565(r,g,b) ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)))
#define SW16(c)       ((uint16_t)(((c)<<8)|((c)>>8)))

#define C_BLACK    SW16(RGB565(  0,  0,  0))
#define C_WHITE    SW16(RGB565(255,255,255))
#define C_GREEN    SW16(RGB565(  0,220,  0))
#define C_RED      SW16(RGB565(255,  0,  0))
#define C_CYAN     SW16(RGB565(  0,200,200))
#define C_YELLOW   SW16(RGB565(255,220,  0))
#define C_GRAY     SW16(RGB565( 45, 45, 45))
#define C_DARK     SW16(RGB565( 18, 18, 28))
#define C_ORANGE   SW16(RGB565(255,130,  0))
#define C_TRIG     SW16(RGB565(255, 60, 60))
#define C_GRID     SW16(RGB565( 85, 85,110))
#define C_AXIS     SW16(RGB565(150,150,175))

/* ---------------- Hàm giao tiếp ST7735 ---------------- */
void ST7735_Init(void);
void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7735_WriteData(const uint8_t *d, uint16_t n);
void ST7735_Clear(uint16_t color);
uint8_t ST7735_TextHit(int px, int py, int sx, int sy, const char *s);

#endif /* __ST7735_H */
