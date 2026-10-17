
#include "lcd_display.h"
#include <string.h>

/* ST7735: SPI1, DC=PB0, RST=PB1, CS=PB10 */
#define LCD_W 160
#define LCD_H 128
#define STRIP_W 16
#define STRIP_PIXELS (LCD_H * STRIP_W)

#define LCD_CS_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define LCD_CS_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define LCD_DC_CMD()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define LCD_DC_DATA() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define LCD_RST_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define LCD_RST_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)

#define LCD_MADCTL 0x60
#define LCD_XOFF 0
#define LCD_YOFF 0

/* RGB565, doi byte theo code goc cua nhom */
#define RGB565(r,g,b) ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)))
#define SW16(c) ((uint16_t)(((c)<<8)|((c)>>8)))
#define C_BLACK SW16(RGB565(0,0,0))
#define C_RED   SW16(RGB565(255,0,0))
#define C_GREEN SW16(RGB565(0,220,0))
#define C_BLUE  SW16(RGB565(0,0,255))
#define C_WHITE SW16(RGB565(255,255,255))

extern SPI_HandleTypeDef hspi1;

static uint16_t strip_buf[STRIP_PIXELS];

static void lcd_cmd(uint8_t c)
{
    LCD_CS_L();
    LCD_DC_CMD();
    HAL_SPI_Transmit(&hspi1, &c, 1, 100);
    LCD_CS_H();
}

static void lcd_data_raw(const uint8_t *d, uint16_t n)
{
    LCD_CS_L();
    LCD_DC_DATA();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)d, n, 500);
    LCD_CS_H();
}

static void lcd_cmd_data(uint8_t c, const uint8_t *d, uint8_t n)
{
    lcd_cmd(c);
    if (n > 0)
        lcd_data_raw(d, n);
}

static void lcd_set_window(uint16_t x0, uint16_t y0,
                           uint16_t x1, uint16_t y1)
{
    uint8_t b[4];

    x0 += LCD_XOFF;
    x1 += LCD_XOFF;
    y0 += LCD_YOFF;
    y1 += LCD_YOFF;

    b[0] = x0 >> 8; b[1] = x0;
    b[2] = x1 >> 8; b[3] = x1;
    lcd_cmd_data(0x2A, b, 4);

    b[0] = y0 >> 8; b[1] = y0;
    b[2] = y1 >> 8; b[3] = y1;
    lcd_cmd_data(0x2B, b, 4);

    lcd_cmd(0x2C);
}

void LCD_Display_Init(void)
{
    LCD_CS_H();
    LCD_DC_DATA();
    LCD_RST_H();
    HAL_Delay(10);

    LCD_RST_L();
    HAL_Delay(20);
    LCD_RST_H();
    HAL_Delay(120);

    lcd_cmd(0x01);
    HAL_Delay(150);

    lcd_cmd(0x11);
    HAL_Delay(150);

    lcd_cmd_data(0xB1, (uint8_t[]){0x01,0x2C,0x2D}, 3);
    lcd_cmd_data(0xB2, (uint8_t[]){0x01,0x2C,0x2D}, 3);
    lcd_cmd_data(0xB3, (uint8_t[]){0x01,0x2C,0x2D,0x01,0x2C,0x2D}, 6);
    lcd_cmd_data(0xB4, (uint8_t[]){0x07}, 1);
    lcd_cmd_data(0xC0, (uint8_t[]){0xA2,0x02,0x84}, 3);
    lcd_cmd_data(0xC1, (uint8_t[]){0xC5}, 1);
    lcd_cmd_data(0xC2, (uint8_t[]){0x0A,0x00}, 2);
    lcd_cmd_data(0xC3, (uint8_t[]){0x8A,0x2A}, 2);
    lcd_cmd_data(0xC4, (uint8_t[]){0x8A,0xEE}, 2);
    lcd_cmd_data(0xC5, (uint8_t[]){0x0E}, 1);

    lcd_cmd(0x20);
    lcd_cmd_data(0x36, (uint8_t[]){LCD_MADCTL}, 1);
    lcd_cmd_data(0x3A, (uint8_t[]){0x05}, 1);

    lcd_cmd_data(0xE0, (uint8_t[]){
        0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
        0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10
    }, 16);

    lcd_cmd_data(0xE1, (uint8_t[]){
        0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
        0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10
    }, 16);

    lcd_cmd(0x13);
    HAL_Delay(10);
    lcd_cmd(0x29);
    HAL_Delay(100);
}

void LCD_Display_Clear(uint16_t color)
{
    uint16_t i;
    uint16_t s;

    for (i = 0; i < STRIP_PIXELS; i++)
        strip_buf[i] = color;

    for (s = 0; s < LCD_W / STRIP_W; s++)
    {
        lcd_set_window(s * STRIP_W, 0,
                       s * STRIP_W + STRIP_W - 1,
                       LCD_H - 1);

        lcd_data_raw((uint8_t *)strip_buf,
                     (uint16_t)(STRIP_PIXELS * 2));
    }
}

void LCD_Display_Test(void)
{
    LCD_Display_Clear(C_BLACK);
    HAL_Delay(500);
    LCD_Display_Clear(C_RED);
    HAL_Delay(500);
    LCD_Display_Clear(C_GREEN);
    HAL_Delay(500);
    LCD_Display_Clear(C_BLUE);
    HAL_Delay(500);
    LCD_Display_Clear(C_WHITE);
}