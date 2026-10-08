/* vim: set ai et ts=4 sw=4: */
#include "stm32f1xx_hal.h"
#include "st7735.h"
#include <stdlib.h>
#include <string.h>

#define DELAY 0x80

// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {
    15,
    ST7735_SWRESET,   DELAY,
      150,
    ST7735_SLPOUT ,   DELAY,
      255,
    ST7735_FRMCTR1, 3      ,
      0x01, 0x2C, 0x2D,
    ST7735_FRMCTR2, 3      ,
      0x01, 0x2C, 0x2D,
    ST7735_FRMCTR3, 6      ,
      0x01, 0x2C, 0x2D,
      0x01, 0x2C, 0x2D,
    ST7735_INVCTR , 1      ,
      0x07,
    ST7735_PWCTR1 , 3      ,
      0xA2,
      0x02,
      0x84,
    ST7735_PWCTR2 , 1      ,
      0xC5,
    ST7735_PWCTR3 , 2      ,
      0x0A,
      0x00,
    ST7735_PWCTR4 , 2      ,
      0x8A,
      0x2A,
    ST7735_PWCTR5 , 2      ,
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,
      0x0E,
    ST7735_INVOFF , 0      ,
    ST7735_MADCTL , 1      ,
      ST7735_ROTATION,
    ST7735_COLMOD , 1      ,
      0x05 },

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {
    2,
    ST7735_CASET  , 4      ,
      0x00, 0x00,
      0x00, 0x7F,
    ST7735_RASET  , 4      ,
      0x00, 0x00,
      0x00, 0x7F },
#endif

#ifdef ST7735_IS_160X80
  init_cmds2[] = {
    3,
    ST7735_CASET  , 4      ,
      0x00, 0x00,
      0x00, 0x4F,
    ST7735_RASET  , 4      ,
      0x00, 0x00,
      0x00, 0x9F ,
    ST7735_INVON, 0 },
#endif

  init_cmds3[] = {
    4,
    ST7735_GMCTRP1, 16      ,
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY,
      10,
    ST7735_DISPON ,    DELAY,
      100 };

static void ST7735_Select(void) {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

void ST7735_Unselect(void) {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset(void) {
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
}

static void ST7735_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

static void ST7735_WriteData(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
}

static void ST7735_ExecuteCommandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    ST7735_WriteData(data, sizeof(data));

    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + ST7735_YSTART;
    data[3] = y1 + ST7735_YSTART;
    ST7735_WriteData(data, sizeof(data));

    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init(void) {
    ST7735_Select();
    ST7735_Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    ST7735_Select();

    ST7735_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(data, sizeof(data));

    ST7735_Unselect();
}

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7735_WriteData(data, sizeof(data));
            }
        }
    }
}

void ST7735_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select();

    while(*str) {
        if(x + font.width >= ST7735_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                str++;
                continue;
            }
        }

        ST7735_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect();
}

void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

    ST7735_Unselect();
}

void ST7735_FillRectangleFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t pixel[] = { color >> 8, color & 0xFF };
    uint8_t *line = malloc(w * sizeof(pixel));
    for(x = 0; x < w; ++x)
        memcpy(line + x * sizeof(pixel), pixel, sizeof(pixel));

    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--)
        HAL_SPI_Transmit(&ST7735_SPI_PORT, line, w * sizeof(pixel), HAL_MAX_DELAY);

    free(line);
    ST7735_Unselect();
}

void ST7735_FillScreen(uint16_t color) {
    ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_FillScreenFast(uint16_t color) {
    ST7735_FillRectangleFast(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect();
}

void ST7735_InvertColors(bool invert) {
    ST7735_Select();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect();
}

void ST7735_SetGamma(GammaDef gamma)
{
    ST7735_Select();
    ST7735_WriteCommand(ST7735_GAMSET);
    ST7735_WriteData((uint8_t *) &gamma, sizeof(gamma));
    ST7735_Unselect();
}
