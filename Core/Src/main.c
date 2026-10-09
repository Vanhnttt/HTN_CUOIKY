/* ============================================================
 * Oscilloscope STM32F103C8T6 + ST7735 1.8" (160x128 landscape)
 *
 * ADC1_IN0  PA0  <- TIM3 TRGO -> DMA1 Ch1 -> adc_buf[] (circular)
 * TIM1_CH1  PA8  -> PWM test 1 kHz (connect PA8 -> PA0 to test)
 * TIM4      PB6/PB7 -> Rotary encoder
 * Enc SW    PB8   short press = next adjust mode  (TIME > VOLT > TRIG > POS)
 *                 long  press = OSC <-> FFT
 * HOLD/RUN  PB9   (button to GND)
 *
 * ST7735 SPI1: SCK=PA5, MOSI=PA7, DC=PB0, RST=PB1, CS=PB10
 *
 * ADJUST MODES (turn the encoder to change the selected one)
 *   TIME  time/div   (sample rate)             border cyan
 *   VOLT  volt/div   (vertical zoom)           border green
 *   TRIG  trigger level                        border red
 *   POS   vertical position (moves the 0 V ground marker)   border orange
 *  The item being adjusted gets a filled cursor box in the mode colour
 *  (TIME/VOLT/TRIG in the header, POS = "P1.65V" at the bottom right).
 *
 * DMA1_Channel1_IRQHandler is defined at the bottom when
 * DMA_HANDLER_IN_MAIN = 1. If the linker complains about a duplicate
 * definition (stm32f1xx_it.c already has it) set it to 0.
 * ============================================================ */

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------------- user options ---------------- */
#define DMA_HANDLER_IN_MAIN  0
#define DEBUG_PINS           0      /* 1 = show raw pin/encoder state in footer */

/* ST7735 MADCTL (0x36):
 *   0xA0 landscape            0x60 landscape rotated 180 deg
 *   0x20 landscape mirrored   0xE0 rotated 180 + mirrored
 *   add 0x08 if red/blue are swapped                                   */
#define LCD_MADCTL   0x60

#define LCD_XOFF     0
#define LCD_YOFF     0

/* set to -1 if turning the knob clockwise goes the wrong way */
#define ENC_DIR      1

/* ---- Analog front end (J2 -> divider -> LM358 -> PA0) calibration ----
 * Measure with a multimeter on PA0 (SW1 on DC):
 *   AFE_ZERO_MV     : PA0 voltage in mV when J2 is shorted to GND
 *   AFE_GAIN_X1000  : (PA0 change in V per 1 V at J2) x 1000   (always positive)
 *   AFE_INVERT      : 1 if PA0 goes DOWN when the J2 voltage goes UP
 * Example: J2 = 0 V -> PA0 = 1.65 V ; J2 = 5 V -> PA0 = 1.45 V
 *          => AFE_ZERO_MV 1650, AFE_GAIN_X1000 40, AFE_INVERT 1
 * Defaults below = PA0 connected straight to the signal (no front end). */
#define AFE_INVERT        0
#define AFE_ZERO_MV       0
#define AFE_GAIN_X1000    1000

/* ============================================================
 * HAL HANDLES
 * ============================================================ */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1, htim3, htim4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* ============================================================
 * SCREEN LAYOUT
 *  y=0..8    Header
 *  y=9       Border (colour = adjust mode)
 *  y=10..105 Scope   (6 DIV x 16 px)
 *  y=106     Border
 *  y=107..127 Footer (2 text rows)
 * ============================================================ */
#define LCD_W        160
#define LCD_H        128

#define HDR_Y1       8
#define BRD_TOP      9
#define SCO_Y0       10
#define SCO_Y1       105
#define SCO_H        96
#define BRD_BOT      106
#define FTR_Y0       107

#define DIV_W        16
#define DIV_H        16
#define SCO_CX       80
#define SCO_CY       58

#define STRIP_W      16
#define STRIP_PIXELS (LCD_H * STRIP_W)          /* 2048 px = 4096 byte */

#define ADC_BUF_LEN  512
#define FFT_N        256

#define ADC_VREF     3.3f
#define ADC_MAX      4095.0f

#define FFT_FLOOR    1024.0f

/* ============================================================
 * LCD PIN MACROS
 * ============================================================ */
#define LCD_CS_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define LCD_CS_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define LCD_DC_CMD()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET)
#define LCD_DC_DATA() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET)
#define LCD_RST_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_RESET)
#define LCD_RST_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  GPIO_PIN_SET)

/* ============================================================
 * COLOURS (RGB565, byte-swapped for SPI MSB first)
 * ============================================================ */
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
#define C_GRID     SW16(RGB565( 85, 85,110))   /* dotted division lines   */
#define C_AXIS     SW16(RGB565(150,150,175))   /* centre axes + crossings */

/* ============================================================
 * TIME / DIV  (TIM3 prescaler=71 -> 1 tick = 1 us)
 *  idx | ARR  | fs       | T/DIV (16 samples)
 *   0  |    1 | 500 kHz  |  32 us
 *   1  |    4 | 200 kHz  |  80 us
 *   2  |    9 | 100 kHz  | 160 us
 *   3  |   19 |  50 kHz  | 320 us
 *   4  |   49 |  20 kHz  | 800 us
 *   5  |   99 |  10 kHz  | 1.6 ms
 *   6  |  199 |   5 kHz  | 3.2 ms
 *   7  |  499 |   2 kHz  |   8 ms
 *   8  |  999 |   1 kHz  |  16 ms
 *   9  | 1999 | 500  Hz  |  32 ms
 * ============================================================ */
static const uint16_t tb_arr[] = { 1, 4, 9, 19, 49, 99, 199, 499, 999, 1999 };
#define TB_COUNT ((int)(sizeof(tb_arr)/sizeof(tb_arr[0])))

/* ============================================================
 * VOLT / DIV  (digital vertical zoom, in millivolts per division)
 * The ADC input range is 0..3.3 V, so there is no analog gain:
 * smaller V/div just magnifies the picture. Use POS to move it.
 * ============================================================ */
static const uint16_t vdiv_mv[] = { 2000, 1000, 500, 200, 100, 50, 20 };
#define VD_COUNT ((int)(sizeof(vdiv_mv)/sizeof(vdiv_mv[0])))

/* adjust modes */
enum { MODE_TIME = 0, MODE_VOLT, MODE_TRIG, MODE_POS, MODE_COUNT };
static const uint16_t    mode_col [MODE_COUNT] = { C_CYAN, C_GREEN, C_RED, C_ORANGE };

/* ============================================================
 * STATE
 * ============================================================ */
volatile uint16_t adc_buf[ADC_BUF_LEN];

static volatile uint8_t  dma_half = 0;
static volatile uint32_t dma_seq  = 0;
static uint32_t          used_seq = 0;

static uint16_t snap[FFT_N];
static uint8_t  snap_tb = 5;

static uint8_t  tb_idx      = 5;          /* 1.6 ms/div  */
static uint8_t  vd_idx      = 1;          /* 1 V/div */
static int16_t  center_mv   = 0;          /* voltage at the centre row (0 = ground at centre, like a real scope) */
static uint16_t trig_lvl    = 2048;
static uint8_t  adj_mode    = MODE_TIME;
static uint8_t  hold        = 0;
static uint8_t  mode_fft    = 0;
static uint8_t  need_redraw = 1;

static volatile uint8_t req_edit = 0, req_fft = 0, req_hold = 0;

/* measurements */
static float     v_min = 0.0f, v_max = 0.0f, v_pp = 0.0f;
static uint32_t  sig_freq_hz = 0;

static uint8_t  wave_lo[LCD_W];
static uint8_t  wave_hi[LCD_W];

/* FFT */
static float    fft_re[FFT_N];
static float    fft_im[FFT_N];
static float    fft_win[FFT_N];
static float    tw_re[FFT_N/2];
static float    tw_im[FFT_N/2];
static uint8_t  fft_mag[LCD_W];
static uint32_t fft_peak_hz = 0;

/* OSD strings */
static char     osd_vdiv[16];
static char     osd_tdiv[16];
static char     osd_trig[24];
static char     osd_mode[8];
static char     osd_f1[64];
static char     osd_f2[64];
static char     osd_pos[16];                 /* vertical position, e.g. P1.65V */
static uint16_t border_col = C_CYAN;

/* text x positions (shared by compose() and the cursor box) */
#define HDR_VDIV_X   1
#define HDR_TDIV_X   42
#define HDR_TRIG_X   84
#define HDR_MODE_X   124
#define FTR_POS_X    112

/* cursor box drawn behind the item that the knob is adjusting */
static uint8_t  hl_on = 0;
static int      hl_x0 = 0, hl_x1 = 0, hl_y0 = 0, hl_y1 = 0;
static uint16_t hl_col = C_CYAN;

/* trigger arrow colour: red = visible, yellow = outside the visible window */
static uint16_t trig_col = C_TRIG;

/* ground (0 V) marker: row + colour (yellow = outside the visible window) */
static int      gnd_y   = SCO_CY;
static uint16_t gnd_col = C_GREEN;

static uint16_t strip_buf[STRIP_PIXELS];

/* ============================================================
 * FONT 5x7 (ASCII 32..122)  byte n = column n, bit0 = top row
 * ============================================================ */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 32   */
    {0x00,0x00,0x5F,0x00,0x00}, /* 33 ! */
    {0x00,0x07,0x00,0x07,0x00}, /* 34 " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 35 # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 36 $ */
    {0x23,0x13,0x08,0x64,0x62}, /* 37 % */
    {0x36,0x49,0x55,0x22,0x50}, /* 38 & */
    {0x00,0x05,0x03,0x00,0x00}, /* 39 ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* 40 ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* 41 ) */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* 42 * */
    {0x08,0x08,0x3E,0x08,0x08}, /* 43 + */
    {0x00,0x50,0x30,0x00,0x00}, /* 44 , */
    {0x08,0x08,0x08,0x08,0x08}, /* 45 - */
    {0x00,0x60,0x60,0x00,0x00}, /* 46 . */
    {0x20,0x10,0x08,0x04,0x02}, /* 47 / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 48 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 49 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 50 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 51 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 52 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 53 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 54 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 55 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 56 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 57 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* 58 : */
    {0x00,0x56,0x36,0x00,0x00}, /* 59 ; */
    {0x00,0x08,0x14,0x22,0x41}, /* 60 < */
    {0x14,0x14,0x14,0x14,0x14}, /* 61 = */
    {0x41,0x22,0x14,0x08,0x00}, /* 62 > */
    {0x02,0x01,0x51,0x09,0x06}, /* 63 ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* 64 @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 65 A */
    {0x7F,0x49,0x49,0x49,0x36}, /* 66 B */
    {0x3E,0x41,0x41,0x41,0x22}, /* 67 C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 68 D */
    {0x7F,0x49,0x49,0x49,0x41}, /* 69 E */
    {0x7F,0x09,0x09,0x01,0x01}, /* 70 F */
    {0x3E,0x41,0x41,0x51,0x32}, /* 71 G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 72 H */
    {0x00,0x41,0x7F,0x41,0x00}, /* 73 I */
    {0x20,0x40,0x41,0x3F,0x01}, /* 74 J */
    {0x7F,0x08,0x14,0x22,0x41}, /* 75 K */
    {0x7F,0x40,0x40,0x40,0x40}, /* 76 L */
    {0x7F,0x02,0x04,0x02,0x7F}, /* 77 M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 78 N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 79 O */
    {0x7F,0x09,0x09,0x09,0x06}, /* 80 P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 81 Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* 82 R */
    {0x46,0x49,0x49,0x49,0x31}, /* 83 S */
    {0x01,0x01,0x7F,0x01,0x01}, /* 84 T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 85 U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 86 V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* 87 W */
    {0x63,0x14,0x08,0x14,0x63}, /* 88 X */
    {0x03,0x04,0x78,0x04,0x03}, /* 89 Y */
    {0x61,0x51,0x49,0x45,0x43}, /* 90 Z */
    {0x00,0x7F,0x41,0x41,0x00}, /* 91 [ */
    {0x02,0x04,0x08,0x10,0x20}, /* 92 \ */
    {0x00,0x41,0x41,0x7F,0x00}, /* 93 ] */
    {0x04,0x02,0x01,0x02,0x04}, /* 94 ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* 95 _ */
    {0x00,0x01,0x02,0x04,0x00}, /* 96 ` */
    {0x20,0x54,0x54,0x54,0x78}, /* 97  a */
    {0x7F,0x48,0x44,0x44,0x38}, /* 98  b */
    {0x38,0x44,0x44,0x44,0x20}, /* 99  c */
    {0x38,0x44,0x44,0x48,0x7F}, /* 100 d */
    {0x38,0x54,0x54,0x54,0x18}, /* 101 e */
    {0x08,0x7E,0x09,0x01,0x02}, /* 102 f */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 103 g */
    {0x7F,0x08,0x04,0x04,0x78}, /* 104 h */
    {0x00,0x44,0x7D,0x40,0x00}, /* 105 i */
    {0x20,0x40,0x44,0x3D,0x00}, /* 106 j */
    {0x7F,0x10,0x28,0x44,0x00}, /* 107 k */
    {0x00,0x41,0x7F,0x40,0x00}, /* 108 l */
    {0x7C,0x04,0x18,0x04,0x78}, /* 109 m */
    {0x7C,0x08,0x04,0x04,0x78}, /* 110 n */
    {0x38,0x44,0x44,0x44,0x38}, /* 111 o */
    {0x7C,0x14,0x14,0x14,0x08}, /* 112 p */
    {0x08,0x14,0x14,0x18,0x7C}, /* 113 q */
    {0x7C,0x08,0x04,0x04,0x08}, /* 114 r */
    {0x48,0x54,0x54,0x54,0x20}, /* 115 s */
    {0x04,0x3F,0x44,0x40,0x20}, /* 116 t */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 117 u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 118 v */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 119 w */
    {0x44,0x28,0x10,0x28,0x44}, /* 120 x */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 121 y */
    {0x44,0x64,0x54,0x4C,0x44}, /* 122 z */
};
#define FONT_FIRST 32
#define FONT_LAST  122
#define FONT_W     5
#define FONT_H     7
#define FONT_GAP   1

/* ============================================================
 * LCD LOW-LEVEL DRIVER
 * ============================================================ */
static void lcd_cmd(uint8_t c)
{
    LCD_CS_L(); LCD_DC_CMD();
    HAL_SPI_Transmit(&hspi1, &c, 1, 10);
    LCD_CS_H();
}
static void lcd_data_raw(const uint8_t *d, uint16_t n)
{
    LCD_CS_L(); LCD_DC_DATA();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)d, n, 200);
    LCD_CS_H();
}
static void lcd_cmd_data(uint8_t c, const uint8_t *d, uint8_t n)
{
    lcd_cmd(c);
    if (n) lcd_data_raw(d, n);
}
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t b[4];
    x0 += LCD_XOFF; x1 += LCD_XOFF;
    y0 += LCD_YOFF; y1 += LCD_YOFF;
    b[0]=x0>>8; b[1]=x0; b[2]=x1>>8; b[3]=x1;
    lcd_cmd_data(0x2A, b, 4);
    b[0]=y0>>8; b[1]=y0; b[2]=y1>>8; b[3]=y1;
    lcd_cmd_data(0x2B, b, 4);
    lcd_cmd(0x2C);
}
static void lcd_init(void)
{
    LCD_CS_H();
    LCD_RST_H(); HAL_Delay(10);
    LCD_RST_L(); HAL_Delay(20);
    LCD_RST_H(); HAL_Delay(120);

    lcd_cmd(0x01); HAL_Delay(150);
    lcd_cmd(0x11); HAL_Delay(150);

    lcd_cmd_data(0xB1,(uint8_t[]){0x01,0x2C,0x2D},3);
    lcd_cmd_data(0xB2,(uint8_t[]){0x01,0x2C,0x2D},3);
    lcd_cmd_data(0xB3,(uint8_t[]){0x01,0x2C,0x2D,0x01,0x2C,0x2D},6);
    lcd_cmd_data(0xB4,(uint8_t[]){0x07},1);
    lcd_cmd_data(0xC0,(uint8_t[]){0xA2,0x02,0x84},3);
    lcd_cmd_data(0xC1,(uint8_t[]){0xC5},1);
    lcd_cmd_data(0xC2,(uint8_t[]){0x0A,0x00},2);
    lcd_cmd_data(0xC3,(uint8_t[]){0x8A,0x2A},2);
    lcd_cmd_data(0xC4,(uint8_t[]){0x8A,0xEE},2);
    lcd_cmd_data(0xC5,(uint8_t[]){0x0E},1);
    lcd_cmd(0x20);
    lcd_cmd_data(0x36,(uint8_t[]){LCD_MADCTL},1);
    lcd_cmd_data(0x3A,(uint8_t[]){0x05},1);   /* RGB565 */
    lcd_cmd_data(0xE0,(uint8_t[]){
        0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
        0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10},16);
    lcd_cmd_data(0xE1,(uint8_t[]){
        0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
        0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10},16);
    lcd_cmd(0x13); HAL_Delay(10);
    lcd_cmd(0x29); HAL_Delay(100);
}

static void lcd_clear(uint16_t color)
{
    for (int i = 0; i < STRIP_PIXELS; i++) strip_buf[i] = color;
    for (int s = 0; s < LCD_W / STRIP_W; s++)
    {
        lcd_set_window((uint16_t)(s * STRIP_W), 0,
                       (uint16_t)(s * STRIP_W + STRIP_W - 1), LCD_H - 1);
        lcd_data_raw((uint8_t *)strip_buf, (uint16_t)(STRIP_PIXELS * 2));
    }
}

/* is pixel (px,py) foreground of string s drawn at (sx,sy)? */
static uint8_t text_hit(int px, int py, int sx, int sy, const char *s)
{
    int row = py - sy;
    if (row < 0 || row >= FONT_H) return 0;

    while (*s)
    {
        int col = px - sx;
        if (col < 0) break;
        if (col < FONT_W)
        {
            uint8_t c = (uint8_t)*s;
            if (c >= FONT_FIRST && c <= FONT_LAST)
                return (font5x7[c - FONT_FIRST][col] >> row) & 1u;
            return 0;
        }
        sx += FONT_W + FONT_GAP;
        s++;
    }
    return 0;
}

/* ============================================================
 * DMA CALLBACKS (ISR only publishes which half is fresh)
 * ============================================================ */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    dma_half = 0;
    dma_seq++;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    dma_half = 1;
    dma_seq++;
}

static uint8_t grab_snapshot(void)
{
    uint32_t s1 = dma_seq;
    if (s1 == used_seq) return 0;

    const volatile uint16_t *src = &adc_buf[dma_half ? FFT_N : 0];
    for (int i = 0; i < FFT_N; i++)
    {
        uint16_t v = src[i];
        snap[i] = AFE_INVERT ? (uint16_t)(4095u - v) : v;   /* higher = more positive */
    }

    if (dma_seq != s1) return 0;     /* possibly torn -> retry */
    used_seq = s1;
    snap_tb  = tb_idx;
    return 1;
}

/* ============================================================
 * INTEGER TEXT FORMATTERS (no %f)
 * ============================================================ */
static void fmt_volt(char *d, size_t n, const char *pfx, float v)
{
    if (v < 0.0f) v = 0.0f;
    uint32_t cv = (uint32_t)(v * 100.0f + 0.5f);
    snprintf(d, n, "%s%lu.%02luV", pfx,
             (unsigned long)(cv / 100u), (unsigned long)(cv % 100u));
}
static void fmt_vsigned(char *d, size_t n, const char *pfx, float v)
{
    int32_t cv = (int32_t)(v * 100.0f + (v < 0.0f ? -0.5f : 0.5f));
    uint32_t a = (uint32_t)(cv < 0 ? -cv : cv);
    snprintf(d, n, "%s%s%lu.%02luV", pfx, (cv < 0) ? "-" : "",
             (unsigned long)(a / 100u), (unsigned long)(a % 100u));
}
static void fmt_hz(char *d, size_t n, const char *pfx, uint32_t hz)
{
    if (hz >= 1000u)
        snprintf(d, n, "%s%lu.%02lukHz", pfx,
                 (unsigned long)(hz / 1000u),
                 (unsigned long)((hz % 1000u) / 10u));
    else
        snprintf(d, n, "%s%luHz", pfx, (unsigned long)hz);
}

/* ============================================================
 * MEASUREMENTS
 * ============================================================ */
/* (normalised) ADC count -> signal voltage at J2, in millivolts */
static inline int32_t adc_to_mv(uint16_t raw)
{
    int32_t norm_mv = (int32_t)((uint32_t)raw * 3300u / 4095u);
    int32_t zero_mv = AFE_INVERT ? (3300 - AFE_ZERO_MV) : AFE_ZERO_MV;
    return ((norm_mv - zero_mv) * 1000) / AFE_GAIN_X1000;
}
static inline float adc_to_v(uint16_t raw)
{
    return (float)adc_to_mv(raw) / 1000.0f;
}

static void process_measurements(void)
{
    uint16_t adc_mn = 4095, adc_mx = 0;
    for (int i = 0; i < FFT_N; i++)
    {
        if (snap[i] < adc_mn) adc_mn = snap[i];
        if (snap[i] > adc_mx) adc_mx = snap[i];
    }
    v_min = adc_to_v(adc_mn);
    v_max = adc_to_v(adc_mx);
    v_pp  = v_max - v_min;

    sig_freq_hz = 0;
    if (v_pp < 0.05f) return;

    uint16_t mid  = adc_mn + (adc_mx - adc_mn) / 2;
    uint16_t hyst = (adc_mx - adc_mn) / 10;
    if (hyst < 20) hyst = 20;

    int first = -1, last = -1;
    uint8_t edges = 0, state = (snap[0] >= mid) ? 1 : 0;

    for (int i = 1; i < FFT_N; i++)
    {
        if (state == 0 && snap[i] >= (uint16_t)(mid + hyst))
        {
            state = 1;
            if (first < 0) first = i;
            last = i; edges++;
        }
        else if (state == 1 && snap[i] <= (uint16_t)(mid - hyst))
            state = 0;
    }

    if (edges >= 2 && last > first)
    {
        uint32_t fs  = 1000000u / ((uint32_t)tb_arr[snap_tb] + 1u);
        uint32_t num = fs * (uint32_t)(edges - 1);
        uint32_t den = (uint32_t)(last - first);
        sig_freq_hz  = (num + den / 2u) / den;
    }
}

/* rising-edge trigger search (raw ADC values). Returns the index of the
 * trigger sample; it is drawn at the centre column (t = 0). */
static uint16_t find_trigger(void)
{
    int hyst = (trig_lvl < 60) ? ((int)trig_lvl / 2) : 30;
    for (int i = LCD_W / 2; i < (FFT_N - LCD_W / 2 - 1); i++)
        if ((int)snap[i-1] < ((int)trig_lvl - hyst) &&
            (int)snap[i]  >= (int)trig_lvl)
            return (uint16_t)i;
    return FFT_N / 2;              /* no trigger: free-run, centred */
}

/* ============================================================
 * VOLT/DIV + POSITION mapping:  millivolts -> screen row
 *   y = SCO_CY - (mv - center_mv) * DIV_H / vdiv_mv      (0 V at centre by default)
 * ============================================================ */
static int mv_to_y(int32_t mv)
{
    int32_t y = SCO_CY - ((mv - (int32_t)center_mv) * DIV_H) / (int32_t)vdiv_mv[vd_idx];
    if (y < SCO_Y0) y = SCO_Y0;
    if (y > SCO_Y1) y = SCO_Y1;
    return (int)y;
}
/* keep the ADC range reachable: centre may move 3 div beyond each end */
static void clamp_center(void)
{
    int lo = (int)adc_to_mv(0)    - 3 * (int)vdiv_mv[vd_idx];
    int hi = (int)adc_to_mv(4095) + 3 * (int)vdiv_mv[vd_idx];
    if (lo > hi) { int t = lo; lo = hi; hi = t; }
    int b = (int)center_mv;
    if (b < lo) b = lo;
    if (b > hi) b = hi;
    center_mv = (int16_t)b;
}

static int adc_to_y(uint16_t raw)
{
    return mv_to_y(adc_to_mv(raw));
}

static void precompute_wave(void)
{
    /* trigger sample sits at the centre column -> show LCD_W/2 samples before it */
    uint16_t start = (uint16_t)(find_trigger() - LCD_W / 2);
    int prev_y = adc_to_y(snap[start]);

    for (int x = 0; x < LCD_W; x++)
    {
        uint16_t idx = start + (uint16_t)x;
        if (idx >= FFT_N) idx = FFT_N - 1;
        int cy = adc_to_y(snap[idx]);

        wave_lo[x] = (uint8_t)((cy < prev_y) ? cy : prev_y);
        wave_hi[x] = (uint8_t)((cy > prev_y) ? cy : prev_y);
        prev_y = cy;
    }
}

/* ============================================================
 * FFT (radix-2, precomputed window + twiddles)
 * ============================================================ */
static void fft_init(void)
{
    for (int i = 0; i < FFT_N; i++)
        fft_win[i] = 0.5f * (1.0f - cosf(2.0f*(float)M_PI*(float)i/(float)(FFT_N-1)));
    for (int k = 0; k < FFT_N/2; k++)
    {
        float a = -2.0f*(float)M_PI*(float)k/(float)FFT_N;
        tw_re[k] = cosf(a);
        tw_im[k] = sinf(a);
    }
}

static void compute_fft(void)
{
    float sum = 0.0f;
    for (int i = 0; i < FFT_N; i++) sum += (float)snap[i];
    float mean = sum / (float)FFT_N;

    for (int i = 0; i < FFT_N; i++)
    {
        fft_re[i] = ((float)snap[i] - mean) * fft_win[i];
        fft_im[i] = 0.0f;
    }

    int j = 0;
    for (int i = 0; i < FFT_N - 1; i++)
    {
        if (i < j)
        {
            float t;
            t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;
        }
        int k = FFT_N >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }

    for (int len = 2; len <= FFT_N; len <<= 1)
    {
        int half = len >> 1;
        int step = FFT_N / len;
        for (int i = 0; i < FFT_N; i += len)
        {
            for (int k = 0; k < half; k++)
            {
                float wr = tw_re[k * step];
                float wi = tw_im[k * step];
                int u = i + k, v = u + half;
                float tr = fft_re[v]*wr - fft_im[v]*wi;
                float ti = fft_re[v]*wi + fft_im[v]*wr;
                fft_re[v] = fft_re[u] - tr;  fft_im[v] = fft_im[u] - ti;
                fft_re[u] += tr;             fft_im[u] += ti;
            }
        }
    }

    float best = 0.0f;
    int   peak_bin = 1;
    for (int i = 1; i < FFT_N/2; i++)
    {
        float m = sqrtf(fft_re[i]*fft_re[i] + fft_im[i]*fft_im[i]);
        fft_re[i] = m;
        if (m > best) { best = m; peak_bin = i; }
    }
    float peak = (best > FFT_FLOOR) ? best : FFT_FLOOR;

    for (int x = 0; x < LCD_W; x++)
    {
        int bin = 1 + x * (FFT_N/2 - 2) / (LCD_W - 1);
        int h = (int)(fft_re[bin] / peak * (float)(SCO_H - 1));
        if (h < 0) h = 0;
        if (h >= SCO_H) h = SCO_H - 1;
        fft_mag[x] = (uint8_t)h;
    }

    uint32_t fs = 1000000u / ((uint32_t)tb_arr[snap_tb] + 1u);
    fft_peak_hz = (best >= FFT_FLOOR / 4.0f)
                  ? (fs * (uint32_t)peak_bin) / FFT_N : 0u;
}

/* ============================================================
 * OSD STRINGS
 * ============================================================ */
static void build_osd(void)
{
    uint32_t sample_us = (uint32_t)tb_arr[snap_tb] + 1u;
    uint32_t fs_hz     = 1000000u / sample_us;
    uint32_t tdiv_us   = sample_us * DIV_W;

    border_col = mode_col[adj_mode];

    /* time/div */
    if (tdiv_us >= 1000u)
    {
        uint32_t t10 = tdiv_us / 100u;
        snprintf(osd_tdiv, sizeof(osd_tdiv), "%lu.%lums",
                 (unsigned long)(t10 / 10u), (unsigned long)(t10 % 10u));
    }
    else
        snprintf(osd_tdiv, sizeof(osd_tdiv), "%luus", (unsigned long)tdiv_us);

    if (mode_fft)
    {
        osd_vdiv[0] = '\0';
        osd_trig[0] = '\0';
        osd_pos[0]  = '\0';
        snprintf(osd_mode, sizeof(osd_mode), "FFT");
        fmt_hz(osd_f1, sizeof(osd_f1), "Nyq:", fs_hz / 2u);
        if (fft_peak_hz) fmt_hz(osd_f2, sizeof(osd_f2), "Pk:", fft_peak_hz);
        else             snprintf(osd_f2, sizeof(osd_f2), "Pk:---");
        return;
    }

    /* volt/div, trigger, mode */
    snprintf(osd_vdiv, sizeof(osd_vdiv), "%umV", (unsigned)vdiv_mv[vd_idx]);
    fmt_vsigned(osd_trig, sizeof(osd_trig), "T", adc_to_v(trig_lvl));
    snprintf(osd_mode, sizeof(osd_mode), "OSC");
    fmt_vsigned(osd_pos, sizeof(osd_pos), "C", (float)center_mv / 1000.0f);

    char a[32], b[32], c[32], d[32];
    fmt_volt(a, sizeof(a), "Vpp:", v_pp);
    if (sig_freq_hz) fmt_hz(b, sizeof(b), "F:", sig_freq_hz);
    else             snprintf(b, sizeof(b), "F:---");
    snprintf(osd_f1, sizeof(osd_f1), "%s  %s", a, b);

    fmt_vsigned(c, sizeof(c), "Mn:", v_min);
    fmt_vsigned(d, sizeof(d), "Mx:", v_max);
    snprintf(osd_f2, sizeof(osd_f2), "%s  %s", c, d);
}

/* Work out the cursor box (filled rectangle, colour = adjust-mode colour)
 * around the text item that the encoder is currently changing. */
static void update_highlight(void)
{
    const char *s = NULL;
    int sx = 0, y0 = 0, y1 = HDR_Y1;

    switch (adj_mode)
    {
    case MODE_TIME: s = osd_tdiv; sx = HDR_TDIV_X; break;
    case MODE_VOLT: s = osd_vdiv; sx = HDR_VDIV_X; break;
    case MODE_TRIG: s = osd_trig; sx = HDR_TRIG_X; break;
    case MODE_POS:  s = osd_pos;  sx = FTR_POS_X;
                    y0 = FTR_Y0 + 11; y1 = FTR_Y0 + 19; break;
    default: break;
    }

    hl_on = 0;
    if (s && s[0])
    {
        int w = (int)strlen(s) * (FONT_W + FONT_GAP) - FONT_GAP;
        hl_x0 = sx - 2;
        hl_x1 = sx + w + 1;
        if (hl_x0 < 0) hl_x0 = 0;
        if (hl_x1 > LCD_W - 1) hl_x1 = LCD_W - 1;
        hl_y0 = y0;
        hl_y1 = y1;
        hl_col = mode_col[adj_mode];
        hl_on = 1;
    }
}

/* ============================================================
 * COMPOSE ONE PIXEL
 * ============================================================ */
static inline uint16_t compose(int x, int y, int trig_y)
{
    uint16_t c;

    if (y <= HDR_Y1 || y >= FTR_Y0) c = C_DARK;
    else if (y == BRD_TOP)          c = border_col;
    else if (y == BRD_BOT)          c = C_GRAY;
    else                            c = C_BLACK;

    if (y >= SCO_Y0 && y <= SCO_Y1)
    {
        /* ---- graticule: dotted lines on every division ---- */
        int gy = y - SCO_Y0;
        uint8_t on_v = ((x % DIV_W) == 0) || (x == LCD_W - 1);   /* vertical line   */
        uint8_t on_h = ((gy % DIV_H) == 0);                      /* horizontal line */

        if (on_v && (gy % 4) == 0) c = C_GRID;
        if (on_h && (x  % 4) == 0) c = C_GRID;
        if (on_v && on_h)          c = C_AXIS;                   /* crossings */

        /* centre axes (denser dots) + minor ticks every 1/4 division */
        if (y == SCO_CY && (x % 2) == 0)  c = C_AXIS;
        if (x == SCO_CX && (gy % 2) == 0) c = C_AXIS;
        if (abs(y - SCO_CY) <= 2 && (x  % 4) == 0) c = C_AXIS;
        if (abs(x - SCO_CX) <= 2 && (gy % 4) == 0) c = C_AXIS;

        if (mode_fft)
        {
            int bar_top = SCO_Y1 - fft_mag[x];
            if (y >= bar_top && y <= SCO_Y1)
                c = (y <= bar_top + fft_mag[x]/3) ? C_YELLOW : C_ORANGE;
        }
        else
        {
            if (y >= (int)wave_lo[x] && y <= (int)wave_hi[x]) c = C_GREEN;

            /* ground (0 V) marker, left edge (yellow = outside the window) */
            if (x < 6)
            {
                int dist = abs(y - gnd_y);
                int wing = 2 - x/2;
                if (dist == 0)              c = gnd_col;
                if (x == 0 && dist <= 2)    c = gnd_col;
                if (x <= 2 && dist == wing) c = gnd_col;
            }

            /* trigger level marker, right edge (yellow = outside the window) */
            if (x >= LCD_W - 6)
            {
                int xr   = LCD_W - 1 - x;
                int dist = abs(y - trig_y);
                int wing = 2 - xr/2;
                if (dist == 0)               c = trig_col;
                if (xr == 0 && dist <= 2)    c = trig_col;
                if (xr <= 2 && dist == wing) c = trig_col;
            }

            /* trigger time marker (t = 0) on top of the centre line */
            if (y <= SCO_Y0 + 3 && abs(x - SCO_CX) <= 3 - (y - SCO_Y0)) c = C_TRIG;
        }
    }

    /* ---- cursor: filled box in the colour of the mode being adjusted ---- */
    if (hl_on && x >= hl_x0 && x <= hl_x1 && y >= hl_y0 && y <= hl_y1)
        c = hl_col;

    /* ---- header: V/div | T/div | trigger | mode | HLD ----
     * The active item is drawn dark on top of its coloured cursor box. */
    if (y >= 1 && y <= 7)
    {
        if (text_hit(x, y, HDR_VDIV_X, 1, osd_vdiv))
            c = (adj_mode == MODE_VOLT) ? C_BLACK : C_WHITE;
        if (text_hit(x, y, HDR_TDIV_X, 1, osd_tdiv))
            c = (adj_mode == MODE_TIME) ? C_BLACK : (mode_fft ? C_CYAN : C_GREEN);
        if (text_hit(x, y, HDR_TRIG_X, 1, osd_trig))
            c = (adj_mode == MODE_TRIG) ? C_BLACK : C_TRIG;
        if (text_hit(x, y, HDR_MODE_X, 1, osd_mode)) c = C_WHITE;
        if (hold && text_hit(x, y, 143, 1, "HLD")) c = C_YELLOW;
    }

    /* ---- footer ---- */
    if (y >= FTR_Y0+2 && y <= FTR_Y0+8)
        if (text_hit(x, y, 2, FTR_Y0+2, osd_f1))  c = C_CYAN;
    if (y >= FTR_Y0+12 && y <= FTR_Y0+18)
    {
        if (text_hit(x, y, 2, FTR_Y0+12, osd_f2)) c = C_YELLOW;
        if (text_hit(x, y, FTR_POS_X, FTR_Y0+12, osd_pos))
            c = (adj_mode == MODE_POS) ? C_BLACK : C_ORANGE;
    }

    return c;
}

/* ============================================================
 * BUTTONS - non-blocking, only sets request flags.
 * Called from the main loop and after every rendered strip.
 * ============================================================ */
static void poll_buttons(void)
{
    static uint8_t  armed = 0;
    static uint8_t  sw_down = 0, long_fired = 0;
    static uint32_t sw_t0 = 0;
    static uint8_t  hold_prev = 1;
    static uint32_t hold_t = 0;

    uint32_t now = HAL_GetTick();

    /* Encoder switch PB8 (active LOW) */
    uint8_t sw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET);
    if (!armed)                      /* ignore a key that is stuck low at boot */
    {
        if (!sw) armed = 1;
        sw = 0;
    }

    if (sw && !sw_down)
    {
        sw_down = 1; long_fired = 0; sw_t0 = now;
    }
    else if (sw && sw_down && !long_fired && (now - sw_t0) >= 800u)
    {
        long_fired = 1; req_fft = 1;               /* long press */
    }
    else if (!sw && sw_down)
    {
        sw_down = 0;
        if (!long_fired && (now - sw_t0) >= 20u) req_edit = 1;   /* short press */
    }

    /* HOLD/RUN PB9 (active LOW) */
    uint8_t h = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    if (hold_prev == 1 && h == 0 && (now - hold_t) > 50u)
    {
        req_hold = 1; hold_t = now;
    }
    hold_prev = h;
}

/* ============================================================
 * RENDER - 10 strips x (compose + 1 bulk SPI)
 * ============================================================ */
static void render_scope(void)
{
    int trig_y = adc_to_y(trig_lvl);

    /* is the trigger level outside the visible window (V/div + POS)? */
    int32_t tmv = adc_to_mv(trig_lvl);
    int32_t ty  = SCO_CY - ((tmv - (int32_t)center_mv) * DIV_H) / (int32_t)vdiv_mv[vd_idx];
    trig_col = (ty < SCO_Y0 - 1 || ty > SCO_Y1 + 1) ? C_YELLOW : C_TRIG;

    int32_t gy0 = SCO_CY + ((int32_t)center_mv * DIV_H) / (int32_t)vdiv_mv[vd_idx];   /* row of 0 V */
    gnd_y   = mv_to_y(0);
    gnd_col = (gy0 < SCO_Y0 - 1 || gy0 > SCO_Y1 + 1) ? C_YELLOW : C_GREEN;

    for (int s = 0; s < LCD_W / STRIP_W; s++)
    {
        int x0 = s * STRIP_W;

        for (int y = 0; y < LCD_H; y++)
            for (int xi = 0; xi < STRIP_W; xi++)
                strip_buf[y * STRIP_W + xi] = compose(x0 + xi, y, trig_y);

        lcd_set_window((uint16_t)x0, 0,
                       (uint16_t)(x0 + STRIP_W - 1), LCD_H - 1);
        lcd_data_raw((uint8_t *)strip_buf, (uint16_t)(STRIP_PIXELS * 2));

        poll_buttons();
    }
}

/* ============================================================
 * INPUT HANDLING
 * ============================================================ */
static void handle_input(void)
{
    static uint16_t last_raw = 0;

    /* ---- encoder: wrap-safe delta, 4 edges per detent ---- */
    uint16_t raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t  diff = (int16_t)(uint16_t)(raw - last_raw);
    int16_t  d    = diff / 4;
    if (d != 0)
    {
        last_raw = (uint16_t)(last_raw + (uint16_t)(d * 4));
        d = (int16_t)(d * ENC_DIR);

        switch (adj_mode)
        {
        case MODE_TIME:                       /* clockwise = slower sweep */
            if (!hold)                        /* locked while HOLD */
            {
                int n = (int)tb_idx + d;
                if (n < 0) n = 0;
                if (n > TB_COUNT - 1) n = TB_COUNT - 1;
                if ((uint8_t)n != tb_idx)
                {
                    tb_idx = (uint8_t)n;
                    HAL_ADC_Stop_DMA(&hadc1);
                    __HAL_TIM_SET_AUTORELOAD(&htim3, tb_arr[tb_idx]);
                    __HAL_TIM_SET_COUNTER(&htim3, 0);
                    used_seq = dma_seq;       /* drop stale data */
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
                }
            }
            break;

        case MODE_VOLT:                       /* clockwise = more V/div (zoom out) */
        {
            int n = (int)vd_idx - d;
            if (n < 0) n = 0;
            if (n > VD_COUNT - 1) n = VD_COUNT - 1;
            vd_idx = (uint8_t)n;
            clamp_center();
            need_redraw = 1;
            break;
        }

        case MODE_TRIG:
        {
            /* step = 2 screen pixels at the current V/div (fine when zoomed in) */
            int step = (int)((float)vdiv_mv[vd_idx] * ((float)AFE_GAIN_X1000 / 1000.0f)
                             * (4095.0f / 3300.0f) / 8.0f);
            if (step < 1) step = 1;
            int n = (int)trig_lvl + d * step;
            if (n < 16)   n = 16;          /* ~0.01 V */
            if (n > 4079) n = 4079;        /* ~3.29 V */
            trig_lvl = (uint16_t)n;
            need_redraw = 1;
            break;
        }

        case MODE_POS:                        /* clockwise = trace moves up */
        {
            int step = (int)vdiv_mv[vd_idx] / 5;       /* 0.2 div per click */
            center_mv = (int16_t)((int)center_mv - d * step);
            clamp_center();
            need_redraw = 1;
            break;
        }

        default: break;
        }
    }

    /* ---- buttons ---- */
    poll_buttons();

    if (req_edit)                              /* short press: next mode */
    {
        req_edit = 0;
        if (!mode_fft) adj_mode = (uint8_t)((adj_mode + 1u) % MODE_COUNT);
        need_redraw = 1;
    }
    if (req_fft)                               /* long press: OSC <-> FFT */
    {
        req_fft = 0;
        mode_fft ^= 1;
        if (mode_fft) adj_mode = MODE_TIME;    /* only TIME makes sense in FFT */
        need_redraw = 1;
    }
    if (req_hold)
    {
        req_hold = 0;
        hold ^= 1;
        need_redraw = 1;
    }
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    fft_init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    lcd_init();
    lcd_clear(C_BLACK);
    HAL_ADCEx_Calibration_Start(&hadc1);

    __HAL_TIM_SET_AUTORELOAD(&htim3, tb_arr[tb_idx]);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
    HAL_TIM_Base_Start(&htim3);

    while (1)
    {
        handle_input();

        uint8_t fresh = (!hold) ? grab_snapshot() : 0;

        if (fresh || need_redraw)
        {
            need_redraw = 0;

            process_measurements();

            if (mode_fft) compute_fft();
            else          precompute_wave();

            build_osd();
            update_highlight();

#if DEBUG_PINS
            snprintf(osd_f2, sizeof(osd_f2), "SW%d HD%d E%u M%d F%d",
                     (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
                     (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9),
                     (unsigned)__HAL_TIM_GET_COUNTER(&htim4),
                     (int)adj_mode, (int)mode_fft);
#endif
            render_scope();
        }
    }
}

/* ============================================================
 * CLOCK & PERIPHERAL INIT
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       o = {0};
    RCC_ClkInitTypeDef       c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};

    o.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    o.HSEState            = RCC_HSE_ON;
    o.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    o.HSIState            = RCC_HSI_ON;
    o.PLL.PLLState        = RCC_PLL_ON;
    o.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL          = RCC_PLL_MUL9;   /* 8 x 9 = 72 MHz */
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType           = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                           |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider      = RCC_HCLK_DIV2;
    c.APB2CLKDivider      = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;  /* 12 MHz */
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    s.Channel      = ADC_CHANNEL_0;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 9 MHz (use _4 for 18 MHz) */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef m = {0};
    TIM_OC_InitTypeDef      oc= {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 71;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;          /* 1 kHz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_RESET;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &m) != HAL_OK) Error_Handler();

    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.Pulse       = 500;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState= TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  ck = {0};
    TIM_MasterConfigTypeDef m  = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 71;           /* 1 MHz */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 99;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    ck.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &ck) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_UPDATE;     /* triggers ADC */
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &m) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef e = {0};
    TIM_MasterConfigTypeDef m = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    e.EncoderMode  = TIM_ENCODERMODE_TI12;
    e.IC1Polarity  = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter    = 3;
    e.IC2Polarity  = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter    = 3;
    if (HAL_TIM_Encoder_Init(&htim4, &e) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_RESET;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &m) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* DC=PB0, RST=PB1, CS=PB10 - start HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin = GPIO_PIN_8; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);  /* Encoder SW */
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);  /* HOLD/RUN   */

    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);  /* PA0 ADC    */
}

/* ============================================================
 * MSP CALLBACKS
 * ============================================================ */
void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    if (h->Instance != ADC1) return;
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin=GPIO_PIN_0; g.Mode=GPIO_MODE_ANALOG; g.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(h, DMA_Handle, hdma_adc1);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI1) return;
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin=GPIO_PIN_5|GPIO_PIN_7; g.Mode=GPIO_MODE_AF_PP;
    g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance==TIM3) __HAL_RCC_TIM3_CLK_ENABLE();
}

/* HAL_TIM_PWM_Init() calls THIS (not Base_MspInit) */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance==TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM4) return;
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin=GPIO_PIN_6|GPIO_PIN_7; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM1) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin=GPIO_PIN_8; g.Mode=GPIO_MODE_AF_PP; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}

/* ============================================================
 * INTERRUPT HANDLER (see note at top of file)
 * ============================================================ */
#if DMA_HANDLER_IN_MAIN
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}
#endif

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif