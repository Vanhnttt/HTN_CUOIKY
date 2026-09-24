/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Digital Oscilloscope - V07 Standard (Optimized Hold & Smart UI Refresh)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
 
/* Private define ------------------------------------------------------------*/
#define LCD_WIDTH       128
#define LCD_HEIGHT      160
 
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_YELLOW    0xFFE0
#define COLOR_GREEN     0x07E0
#define COLOR_CYAN      0x07FF
#define COLOR_RED       0xF800
#define COLOR_GRAY      0x39E7
 
#define ADC_BUFFER_SIZE 240
#define WAVE_POINTS     120
#define TRIG_HYSTERESIS 30
 
/* Private macro -------------------------------------------------------------*/
#define TFT_CS_LOW()    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)
#define TFT_CS_HIGH()   HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)
#define TFT_DC_LOW()    HAL_GPIO_WritePin(GPIOB, TFT_DC_Pin, GPIO_PIN_RESET)
#define TFT_DC_HIGH()   HAL_GPIO_WritePin(GPIOB, TFT_DC_Pin, GPIO_PIN_SET)
#define TFT_RST_LOW()   HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin, GPIO_PIN_RESET)
#define TFT_RST_HIGH()  HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin, GPIO_PIN_SET)
 
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
 
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
 
uint16_t adc_sample_buffer[ADC_BUFFER_SIZE];
uint8_t prev_wave_y[WAVE_POINTS];
 
uint8_t is_hold = 0;
uint8_t time_scale_idx = 1;
uint8_t volt_scale_idx = 1;
uint16_t trig_level_adc = 2048;
 
float signal_vpp   = 0.0f;
float signal_vmax  = 0.0f;
float signal_vmin  = 0.0f;
float signal_vrms  = 0.0f;
float signal_freq  = 0.0f;
volatile uint8_t dma_complete = 0;

/* Các biến lưu giá trị cũ để kiểm soát chỉ load lại khi thay đổi */
static float old_vpp = -1.0f;
static float old_vmax = -1.0f;
static float old_vmin = -1.0f;
static float old_freq = -1.0f;
static uint8_t old_hold = 0xFF;
static uint8_t old_time_idx = 0xFF;
static uint8_t old_volt_idx = 0xFF;
static uint16_t old_trig = 0xFFFF;
 
const uint32_t arr_table[4] = { 49, 99, 199, 999 };
const char* time_label_table[4] = { "50us", "100us", "200us", "1ms" };
const float volt_scale_table[3] = { 0.5f, 1.0f, 2.0f };
const char* volt_label_table[3] = { "0.5V", "1.0V", "2.0V" };
 
const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x08,0x2A,0x1C,0x08},{0x7F,0x7F,0x7F,0x7F,0x7F}
};
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
 
void ST7735_WriteCommand(uint8_t cmd);
void ST7735_WriteData(uint8_t* buff, size_t buff_size);
void ST7735_Init(void);
void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color);
void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawChar(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg);
void ST7735_DrawString(uint8_t x, uint8_t y, const char* str, uint16_t color, uint16_t bg);
void Draw_ScopeGrid(void);
void Update_ScopeInfo(void);
void Process_Waveform(void);
 
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    dma_complete = 1;
}
 
/* USER CODE BEGIN 0 */
void ST7735_WriteCommand(uint8_t cmd) { TFT_DC_LOW(); TFT_CS_LOW(); HAL_SPI_Transmit(&hspi1, &cmd, 1, 100); TFT_CS_HIGH(); }
void ST7735_WriteData(uint8_t* buff, size_t buff_size) { TFT_DC_HIGH(); TFT_CS_LOW(); HAL_SPI_Transmit(&hspi1, buff, buff_size, 100); TFT_CS_HIGH(); }
void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    ST7735_WriteCommand(0x2A); uint8_t data[4] = { 0x00, x0, 0x00, x1 }; ST7735_WriteData(data, 4);
    ST7735_WriteCommand(0x2B); data[1] = y0; data[3] = y1; ST7735_WriteData(data, 4);
    ST7735_WriteCommand(0x2C);
}
void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    ST7735_SetAddressWindow(x, y, x, y);
    uint8_t data[2] = { color >> 8, color & 0xFF }; ST7735_WriteData(data, 2);
}
void ST7735_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if ((x + w - 1) >= LCD_WIDTH)  w = LCD_WIDTH - x;
    if ((y + h - 1) >= LCD_HEIGHT) h = LCD_HEIGHT - y;
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    uint8_t hi = color >> 8, lo = color & 0xFF; uint8_t line_buf[128 * 2];
    for (int i = 0; i < w * 2; i += 2) { line_buf[i] = hi; line_buf[i + 1] = lo; }
    TFT_DC_HIGH(); TFT_CS_LOW();
    for (int j = 0; j < h; j++) { HAL_SPI_Transmit(&hspi1, line_buf, w * 2, 100); }
    TFT_CS_HIGH();
}
void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy, e2;
    while (1) {
        if (x0 >= 0 && x0 < LCD_WIDTH && y0 >= 0 && y0 < LCD_HEIGHT) ST7735_DrawPixel((uint8_t)x0, (uint8_t)y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
void ST7735_DrawChar(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg) {
    if (ch < 32 || ch > 127) ch = ' ';
    uint8_t idx = ch - 32;
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[idx][i];
        for (uint8_t j = 0; j < 7; j++) {
            if (line & (1 << j)) ST7735_DrawPixel(x + i, y + j, color);
            else if (bg != color) ST7735_DrawPixel(x + i, y + j, bg);
        }
    }
}
void ST7735_DrawString(uint8_t x, uint8_t y, const char* str, uint16_t color, uint16_t bg) {
    while (*str) { if (x + 6 > LCD_WIDTH) break; ST7735_DrawChar(x, y, *str++, color, bg); x += 6; }
}
void ST7735_Init(void) {
    TFT_RST_LOW(); HAL_Delay(10); TFT_RST_HIGH(); HAL_Delay(10);
    ST7735_WriteCommand(0x01); HAL_Delay(50);
    ST7735_WriteCommand(0x11); HAL_Delay(50);
    ST7735_WriteCommand(0x3A); uint8_t cmod = 0x05; ST7735_WriteData(&cmod, 1);
    ST7735_WriteCommand(0x36); uint8_t madctl = 0x08; ST7735_WriteData(&madctl, 1);
    ST7735_WriteCommand(0x29); HAL_Delay(20);
    ST7735_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, COLOR_BLACK);
}
void Draw_ScopeGrid(void) {
    for (int x = 4; x <= 124; x++) { ST7735_DrawPixel(x, 22, COLOR_WHITE); ST7735_DrawPixel(x, 142, COLOR_WHITE); }
    for (int y = 22; y <= 142; y++) { ST7735_DrawPixel(4, y, COLOR_WHITE); ST7735_DrawPixel(124, y, COLOR_WHITE); }
    for (int x = 4; x <= 124; x += 20) { for (int y = 22; y <= 142; y += 4) { ST7735_DrawPixel(x, y, COLOR_GRAY); } }
    for (int y = 22; y <= 142; y += 20) { for (int x = 4; x <= 124; x += 4) { ST7735_DrawPixel(x, y, COLOR_GRAY); } }
}
 
/* THUẬT TOÁN TỐI ƯU: Chỉ load/vẽ lại màn hình khi có sự thay đổi giá trị thực tế */
void Update_ScopeInfo(void) {
    char str[30];

    // 1. Kiểm tra và cập nhật dòng Trạng thái + Trigger
    if (is_hold != old_hold || trig_level_adc != old_trig) {
        ST7735_FillRect(2, 4, 124, 8, COLOR_BLACK); 
        if (is_hold) ST7735_DrawString(2, 4, "HOLD", COLOR_RED, COLOR_BLACK);
        else ST7735_DrawString(2, 4, "RUN ", COLOR_GREEN, COLOR_BLACK);
        
        float v_trig = (trig_level_adc * 3.3f) / 4095.0f;
        sprintf(str, "Trg:%.2fV", v_trig); 
        ST7735_DrawString(56, 4, str, COLOR_YELLOW, COLOR_BLACK);

        old_hold = is_hold;
        old_trig = trig_level_adc;
    }

    // 2. Kiểm tra và cập nhật Thang đo Time/Volt khi thay đổi
    if (time_scale_idx != old_time_idx || volt_scale_idx != old_volt_idx) {
        ST7735_FillRect(2, 13, 124, 8, COLOR_BLACK);
        sprintf(str, "T:%s V:%s", time_label_table[time_scale_idx], volt_label_table[volt_scale_idx]);
        ST7735_DrawString(2, 13, str, COLOR_WHITE, COLOR_BLACK);

        old_time_idx = time_scale_idx;
        old_volt_idx = volt_scale_idx;
    }

    // 3. Kiểm tra và cập nhật Max, Min khi giá trị thay đổi
    if (signal_vmax != old_vmax || signal_vmin != old_vmin) {
        ST7735_FillRect(2, 144, 124, 8, COLOR_BLACK); 
        sprintf(str, "Max:%.1fV", signal_vmax); 
        ST7735_DrawString(2, 144, str, COLOR_CYAN, COLOR_BLACK);
        
        sprintf(str, "Min:%.1fV", signal_vmin); 
        ST7735_DrawString(68, 144, str, COLOR_CYAN, COLOR_BLACK);

        old_vmax = signal_vmax;
        old_vmin = signal_vmin;
    }

    // 4. Kiểm tra và cập nhật Vpp, Tần số khi giá trị thay đổi
    if (signal_vpp != old_vpp || signal_freq != old_freq) {
        ST7735_FillRect(2, 152, 124, 8, COLOR_BLACK); 
        sprintf(str, "Vpp:%.2fV", signal_vpp); 
        ST7735_DrawString(2, 152, str, COLOR_WHITE, COLOR_BLACK);
        
        if (signal_freq >= 1000.0f) {
            sprintf(str, "f:%.1fkHz", signal_freq / 1000.0f);
        } else {
            sprintf(str, "f:%.0fHz", signal_freq);
        }
        ST7735_DrawString(68, 152, str, COLOR_WHITE, COLOR_BLACK);

        old_vpp = signal_vpp;
        old_freq = signal_freq;
    }
}
 
void Process_Waveform(void)
{
    // Nếu đang bật chế độ HOLD: chỉ gọi cập nhật thông tin giao diện (cho phép đổi scale/trig khi hold) rồi thoát ngay
    if (is_hold) {
        Update_ScopeInfo();
        return;
    }
 
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr_table[time_scale_idx]);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
 
    dma_complete = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_sample_buffer, ADC_BUFFER_SIZE);
 
    uint32_t start_tick = HAL_GetTick();
    while (!dma_complete) {
        if (HAL_GetTick() - start_tick > 200) break;
    }
    HAL_ADC_Stop_DMA(&hadc1);
 
    uint16_t adc_max = 0, adc_min = 4095;
    float sum_sq = 0.0f;
    for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
        uint16_t s = adc_sample_buffer[i];
        if (s > adc_max) adc_max = s;
        if (s < adc_min) adc_min = s;
        float v = (s * 3.3f) / 4095.0f;
        sum_sq += v * v;
    }
    uint16_t threshold = (adc_max + adc_min) / 2;
 
    int trigger_idx = 0;
    for (int i = 1; i < WAVE_POINTS; i++) {
        if (adc_sample_buffer[i - 1] < (int32_t)(trig_level_adc - TRIG_HYSTERESIS) &&
            adc_sample_buffer[i]     >= (int32_t)(trig_level_adc + TRIG_HYSTERESIS)) {
            trigger_idx = i;
            break;
        }
    }
 
    int first_edge_idx = -1, second_edge_idx = -1;
    for (int i = 1; i < ADC_BUFFER_SIZE; i++) {
        if (adc_sample_buffer[i - 1] < threshold && adc_sample_buffer[i] >= threshold) {
            if (first_edge_idx == -1) {
                first_edge_idx = i;
            } else {
                second_edge_idx = i;
                break;
            }
        }
    }
 
    if (adc_max > adc_min && (adc_max - adc_min) > 80) {
        signal_vpp  = ((adc_max - adc_min) * 3.3f) / 4095.0f;
        signal_vmax = (adc_max * 3.3f) / 4095.0f;
        signal_vmin = (adc_min * 3.3f) / 4095.0f;
        signal_vrms = sqrtf(sum_sq / ADC_BUFFER_SIZE);
 
        if (first_edge_idx != -1 && second_edge_idx != -1) {
            int32_t period_ticks = second_edge_idx - first_edge_idx;
            if (period_ticks > 0) {
                float period_us = period_ticks * (float)(arr_table[time_scale_idx] + 1);
                signal_freq = 1000000.0f / period_us;
            }
        }
    } else {
        signal_vpp = 0.0f; signal_vmax = 0.0f; signal_vmin = 0.0f;
        signal_vrms = 0.0f; signal_freq = 0.0f;
    }
 
    for (int i = 0; i < WAVE_POINTS - 1; i++) {
        ST7735_DrawLine(4 + i, prev_wave_y[i], 4 + i + 1, prev_wave_y[i + 1], COLOR_BLACK);
    }
    Draw_ScopeGrid();
 
    float v_scale = volt_scale_table[volt_scale_idx];
    for (int i = 0; i < WAVE_POINTS; i++) {
        uint16_t sample = adc_sample_buffer[trigger_idx + i]; 
        int32_t offset_val = (int32_t)sample - 2048;
        int16_t y = 82 - (int16_t)((offset_val * 42.0f * v_scale) / 2048.0f);
        if (y < 23) y = 23; if (y > 141) y = 141;
        prev_wave_y[i] = (uint8_t)y;
    }
 
    for (int i = 0; i < WAVE_POINTS - 1; i++) {
        ST7735_DrawLine(4 + i, prev_wave_y[i], 4 + i + 1, prev_wave_y[i + 1], COLOR_CYAN);
    }
    Update_ScopeInfo();
}
/* USER CODE END 0 */
 
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim3);
 
  ADC1->CR2 |= ADC_CR2_ADON; 
  HAL_Delay(5);
  HAL_ADCEx_Calibration_Start(&hadc1);
 
  ST7735_Init();
  Draw_ScopeGrid();
  Update_ScopeInfo();
 
  while (1)
  {
    Process_Waveform();
  }
}
 
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
 
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
 
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}
 
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
 
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) { Error_Handler(); }
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
 
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; 
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
 
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}
 
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}
 
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
 
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
 
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
}
 
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
 
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71; 
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99; 
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
 
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
 
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; 
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}
 
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
 
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
 
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TFT_DC_Pin|TFT_RST_Pin, GPIO_PIN_RESET);
 
  GPIO_InitStruct.Pin = TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = TFT_DC_Pin|TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_4 | BTN_VOLT_Pin | BTN_TRIG_Pin | BTN_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}
 
void Error_Handler(void) { __disable_irq(); while (1) { } }

void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(BTN_VOLT_Pin);
    HAL_GPIO_EXTI_IRQHandler(BTN_TRIG_Pin);
    HAL_GPIO_EXTI_IRQHandler(BTN_HOLD_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_irq_tick = 0;
    if (HAL_GetTick() - last_irq_tick < 150) return; 

    if (GPIO_Pin == GPIO_PIN_4) {
        time_scale_idx = (time_scale_idx + 1) % 4;
    }
    else if (GPIO_Pin == BTN_VOLT_Pin) {
        volt_scale_idx = (volt_scale_idx + 1) % 3;
    }
    else if (GPIO_Pin == BTN_TRIG_Pin) {
        trig_level_adc += 200; 
        if (trig_level_adc > 3000) trig_level_adc = 1000;
    }
    else if (GPIO_Pin == BTN_HOLD_Pin) {
        is_hold = !is_hold;
    }
    
    last_irq_tick = HAL_GetTick();
}