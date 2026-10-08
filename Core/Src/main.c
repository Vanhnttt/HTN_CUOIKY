/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : May hien song mini V07 - STM32F103C8T6 + ST7735 128x160
 ******************************************************************************
 * Pinout:
 *   ADC1_IN0 = PA0          SPI1_SCK = PA5       SPI1_MOSI = PA7
 *   DL_A0 (DC) = PB0        DL_RS (RESET) = PB1  DL_CS = PB10
 *   TIM1_CH1 (PWM test) = PA8
 *   Encoder CLK = PB5, DT = PB4, SW = PB8, BUT_HOLD_RUN = PB9
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "st7735.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE       1       /* 1 = chi test man hinh, chay on thi dat 0 */
#define DISPLAY_SELFTEST 1       /* 1 = chay thu doi mau luc khoi dong, chay on thi dat 0 */

#define AFE_RATIO        20.0f   /* he so suy hao mach vao: Vin = (Vadc - 1.667V) * 20 */
#define ADC_ZERO_CODE    2068    /* ma ADC tuong ung 0V (1.667V) */
#define DEF_TIME_IDX     3       /* mac dinh 400us/div */
#define DEF_VOLT_IDX     3       /* mac dinh 1V/div */

#define HALF_SIZE        256     /* nua bo dem DMA */
#define ADC_BUF_SIZE     (HALF_SIZE * 2)

/* Khung do thi (6 x 5 o) */
#define GRID_X0          4
#define GRID_Y0          22
#define GRID_W           120
#define GRID_H           100
#define GRID_DIV_X       20
#define GRID_DIV_Y       20
#define V_TOTAL_DIV      5.0f
#define Y_CENTER         (GRID_Y0 + GRID_H / 2)
#define WAVE_POINTS      (GRID_W + 1)

static const uint16_t ts_us_table[] = { 2, 5, 10, 20, 50, 100, 200, 500, 1000 };
static const char *const time_label[] = { "40us", "100us", "200us", "400us", "1ms", "2ms", "4ms", "10ms", "20ms" };
static const float vdiv_table[] = { 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f };
static const char *const vdiv_label[] = { "100mV", "200mV", "500mV", "1V", "2V", "5V", "10V" };
#define TIME_COUNT   ((int)(sizeof(ts_us_table) / sizeof(ts_us_table[0])))
#define VOLT_COUNT   ((int)(sizeof(vdiv_table) / sizeof(vdiv_table[0])))

#define V_HALF_DIV       (V_TOTAL_DIV / 2.0f)
#define VREF_V           3.3f
#define CAL_GAIN         1.000f
#define VOLT_PER_CODE    ((VREF_V / 4095.0f) * AFE_RATIO * CAL_GAIN)
#define SIGNAL_MIN_PP    16      /* nguong chong nhieu */
#define HYST_MIN         6       /* do tre trigger */
#define MIN_SMP_PERIOD   4.0f
#define AUTO_P_MIN       24.0f
#define AUTO_P_MAX       64.0f
#define AUTO_V_HI        (0.90f * V_TOTAL_DIV)
#define AUTO_V_LO        (0.28f * V_TOTAL_DIV)
#define AUTO_V_NEW       (0.84f * V_TOTAL_DIV)

static uint16_t cap_buf[HALF_SIZE];
static uint8_t  cap_ts_idx    = DEF_TIME_IDX;
static int      g_time_idx    = DEF_TIME_IDX;
static int      g_volt_idx    = DEF_VOLT_IDX;
static uint8_t  g_time_auto   = 1;
static uint8_t  g_volt_auto   = 1;
static uint8_t  g_trig_auto   = 1;
static float    g_trig_v      = 0.0f;
static int      g_trig_code   = ADC_ZERO_CODE;
static int      g_center_code = ADC_ZERO_CODE;
static float    g_px_per_code = 1.0f;
static uint8_t  g_scan_cnt    = 0;
static int      g_last_mid    = 0;

typedef struct {
    int     trig_idx, mid_code;
    uint8_t triggered, valid, over, too_fast;
    float   vpp, vmax, vmin, freq, p_smp;
} Meas_t;
static Meas_t meas;

/* Mau RGB565 */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_GRAY    0x4A69
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF

/* Encoder / nut nhan */
#define ENC_DIR              (+1)   /* doi thanh -1 neu xoay nguoc chieu */
#define ENC_COUNTS_PER_STEP  4      /* doi thanh 2 neu 1 nac nhay 2 buoc */
#define DEBOUNCE_MS          25
#define LONG_PRESS_MS        800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static uint16_t adc_buf[ADC_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint8_t cap_request = 0;
static volatile uint8_t cap_ready   = 0;
static volatile uint8_t cap_discard = 0;
static volatile uint8_t time_active = DEF_TIME_IDX;

static uint8_t g_hold = 0, g_redraw = 0, g_have_data = 0;
typedef enum { SEL_TIME = 0, SEL_VOLT, SEL_TRIG, SEL_COUNT } Sel_t;
static uint8_t g_sel = SEL_TIME;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ============================================================================
 *  1. XU LY SO LIEU & TRIGGER
 * ========================================================================== */
static void Update_Scales(void) {
    const float vpc = VOLT_PER_CODE;
    const float vd  = vdiv_table[g_volt_idx];
    g_px_per_code = ((float)GRID_DIV_Y / vd) * vpc;
    if (!g_trig_auto) {
        float vc  = (float)(g_center_code - ADC_ZERO_CODE) * vpc;
        float lim = (V_HALF_DIV - 0.1f) * vd;
        if (g_trig_v > vc + lim) g_trig_v = vc + lim;
        if (g_trig_v < vc - lim) g_trig_v = vc - lim;
        g_trig_code = ADC_ZERO_CODE + (int)(g_trig_v / vpc + (g_trig_v >= 0.0f ? 0.5f : -0.5f));
    }
}

static int Y_raw(int code) {
    float d = (float)(code - g_center_code) * g_px_per_code;
    int off = (int)(d + (d >= 0.0f ? 0.5f : -0.5f));
    return Y_CENTER - off;
}

static int Y_of_code(int code) {
    int y = Y_raw(code);
    if (y < GRID_Y0 + 1)           y = GRID_Y0 + 1;
    if (y > GRID_Y0 + GRID_H - 1)  y = GRID_Y0 + GRID_H - 1;
    return y;
}

static void Analyze(void) {
    uint16_t mn = 0xFFFF, mx = 0;
    for (int i = 0; i < HALF_SIZE; i++) {
        uint16_t s = cap_buf[i];
        if (s < mn) mn = s;
        if (s > mx) mx = s;
    }
    const float vpc = VOLT_PER_CODE;
    int pp = (int)mx - (int)mn;
    meas.valid    = (pp >= SIGNAL_MIN_PP);
    meas.over     = (mx >= 4085 || mn <= 10);
    meas.mid_code = ((int)mx + (int)mn) / 2;
    meas.vmax = ((int)mx - ADC_ZERO_CODE) * vpc;
    meas.vmin = ((int)mn - ADC_ZERO_CODE) * vpc;
    meas.vpp  = meas.valid ? (float)pp * vpc : 0.0f;
    meas.freq = 0.0f;
    meas.p_smp = 0.0f;
    meas.too_fast = 0;

    if (meas.valid) {
        int dd = meas.mid_code - g_center_code;
        if (dd < 0) dd = -dd;
        if (dd > 3) g_center_code = meas.mid_code;
    } else {
        g_center_code = ADC_ZERO_CODE;
    }

    int hy = pp / 8;
    if (hy < HYST_MIN) hy = HYST_MIN;
    if (hy > 40)       hy = 40;

    if (g_trig_auto && meas.valid) {
        g_trig_code = meas.mid_code;
        g_trig_v    = (float)(g_trig_code - ADC_ZERO_CODE) * vpc;
    }

    /* Trigger canh len co hysteresis */
    meas.trig_idx  = 0;
    meas.triggered = 0;
    uint8_t armed = 0;
    for (int i = 0; i <= HALF_SIZE - WAVE_POINTS; i++) {
        int s = cap_buf[i];
        if (s < g_trig_code - hy) {
            armed = 1;
        } else if (armed && s >= g_trig_code) {
            meas.trig_idx  = i;
            meas.triggered = 1;
            break;
        }
    }

    /* Do tan so: noi suy tuyen tinh giua cac canh len */
    if (meas.valid) {
        int thr = meas.mid_code;
        int hf  = pp / 8;
        if (hf < HYST_MIN) hf = HYST_MIN;
        uint8_t arm = (cap_buf[0] < thr - hf);
        int   edges = 0;
        float t_first = 0.0f, t_last = 0.0f;
        for (int i = 1; i < HALF_SIZE; i++) {
            int s = cap_buf[i];
            if (s < thr - hf) {
                arm = 1;
            } else if (arm && s >= thr) {
                float prev = (float)cap_buf[i - 1];
                float den  = (float)s - prev;
                float frac = (den > 0.0f) ? ((float)thr - prev) / den : 0.0f;
                float t    = (float)(i - 1) + frac;
                if (edges == 0) t_first = t;
                t_last = t;
                edges++;
                arm = 0;
            }
        }
        if (edges >= 2) {
            meas.p_smp = (t_last - t_first) / (float)(edges - 1);
            if (meas.p_smp >= MIN_SMP_PERIOD)
                meas.freq = 1.0e6f / (meas.p_smp * (float)ts_us_table[cap_ts_idx]);
            else
                meas.too_fast = 1;
        }
    }
}

static uint8_t AutoRange_Step(void) {
    uint8_t chg = 0;
    if (!meas.valid) {
        int dm = meas.mid_code - g_last_mid;
        if (dm < 0) dm = -dm;
        if (dm > 20) g_scan_cnt = 0;
        g_last_mid = meas.mid_code;
        if (g_time_auto) {
            if (g_scan_cnt < TIME_COUNT) {
                g_scan_cnt++;
                g_time_idx = (g_time_idx + 1) % TIME_COUNT;
                chg |= 1;
            } else if (g_time_idx != DEF_TIME_IDX) {
                g_time_idx = DEF_TIME_IDX;
                chg |= 1;
            }
        }
        return chg;
    }
    g_scan_cnt = 0;
    g_last_mid = meas.mid_code;
    if (g_time_auto) {
        if (meas.freq > 0.0f) {
            if (meas.p_smp > AUTO_P_MAX && g_time_idx < TIME_COUNT - 1) { g_time_idx++; chg |= 1; }
            else if (meas.p_smp < AUTO_P_MIN && g_time_idx > 0)         { g_time_idx--; chg |= 1; }
        } else if (meas.too_fast) {
            if (g_time_idx > 0) { g_time_idx--; chg |= 1; }
        } else if (g_time_idx < TIME_COUNT - 1) {
            g_time_idx++; chg |= 1;
        }
    }
    if (g_volt_auto) {
        float d = meas.vpp / vdiv_table[g_volt_idx];
        if (d > AUTO_V_HI && g_volt_idx < VOLT_COUNT - 1) {
            g_volt_idx++; chg |= 2;
        } else if (d < AUTO_V_LO && g_volt_idx > 0 && meas.vpp / vdiv_table[g_volt_idx - 1] <= AUTO_V_NEW) {
            g_volt_idx--; chg |= 2;
        }
    }
    return chg;
}

static void fmt_val(char *dst, const char *pre, float v, int plus, const char *suf) {
    float   a   = (v < 0.0f) ? -v : v;
    int     dec = (a < 9.995f) ? 2 : 1;
    int32_t sc  = (dec == 2) ? 100 : 10;
    int32_t iv  = (int32_t)(a * (float)sc + 0.5f);
    char    sg  = (v < 0.0f && iv != 0) ? '-' : (plus ? '+' : 0);
    int n = 0;
    n += sprintf(dst + n, "%s", pre);
    if (sg) dst[n++] = sg;
    n += sprintf(dst + n, "%ld.", (long)(iv / sc));
    if (dec == 2) n += sprintf(dst + n, "%02ld", (long)(iv % sc));
    else          n += sprintf(dst + n, "%ld",   (long)(iv % sc));
    sprintf(dst + n, "%s", suf);
}

static void fmt_freq(char *dst, float f, const char *pre) {
    if (f <= 0.0f) sprintf(dst, "%s---", pre);
    else if (f >= 9995.0f) { long k = (long)(f / 100.0f + 0.5f); sprintf(dst, "%s%ld.%ldkHz", pre, k / 10, k % 10); }
    else if (f >= 1000.0f) { long k = (long)(f / 10.0f + 0.5f);  sprintf(dst, "%s%ld.%02ldkHz", pre, k / 100, k % 100); }
    else if (f >= 100.0f)  { sprintf(dst, "%s%ldHz", pre, (long)(f + 0.5f)); }
    else { long t = (long)(f * 10.0f + 0.5f); sprintf(dst, "%s%ld.%ldHz", pre, t / 10, t % 10); }
}

/* ============================================================================
 *  2. NGOAI VI: ADC + TIMER + DMA, PWM TEST
 * ========================================================================== */
static void Capture_Copy(const uint16_t *src) {
    if (cap_discard) { cap_discard--; return; }
    if (cap_request) {
        memcpy(cap_buf, src, sizeof(cap_buf));
        cap_ts_idx  = time_active;
        cap_request = 0;
        cap_ready   = 1;
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) Capture_Copy(&adc_buf[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) Capture_Copy(&adc_buf[HALF_SIZE]);
}

static void Apply_Timebase(void) {
    time_active = (uint8_t)g_time_idx;
    __HAL_TIM_SET_AUTORELOAD(&htim2, ts_us_table[g_time_idx] - 1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    cap_ready = 0;
    cap_request = 0;
    cap_discard = 1;   /* bo nua buffer dau sau khi doi time/div */
}

/* ADC1 duoc kich boi TIM2 CC2 (EXTSEL = 011 tren STM32F103) */
static void ADC_ForceTimerTrigger(void) {
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
    MODIFY_REG(hadc1.Instance->CR2, ADC_CR2_EXTSEL, ADC_EXTERNALTRIGCONV_T2_CC2);
}

static void ADC_TriggerTimer_Start(void) {
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

/* Ep DMA ADC1 chay circular, half-word (neu CubeMX chua dat) */
static void DMA_ForceCircular(void) {
    if (hdma_adc1.Instance == NULL) {
        hdma_adc1.Instance = DMA1_Channel1;
        hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
        __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
    }
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    HAL_DMA_Init(&hdma_adc1);
}

/* GPIO encoder (PB4 = DT, PB5 = CLK). Giai phong PB3/PB4 khoi JTAG, giu SWD. */
static void Encoder_GPIO_Init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin  = CLK_Pin | DT_Pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
}

/* Ep chan SPI1 ve PA5 (SCK) / PA7 (MOSI), khong remap */
static void SPI1_Pins_Force(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_AFIO_REMAP_SPI1_DISABLE();
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}

/* PWM test 10Hz..10kHz tren PA8 (TIM1_CH1), duty 50% */
typedef struct { uint16_t psc; uint16_t arr; } PwmCfg_t;
static const PwmCfg_t pwm_table[] = { { 719, 9999 }, { 71, 9999 }, { 71, 999 }, { 71, 199 }, { 71, 99 } };
static const char *const pwm_label[] = { "PWM:10Hz", "PWM:100Hz", "PWM:1kHz", "PWM:5kHz", "PWM:10kHz" };
#define PWM_COUNT   ((int)(sizeof(pwm_table) / sizeof(pwm_table[0])))
static int g_pwm_idx = 2;

static void PWM_Apply(void) {
    const PwmCfg_t *p = &pwm_table[g_pwm_idx];
    __HAL_TIM_SET_PRESCALER(&htim1, p->psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, p->arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (p->arr + 1) / 2);
    htim1.Instance->EGR = TIM_EGR_UG;
}

/* ============================================================================
 *  3. GIAO DIEN MAN HINH (128 x 160)
 * ========================================================================== */
static void Scope_Render(void) {
    uint8_t  wy[WAVE_POINTS];
    uint16_t col[GRID_H + 1];

    for (int i = 0; i < WAVE_POINTS; i++)
        wy[i] = (uint8_t)Y_of_code(cap_buf[meas.trig_idx + i]);

    int rt = Y_of_code(g_trig_code) - GRID_Y0;   /* hang cua vach trigger */
    int rg = Y_raw(ADC_ZERO_CODE) - GRID_Y0;     /* hang cua moc 0V */

    for (int c = 0; c < WAVE_POINTS; c++) {
        for (int r = 0; r <= GRID_H; r++) {
            uint16_t px = C_BLACK;
            if (c == 0 || c == GRID_W || r == 0 || r == GRID_H)         px = C_WHITE;
            else if ((c % GRID_DIV_X) == 0 && (r % 4) == 0)             px = C_GRAY;
            else if ((r % GRID_DIV_Y) == 0 && (c % 4) == 0)             px = C_GRAY;
            col[r] = px;
        }
        if (rt > 0 && rt < GRID_H && c > 0 && c < GRID_W && (c % 3) == 0)
            col[rt] = C_YELLOW;

        if (rg > 1 && rg < GRID_H - 1) {
            if (c == 1)      { col[rg - 1] = C_GREEN; col[rg] = C_GREEN; col[rg + 1] = C_GREEN; }
            else if (c == 2) { col[rg] = C_GREEN; }
        }

        int y1 = wy[c];
        int y0 = (c > 0) ? wy[c - 1] : wy[c];
        int lo = (y0 < y1) ? y0 : y1;
        int hi = (y0 < y1) ? y1 : y0;
        for (int y = lo; y <= hi; y++) col[y - GRID_Y0] = C_CYAN;

        for (int r = 0; r <= GRID_H; r++)        /* doi byte cho ST7735 (big-endian) */
            col[r] = (uint16_t)((col[r] << 8) | (col[r] >> 8));

        ST7735_DrawImage(GRID_X0 + c, GRID_Y0, 1, GRID_H + 1, (const uint16_t *)col);
    }
}

typedef struct { char txt[16]; uint16_t col; } Field_t;
static Field_t fd_status, fd_trig, fd_time, fd_volt, fd_max, fd_min, fd_vpp, fd_freq, fd_pwm;

/* Chi ve lai khi noi dung / mau thay doi -> man hinh khong nhap nhay */
static void Field_Draw(Field_t *f, uint16_t x, uint16_t y, int width, const char *s, uint16_t col) {
    char buf[16];
    int n = (int)strlen(s);
    if (width > 15) width = 15;
    if (n > width) n = width;
    memcpy(buf, s, (size_t)n);
    for (int i = n; i < width; i++) buf[i] = ' ';
    buf[width] = 0;
    if (f->col == col && strcmp(f->txt, buf) == 0) return;
    ST7735_WriteString(x, y, buf, Font_7x10, col, C_BLACK);
    strcpy(f->txt, buf);
    f->col = col;
}

/* Font 7x10 -> man 128px chi du 18 ky tu / dong, chia 2 cot moi cot 9 ky tu */
static void Info_Update(void) {
    char s[24];

    const char *st = g_hold ? "HOLD" : (meas.over ? "OVR!" : (meas.triggered ? "TRIG" : "AUTO"));
    uint16_t   stc = (g_hold || meas.over) ? C_RED : (meas.triggered ? C_GREEN : C_YELLOW);
    Field_Draw(&fd_status, 0, 0, 4, st, stc);

    fmt_val(s, g_trig_auto ? "Tr*" : "Tr:", g_trig_v, 1, "V");
    Field_Draw(&fd_trig, 34, 0, 10, s, (g_sel == SEL_TRIG) ? C_YELLOW : C_WHITE);

    /* "T*400us" / "T:400us"  ( * = auto, : = manual ) */
    sprintf(s, "%s%s", g_time_auto ? "T*" : "T:", time_label[g_time_idx]);
    Field_Draw(&fd_time, 0, 11, 8, s, (g_sel == SEL_TIME) ? C_YELLOW : C_WHITE);

    sprintf(s, "%s%s", g_volt_auto ? "V*" : "V:", vdiv_label[g_volt_idx]);
    Field_Draw(&fd_volt, 64, 11, 8, s, (g_sel == SEL_VOLT) ? C_YELLOW : C_WHITE);

    fmt_val(s, "Mx", meas.vmax, 1, "V");
    Field_Draw(&fd_max, 0, 126, 9, s, C_CYAN);

    fmt_val(s, "Mn", meas.vmin, 1, "V");
    Field_Draw(&fd_min, 63, 126, 9, s, C_CYAN);

    fmt_val(s, "Vpp:", meas.vpp, 0, "V");
    Field_Draw(&fd_vpp, 0, 138, 9, s, C_WHITE);

    fmt_freq(s, meas.freq, "f:");
    Field_Draw(&fd_freq, 63, 138, 9, s, C_WHITE);

    Field_Draw(&fd_pwm, 0, 150, 9, pwm_label[g_pwm_idx], C_CYAN);
}

/* ============================================================================
 *  4. NUT NHAN + ENCODER (quet trong ngat SysTick 1ms)
 * ========================================================================== */
#define EV_NONE   0
#define EV_SHORT  1
#define EV_LONG   2
typedef struct { uint8_t cnt, state, long_done; uint16_t held; } Deb_t;

static Deb_t deb_sw, deb_hold;
static volatile uint8_t ev_sw = 0, ev_hold = 0;
static volatile int16_t enc_steps = 0;
static volatile uint8_t inputs_run = 0;

static uint8_t Deb_Update(Deb_t *d, uint8_t pressed) {
    uint8_t ev = EV_NONE;
    if (pressed == d->state) {
        d->cnt = 0;
    } else if (++d->cnt >= DEBOUNCE_MS) {
        d->cnt = 0;
        d->state = pressed;
        if (pressed) { d->held = 0; d->long_done = 0; }
        else if (!d->long_done) ev = EV_SHORT;
    }
    if (d->state && !d->long_done) {
        if (++d->held >= LONG_PRESS_MS) { d->long_done = 1; ev = EV_LONG; }
    }
    return ev;
}

static uint8_t quad_state = 0;
static int8_t  quad_acc = 0;
static const int8_t quad_tab[16] = { 0,-1,1,0,  1,0,0,-1,  -1,0,0,1,  0,1,-1,0 };

static void Encoder_Tick(void) {
    uint8_t a = (HAL_GPIO_ReadPin(CLK_GPIO_Port, CLK_Pin) == GPIO_PIN_SET);
    uint8_t b = (HAL_GPIO_ReadPin(DT_GPIO_Port,  DT_Pin)  == GPIO_PIN_SET);
    quad_state = (uint8_t)(((quad_state << 2) | (a << 1) | b) & 0x0F);
    quad_acc += quad_tab[quad_state];
    if (quad_acc >= ENC_COUNTS_PER_STEP) {
        quad_acc -= ENC_COUNTS_PER_STEP;  enc_steps += ENC_DIR;
    } else if (quad_acc <= -ENC_COUNTS_PER_STEP) {
        quad_acc += ENC_COUNTS_PER_STEP;  enc_steps -= ENC_DIR;
    }
}

static void Inputs_Tick(void) {
    Encoder_Tick();
    uint8_t e;
    e = Deb_Update(&deb_sw,   HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET);
    if (e) ev_sw = e;
    e = Deb_Update(&deb_hold, HAL_GPIO_ReadPin(BUT_HOLD_RUN_GPIO_Port, BUT_HOLD_RUN_Pin) == GPIO_PIN_RESET);
    if (e) ev_hold = e;
}

/* Ghi de ham weak cua HAL */
void HAL_IncTick(void) {
    uwTick += uwTickFreq;
    if (inputs_run) Inputs_Tick();
}

static int Encoder_TakeSteps(void) {
    __disable_irq();
    int v = enc_steps;
    enc_steps = 0;
    __enable_irq();
    return v;
}

static void AutoSet(void) {
    g_hold = 0;
    g_time_auto = g_volt_auto = g_trig_auto = 1;
    g_time_idx = 0;
    g_scan_cnt = 0;
    Apply_Timebase();
    Update_Scales();
}

/*  Nut HOLD/RUN : nhan ngan = Hold/Run, nhan giu = doi tan so PWM test
 *  Nut SW       : nhan ngan = chon Time -> Volt -> Trig, nhan giu = AutoSet
 *  Encoder      : chinh thong so dang chon */
static void UI_Process(void) {
    uint8_t e;
    e = ev_hold; ev_hold = 0;
    if (e == EV_SHORT) {
        g_hold ^= 1;
        if (g_hold) cap_request = 0;
    } else if (e == EV_LONG) {
        g_pwm_idx = (g_pwm_idx + 1) % PWM_COUNT;
        PWM_Apply();
    }

    e = ev_sw; ev_sw = 0;
    if (e == EV_SHORT) {
        g_sel = (uint8_t)((g_sel + 1) % SEL_COUNT);
    } else if (e == EV_LONG) {
        AutoSet();
    }

    int d = Encoder_TakeSteps();
    if (d != 0) {
        switch (g_sel) {
        case SEL_TIME:
            if (!g_hold) {
                g_time_auto = 0;
                g_time_idx += d;
                if (g_time_idx < 0) g_time_idx = 0;
                if (g_time_idx > TIME_COUNT - 1) g_time_idx = TIME_COUNT - 1;
                Apply_Timebase();
            }
            break;
        case SEL_VOLT:
            g_volt_auto = 0;
            g_volt_idx += d;
            if (g_volt_idx < 0) g_volt_idx = 0;
            if (g_volt_idx > VOLT_COUNT - 1) g_volt_idx = VOLT_COUNT - 1;
            Update_Scales();
            g_redraw = 1;
            break;
        default:
            g_trig_auto = 0;
            g_trig_v += (float)d * vdiv_table[g_volt_idx] / 10.0f;
            Update_Scales();
            g_redraw = 1;
            break;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  /* LUU Y: KHONG remap SPI1. Man hinh noi PA5 (SCK) / PA7 (MOSI) = SPI1 mac dinh. */
  Encoder_GPIO_Init();
  DMA_ForceCircular();
  SPI1_Pins_Force();

  ST7735_Init();
  
#if DEBUG_MODE
  while (1) {
      ST7735_FillScreen(ST7735_RED);   HAL_Delay(500);
      ST7735_FillScreen(ST7735_GREEN); HAL_Delay(500);
      ST7735_FillScreen(ST7735_BLUE);  HAL_Delay(500);
  }
#endif
#if DISPLAY_SELFTEST
  ST7735_FillScreen(ST7735_RED);   HAL_Delay(250);
  ST7735_FillScreen(ST7735_GREEN); HAL_Delay(250);
  ST7735_FillScreen(ST7735_BLUE);  HAL_Delay(250);
#endif
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_WriteString(10, 60, "Mini Scope", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  ST7735_WriteString(10, 75, "Starting...", Font_7x10, ST7735_CYAN, ST7735_BLACK);
  HAL_Delay(1000);
  ST7735_FillScreen(ST7735_BLACK);

  Update_Scales();
  PWM_Apply();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PWM test ra PA8 */

  inputs_run = 1;
  ADC_ForceTimerTrigger();
  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  Apply_Timebase();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_SIZE);
  ADC_TriggerTimer_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    UI_Process();

    if (cap_ready) {
        cap_ready   = 0;
        g_have_data = 1;
        g_redraw    = 1;
    }

    if (g_redraw && g_have_data) {
        cap_request = 0;
        Analyze();
        if (!g_hold) {
            uint8_t chg = AutoRange_Step();
            if (chg & 1) Apply_Timebase();
        }
        Update_Scales();
        Scope_Render();
        g_redraw = 0;
    }

    Info_Update();

    if (!g_hold && !cap_ready && !cap_request)
        cap_request = 1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DL_A0_Pin|DL_RS_Pin|DL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DL_A0_Pin DL_RS_Pin DL_CS_Pin */
  GPIO_InitStruct.Pin = DL_A0_Pin|DL_RS_Pin|DL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_Pin BUT_HOLD_RUN_Pin */
  GPIO_InitStruct.Pin = SW_Pin|BUT_HOLD_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */