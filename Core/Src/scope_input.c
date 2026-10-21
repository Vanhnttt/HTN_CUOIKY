#include "scope_input.h"
#include "scope_acquisition.h"
#include "signal_calc.h"
#include "tim.h"

const uint16_t mode_col[MODE_COUNT] = { C_CYAN, C_GREEN, C_RED, C_ORANGE };

uint8_t      tb_idx      = 5;
uint8_t      vd_idx      = 1;
int16_t      center_mv   = 0;
uint16_t     trig_lvl    = 2048;
AdjustMode_t adj_mode    = MODE_TIME;
uint8_t      hold        = 0;
uint8_t      mode_fft    = 0;
uint8_t      need_redraw = 1;

static volatile uint8_t req_edit = 0;
static volatile uint8_t req_fft  = 0;
static volatile uint8_t req_hold = 0;

void Scope_Input_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void Scope_Input_PollButtons(void)
{
    static uint8_t  armed = 0;
    static uint8_t  sw_down = 0, long_fired = 0;
    static uint32_t sw_t0 = 0;
    static uint8_t  hold_prev = 1;
    static uint32_t hold_t = 0;

    uint32_t now = HAL_GetTick();

    /* Nút ấn Encoder PB8 */
    uint8_t sw = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET);
    if (!armed)
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
        long_fired = 1; req_fft = 1;               /* Nhấn giữ: OSC <-> FFT */
    }
    else if (!sw && sw_down)
    {
        sw_down = 0;
        if (!long_fired && (now - sw_t0) >= 20u) req_edit = 1; /* Nhấn nhả: đổi Mode */
    }

    /* Nút HOLD/RUN PB9 */
    uint8_t h = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    if (hold_prev == 1 && h == 0 && (now - hold_t) > 50u)
    {
        req_hold = 1; hold_t = now;
    }
    hold_prev = h;
}

void Scope_Input_Handle(void)
{
    static uint16_t last_raw = 0;

    uint16_t raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t  diff = (int16_t)(uint16_t)(raw - last_raw);
    int16_t  d    = diff / 4;

    if (d != 0)
    {
        last_raw = (uint16_t)(last_raw + (uint16_t)(d * 4));
        d = (int16_t)(d * ENC_DIR);

        switch (adj_mode)
        {
        case MODE_TIME:
            if (!hold)
            {
                int n = (int)tb_idx + d;
                if (n < 0) n = 0;
                if (n > TB_COUNT - 1) n = TB_COUNT - 1;
                if ((uint8_t)n != tb_idx)
                {
                    tb_idx = (uint8_t)n;
                    Scope_Acquisition_SetTimebase(tb_idx);
                }
            }
            break;

        case MODE_VOLT:
        {
            int n = (int)vd_idx - d;
            if (n < 0) n = 0;
            if (n > VD_COUNT - 1) n = VD_COUNT - 1;
            vd_idx = (uint8_t)n;
            center_mv = SigCalc_ClampCenter(center_mv, vd_idx);
            need_redraw = 1;
            break;
        }

        case MODE_TRIG:
        {
            int step = (int)((float)vdiv_mv[vd_idx] * ((float)AFE_GAIN_X1000 / 1000.0f)
                             * (4095.0f / 3300.0f) / 8.0f);
            if (step < 1) step = 1;
            int n = (int)trig_lvl + d * step;
            if (n < 16)   n = 16;
            if (n > 4079) n = 4079;
            trig_lvl = (uint16_t)n;
            need_redraw = 1;
            break;
        }

        case MODE_POS:
        {
            int step = (int)vdiv_mv[vd_idx] / 5;
            center_mv = (int16_t)((int)center_mv - d * step);
            center_mv = SigCalc_ClampCenter(center_mv, vd_idx);
            need_redraw = 1;
            break;
        }

        default: break;
        }
    }

    Scope_Input_PollButtons();

    if (req_edit)
    {
        req_edit = 0;
        if (!mode_fft) adj_mode = (AdjustMode_t)((adj_mode + 1u) % MODE_COUNT);
        need_redraw = 1;
    }
    if (req_fft)
    {
        req_fft = 0;
        mode_fft ^= 1;
        if (mode_fft) adj_mode = MODE_TIME;
        need_redraw = 1;
    }
    if (req_hold)
    {
        req_hold = 0;
        hold ^= 1;
        need_redraw = 1;
    }
}
