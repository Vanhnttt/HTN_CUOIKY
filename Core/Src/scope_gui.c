#include "scope_gui.h"
#include "scope_input.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define HDR_VDIV_X   1
#define HDR_TDIV_X   42
#define HDR_TRIG_X   84
#define HDR_MODE_X   124
#define FTR_POS_X    112

#define HDR_Y1       8
#define BRD_TOP      9
#define BRD_BOT      106
#define FTR_Y0       107
#define DIV_W        16

static char     osd_vdiv[16];
static char     osd_tdiv[16];
static char     osd_trig[24];
static char     osd_mode[8];
static char     osd_f1[64];
static char     osd_f2[64];
static char     osd_pos[16];
static uint16_t border_col = C_CYAN;

static uint8_t  hl_on = 0;
static int      hl_x0 = 0, hl_x1 = 0, hl_y0 = 0, hl_y1 = 0;
static uint16_t hl_col = C_CYAN;

static uint16_t trig_col = C_TRIG;
static int      gnd_y    = SCO_CY;
static uint16_t gnd_col  = C_GREEN;

static uint16_t strip_buf[STRIP_PIXELS];

static void fmt_volt(char *d, size_t n, const char *pfx, float v)
{
    if (v < 0.0f) v = 0.0f;
    uint32_t cv = (uint32_t)(v * 100.0f + 0.5f);
    snprintf(d, n, "%s%lu.%02luV", pfx, (unsigned long)(cv / 100u), (unsigned long)(cv % 100u));
}

static void fmt_vsigned(char *d, size_t n, const char *pfx, float v)
{
    int32_t cv = (int32_t)(v * 100.0f + (v < 0.0f ? -0.5f : 0.5f));
    uint32_t a = (uint32_t)(cv < 0 ? -cv : cv);
    snprintf(d, n, "%s%s%lu.%02luV", pfx, (cv < 0) ? "-" : "", (unsigned long)(a / 100u), (unsigned long)(a % 100u));
}

static void fmt_hz(char *d, size_t n, const char *pfx, uint32_t hz)
{
    if (hz >= 1000u)
        snprintf(d, n, "%s%lu.%02lukHz", pfx, (unsigned long)(hz / 1000u), (unsigned long)((hz % 1000u) / 10u));
    else
        snprintf(d, n, "%s%luHz", pfx, (unsigned long)hz);
}

void Scope_GUI_Init(void)
{
    for (int i = 0; i < STRIP_PIXELS; i++) strip_buf[i] = C_BLACK;
}

void Scope_GUI_BuildOSD(uint8_t snap_tb, uint8_t vd_idx, uint16_t trig_lvl, int16_t center_mv, uint8_t adj_mode, uint8_t mode_fft, uint8_t hold)
{
    (void)hold;
    uint32_t sample_us = (uint32_t)tb_arr[snap_tb] + 1u;
    uint32_t fs_hz     = 1000000u / sample_us;
    uint32_t tdiv_us   = sample_us * DIV_W;

    border_col = mode_col[adj_mode];

    if (tdiv_us >= 1000u)
    {
        uint32_t t10 = tdiv_us / 100u;
        snprintf(osd_tdiv, sizeof(osd_tdiv), "%lu.%lums", (unsigned long)(t10 / 10u), (unsigned long)(t10 % 10u));
    }
    else
    {
        snprintf(osd_tdiv, sizeof(osd_tdiv), "%luus", (unsigned long)tdiv_us);
    }

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

    snprintf(osd_vdiv, sizeof(osd_vdiv), "%umV", (unsigned)vdiv_mv[vd_idx]);
    fmt_vsigned(osd_trig, sizeof(osd_trig), "T", SigCalc_AdcToV(trig_lvl));
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

void Scope_GUI_UpdateHighlight(uint8_t adj_mode)
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
        int w = (int)strlen(s) * (5 + 1) - 1;
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

static inline uint16_t compose(int x, int y, int trig_y, uint8_t adj_mode, uint8_t mode_fft, uint8_t hold)
{
    uint16_t c;

    if (y <= HDR_Y1 || y >= FTR_Y0) c = C_DARK;
    else if (y == BRD_TOP)          c = border_col;
    else if (y == BRD_BOT)          c = C_GRAY;
    else                            c = C_BLACK;

    if (y >= SCO_Y0 && y <= SCO_Y1)
    {
        int gy = y - SCO_Y0;
        uint8_t on_v = ((x % DIV_W) == 0) || (x == LCD_W - 1);
        uint8_t on_h = ((gy % DIV_H) == 0);

        if (on_v && (gy % 4) == 0) c = C_GRID;
        if (on_h && (x  % 4) == 0) c = C_GRID;
        if (on_v && on_h)          c = C_AXIS;

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

            if (x < 6)
            {
                int dist = abs(y - gnd_y);
                int wing = 2 - x/2;
                if (dist == 0)              c = gnd_col;
                if (x == 0 && dist <= 2)    c = gnd_col;
                if (x <= 2 && dist == wing) c = gnd_col;
            }

            if (x >= LCD_W - 6)
            {
                int xr   = LCD_W - 1 - x;
                int dist = abs(y - trig_y);
                int wing = 2 - xr/2;
                if (dist == 0)               c = trig_col;
                if (xr == 0 && dist <= 2)    c = trig_col;
                if (xr <= 2 && dist == wing) c = trig_col;
            }

            if (y <= SCO_Y0 + 3 && abs(x - SCO_CX) <= 3 - (y - SCO_Y0)) c = C_TRIG;
        }
    }

    if (hl_on && x >= hl_x0 && x <= hl_x1 && y >= hl_y0 && y <= hl_y1)
        c = hl_col;

    if (y >= 1 && y <= 7)
    {
        if (ST7735_TextHit(x, y, HDR_VDIV_X, 1, osd_vdiv))
            c = (adj_mode == MODE_VOLT) ? C_BLACK : C_WHITE;
        if (ST7735_TextHit(x, y, HDR_TDIV_X, 1, osd_tdiv))
            c = (adj_mode == MODE_TIME) ? C_BLACK : (mode_fft ? C_CYAN : C_GREEN);
        if (ST7735_TextHit(x, y, HDR_TRIG_X, 1, osd_trig))
            c = (adj_mode == MODE_TRIG) ? C_BLACK : C_TRIG;
        if (ST7735_TextHit(x, y, HDR_MODE_X, 1, osd_mode)) c = C_WHITE;
        if (hold && ST7735_TextHit(x, y, 143, 1, "HLD")) c = C_YELLOW;
    }

    if (y >= FTR_Y0 + 2 && y <= FTR_Y0 + 8)
        if (ST7735_TextHit(x, y, 2, FTR_Y0 + 2, osd_f1)) c = C_CYAN;
    if (y >= FTR_Y0 + 12 && y <= FTR_Y0 + 18)
    {
        if (ST7735_TextHit(x, y, 2, FTR_Y0 + 12, osd_f2)) c = C_YELLOW;
        if (ST7735_TextHit(x, y, FTR_POS_X, FTR_Y0 + 12, osd_pos))
            c = (adj_mode == MODE_POS) ? C_BLACK : C_ORANGE;
    }

    return c;
}

void Scope_GUI_Render(uint16_t trig_lvl, int16_t center_mv, uint8_t vd_idx, uint8_t adj_mode, uint8_t mode_fft, uint8_t hold)
{
    int trig_y = SigCalc_AdcToY(trig_lvl, center_mv, vd_idx);

    int32_t tmv = SigCalc_AdcToMv(trig_lvl);
    int32_t ty  = SCO_CY - ((tmv - (int32_t)center_mv) * DIV_H) / (int32_t)vdiv_mv[vd_idx];
    trig_col = (ty < SCO_Y0 - 1 || ty > SCO_Y1 + 1) ? C_YELLOW : C_TRIG;

    int32_t gy0 = SCO_CY + ((int32_t)center_mv * DIV_H) / (int32_t)vdiv_mv[vd_idx];
    gnd_y   = SigCalc_MvToY(0, center_mv, vd_idx);
    gnd_col = (gy0 < SCO_Y0 - 1 || gy0 > SCO_Y1 + 1) ? C_YELLOW : C_GREEN;

    for (int s = 0; s < LCD_W / STRIP_W; s++)
    {
        int x0 = s * STRIP_W;

        for (int y = 0; y < LCD_H; y++)
            for (int xi = 0; xi < STRIP_W; xi++)
                strip_buf[y * STRIP_W + xi] = compose(x0 + xi, y, trig_y, adj_mode, mode_fft, hold);

        ST7735_SetWindow((uint16_t)x0, 0, (uint16_t)(x0 + STRIP_W - 1), LCD_H - 1);
        ST7735_WriteData((uint8_t *)strip_buf, (uint16_t)(STRIP_PIXELS * 2));

        Scope_Input_PollButtons(); /* Polling nút giữa các dải strip tăng độ nhạy phản hồi */
    }
}
