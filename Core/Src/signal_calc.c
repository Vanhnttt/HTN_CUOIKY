/* ==============================================================================
 * KHỐI XỬ LÝ & TÍNH TOÁN TÍN HIỆU (SIGNAL CALCULATION MODULE)
 * Đề tài V07: "Xử lý: tìm điểm trigger, tính Vmin/Vmax/Vpp, tần số (đếm cắt ngưỡng)"
 * ============================================================================== */

#include "signal_calc.h"
#include <stdlib.h>

const uint16_t vdiv_mv[VD_COUNT] = { 2000, 1000, 500, 200, 100, 50, 20 };

/* Biến lưu kết quả tính toán */
float    v_min       = 0.0f;
float    v_max       = 0.0f;
float    v_pp        = 0.0f;
uint32_t sig_freq_hz = 0;
uint8_t  wave_lo[LCD_W];
uint8_t  wave_hi[LCD_W];

static uint16_t adc_min_raw = 4095;
static uint16_t adc_max_raw = 0;

/* --------------------------------------------------------------------------
 * 1. Chuyển đổi ADC raw sang điện áp mV và V (có bù sai số mạch phân áp AFE)
 * -------------------------------------------------------------------------- */
int32_t SigCalc_AdcToMv(uint16_t raw_adc)
{
    uint16_t v = AFE_INVERT ? (uint16_t)(4095u - raw_adc) : raw_adc;
    int32_t norm_mv = (int32_t)((uint32_t)v * 3300u / 4095u);
    int32_t zero_mv = AFE_INVERT ? (3300 - AFE_ZERO_MV) : AFE_ZERO_MV;
    return ((norm_mv - zero_mv) * 1000) / AFE_GAIN_X1000;
}

float SigCalc_AdcToV(uint16_t raw_adc)
{
    return (float)SigCalc_AdcToMv(raw_adc) / 1000.0f;
}

/* --------------------------------------------------------------------------
 * 2. Chuyển đổi điện áp sang tọa độ dòng Y trên màn hình ST7735 (160x128)
 * -------------------------------------------------------------------------- */
int SigCalc_MvToY(int32_t mv, int16_t center_mv, uint8_t vd_idx)
{
    int32_t y = SCO_CY - ((mv - (int32_t)center_mv) * DIV_H) / (int32_t)vdiv_mv[vd_idx];
    if (y < SCO_Y0) y = SCO_Y0;
    if (y > SCO_Y1) y = SCO_Y1;
    return (int)y;
}

int SigCalc_AdcToY(uint16_t raw_adc, int16_t center_mv, uint8_t vd_idx)
{
    return SigCalc_MvToY(SigCalc_AdcToMv(raw_adc), center_mv, vd_idx);
}

int16_t SigCalc_ClampCenter(int16_t center_mv, uint8_t vd_idx)
{
    int lo = (int)SigCalc_AdcToMv(0)    - 3 * (int)vdiv_mv[vd_idx];
    int hi = (int)SigCalc_AdcToMv(4095) + 3 * (int)vdiv_mv[vd_idx];
    if (lo > hi) { int t = lo; lo = hi; hi = t; }
    int b = (int)center_mv;
    if (b < lo) b = lo;
    if (b > hi) b = hi;
    return (int16_t)b;
}

/* --------------------------------------------------------------------------
 * 3. TÍNH TOÁN: Vmin, Vmax, Vpp
 *    Tìm mẫu nhỏ nhất, lớn nhất trong 256 mẫu và tính hiệu số đỉnh-đỉnh Vpp
 * -------------------------------------------------------------------------- */
void SigCalc_Tinh_Vmin_Vmax_Vpp(const uint16_t *snap_buf)
{
    adc_min_raw = 4095;
    adc_max_raw = 0;

    for (int i = 0; i < SNAPSHOT_LEN; i++)
    {
        uint16_t v = AFE_INVERT ? (uint16_t)(4095u - snap_buf[i]) : snap_buf[i];
        if (v < adc_min_raw) adc_min_raw = v;
        if (v > adc_max_raw) adc_max_raw = v;
    }

    v_min = SigCalc_AdcToV(adc_min_raw);
    v_max = SigCalc_AdcToV(adc_max_raw);
    v_pp  = v_max - v_min;
}

/* --------------------------------------------------------------------------
 * 4. TÍNH TOÁN: Tần số bằng thuật toán ĐẾM CẮT NGƯỠNG (Zero-Crossing with Hysteresis)
 *    Xác định ngưỡng giữa (mid) và dải trễ (hyst) để lọc nhiễu, đếm số chu kỳ
 * -------------------------------------------------------------------------- */
void SigCalc_TinhTanSo_DemCatNguong(const uint16_t *snap_buf, uint8_t snap_tb)
{
    sig_freq_hz = 0;
    if (v_pp < 0.05f) return; /* Tín hiệu quá bé hoặc không có tín hiệu */

    uint16_t mid  = adc_min_raw + (adc_max_raw - adc_min_raw) / 2;
    uint16_t hyst = (adc_max_raw - adc_min_raw) / 10;
    if (hyst < 20) hyst = 20;

    int first = -1, last = -1;
    uint8_t edges = 0;
    uint8_t state = (snap_buf[0] >= mid) ? 1 : 0;

    for (int i = 1; i < SNAPSHOT_LEN; i++)
    {
        uint16_t v = AFE_INVERT ? (uint16_t)(4095u - snap_buf[i]) : snap_buf[i];

        /* Cắt ngưỡng cạnh lên: tín hiệu vượt qua (mid + hyst) */
        if (state == 0 && v >= (uint16_t)(mid + hyst))
        {
            state = 1;
            if (first < 0) first = i;
            last = i; 
            edges++;
        }
        /* Cắt ngưỡng cạnh xuống: tín hiệu tụt xuống dưới (mid - hyst) */
        else if (state == 1 && v <= (uint16_t)(mid - hyst))
        {
            state = 0;
        }
    }

    /* Nếu bắt được từ 2 cạnh lên trở lên thì tính tần số chính xác */
    if (edges >= 2 && last > first)
    {
        uint32_t fs  = Scope_Acquisition_GetSamplingFreq(snap_tb);
        uint32_t num = fs * (uint32_t)(edges - 1);
        uint32_t den = (uint32_t)(last - first);
        sig_freq_hz  = (num + den / 2u) / den;
    }
}

void SigCalc_XuLyTatCa(const uint16_t *snap_buf, uint8_t snap_tb)
{
    SigCalc_Tinh_Vmin_Vmax_Vpp(snap_buf);
    SigCalc_TinhTanSo_DemCatNguong(snap_buf, snap_tb);
}

/* --------------------------------------------------------------------------
 * 5. TÌM ĐIỂM TRIGGER CẠNH LÊN (Rising Edge Trigger with Hysteresis)
 *    Tìm mẫu tín hiệu cắt qua mức trig_lvl theo chiều tăng để cố định sóng
 * -------------------------------------------------------------------------- */
uint16_t SigCalc_TimDiemTrigger(const uint16_t *snap_buf, uint16_t trig_lvl)
{
    int hyst = (trig_lvl < 60) ? ((int)trig_lvl / 2) : 30;

    /* Tìm trong vùng có thể căn giữa màn hình */
    for (int i = LCD_W / 2; i < (SNAPSHOT_LEN - LCD_W / 2 - 1); i++)
    {
        uint16_t v_prev = AFE_INVERT ? (uint16_t)(4095u - snap_buf[i - 1]) : snap_buf[i - 1];
        uint16_t v_curr = AFE_INVERT ? (uint16_t)(4095u - snap_buf[i])     : snap_buf[i];

        if ((int)v_prev < ((int)trig_lvl - hyst) && (int)v_curr >= (int)trig_lvl)
        {
            return (uint16_t)i; /* Đã tìm thấy điểm Trigger cạnh lên */
        }
    }
    return SNAPSHOT_LEN / 2;    /* Không tìm thấy trigger: để sóng chạy tự do ở giữa */
}

/* --------------------------------------------------------------------------
 * 6. DỰNG TỌA ĐỘ DẠNG SÓNG ĐỂ HIỂN THỊ (Precompute Wave)
 *    Nối các điểm Y liên tiếp thành dải biên wave_lo - wave_hi không bị đứt nét
 * -------------------------------------------------------------------------- */
void SigCalc_ChuanBiDangSong(const uint16_t *snap_buf, uint16_t trig_lvl, int16_t center_mv, uint8_t vd_idx)
{
    uint16_t start = (uint16_t)(SigCalc_TimDiemTrigger(snap_buf, trig_lvl) - LCD_W / 2);
    int prev_y = SigCalc_AdcToY(snap_buf[start], center_mv, vd_idx);

    for (int x = 0; x < LCD_W; x++)
    {
        uint16_t idx = start + (uint16_t)x;
        if (idx >= SNAPSHOT_LEN) idx = SNAPSHOT_LEN - 1;
        int cy = SigCalc_AdcToY(snap_buf[idx], center_mv, vd_idx);

        wave_lo[x] = (uint8_t)((cy < prev_y) ? cy : prev_y);
        wave_hi[x] = (uint8_t)((cy > prev_y) ? cy : prev_y);
        prev_y = cy;
    }
}
