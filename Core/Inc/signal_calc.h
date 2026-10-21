/* ==============================================================================
 * KHỐI XỬ LÝ & TÍNH TOÁN TÍN HIỆU (SIGNAL CALCULATION MODULE)
 * Đề tài V07: "Xử lý: tìm điểm trigger, tính Vmin/Vmax/Vpp, tần số (đếm cắt ngưỡng)"
 * ============================================================================== */

#ifndef __SIGNAL_CALC_H
#define __SIGNAL_CALC_H

#include "main.h"
#include "st7735.h"
#include "scope_acquisition.h"

/* ---------------- Cấu hình mạch phân áp & bảo vệ đầu vào (AFE) ---------------- */
#define AFE_INVERT        0      /* 1 nếu mạch đảo pha (V vào tăng thì ADC giảm) */
#define AFE_ZERO_MV       0      /* Điện áp tại PA0 (mV) khi đầu vào nối GND */
#define AFE_GAIN_X1000    1000   /* Hệ số khuếch đại AFE x 1000 */

/* ---------------- Thang đo Volt/Div ---------------- */
#define VD_COUNT          7
extern const uint16_t vdiv_mv[VD_COUNT];

/* ---------------- Kích thước hiển thị dạng sóng ---------------- */
#define SCO_Y0       10
#define SCO_Y1       105
#define SCO_H        96
#define DIV_H        16
#define SCO_CX       80
#define SCO_CY       58

/* ---------------- Biến kết quả tính toán ---------------- */
extern float    v_min;          /* Điện áp nhỏ nhất (V) */
extern float    v_max;          /* Điện áp lớn nhất (V) */
extern float    v_pp;           /* Điện áp đỉnh - đỉnh: Vpp = Vmax - Vmin (V) */
extern uint32_t sig_freq_hz;    /* Tần số tính theo thuật toán đếm cắt ngưỡng (Hz) */
extern uint8_t  wave_lo[LCD_W]; /* Biên dưới dạng sóng trên màn hình */
extern uint8_t  wave_hi[LCD_W]; /* Biên trên dạng sóng trên màn hình */

/* ---------------- CÁC HÀM TÍNH TOÁN THEO YÊU CẦU ĐỀ TÀI ---------------- */

/* 1. Chuyển đổi giá trị ADC sang điện áp thật (mV và V) qua mạch AFE */
int32_t  SigCalc_AdcToMv(uint16_t raw_adc);
float    SigCalc_AdcToV(uint16_t raw_adc);

/* 2. Ánh xạ điện áp sang tọa độ Y trên màn hình ST7735 */
int      SigCalc_MvToY(int32_t mv, int16_t center_mv, uint8_t vd_idx);
int      SigCalc_AdcToY(uint16_t raw_adc, int16_t center_mv, uint8_t vd_idx);
int16_t  SigCalc_ClampCenter(int16_t center_mv, uint8_t vd_idx);

/* 3. TÍNH TOÁN: Vmin, Vmax, Vpp & Tần số bằng thuật toán đếm cắt ngưỡng */
void     SigCalc_Tinh_Vmin_Vmax_Vpp(const uint16_t *snap_buf);
void     SigCalc_TinhTanSo_DemCatNguong(const uint16_t *snap_buf, uint8_t snap_tb);
void     SigCalc_XuLyTatCa(const uint16_t *snap_buf, uint8_t snap_tb);

/* 4. TÌM ĐIỂM TRIGGER CẠNH LÊN (Rising Edge Trigger with Hysteresis) */
uint16_t SigCalc_TimDiemTrigger(const uint16_t *snap_buf, uint16_t trig_lvl);

/* 5. DỰNG TỌA ĐỘ DẠNG SÓNG HIỂN THỊ (Precompute Wave) */
void     SigCalc_ChuanBiDangSong(const uint16_t *snap_buf, uint16_t trig_lvl, int16_t center_mv, uint8_t vd_idx);

#endif /* __SIGNAL_CALC_H */
