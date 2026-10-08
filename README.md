/* Struct lưu trữ các kết quả tính toán của bạn */
typedef struct {
    int     trig_idx, mid_code;
    uint8_t triggered, valid, over, too_fast;
    float   vpp, vmax, vmin, freq, p_smp;
} Meas_t;
static Meas_t meas;

static void Analyze(void) {
    uint16_t mn = 0xFFFF, mx = 0;
    
    // =========================================================
    // NHIỆM VỤ 1: TÍNH VMAX, VMIN, VPP
    // =========================================================
    /* 1.1 Quét toàn bộ mảng lấy mẫu để tìm giá trị ADC lớn nhất (mx) và nhỏ nhất (mn) */
    for (int i = 0; i < HALF_SIZE; i++) {
        uint16_t s = cap_buf[i];
        if (s < mn) mn = s;
        if (s > mx) mx = s;
    }
    
    const float vpc = VOLT_PER_CODE; // Hệ số chuyển đổi từ mã ADC sang Volts
    int pp = (int)mx - (int)mn;      // Biên độ đỉnh-đỉnh (đơn vị ADC code)
    
    meas.valid    = (pp >= SIGNAL_MIN_PP); // Bỏ qua nếu tín hiệu quá nhỏ (nhiễu)
    meas.mid_code = ((int)mx + (int)mn) / 2; // Điểm giữa của tín hiệu

    /* 1.2 Đổi từ giá trị ADC sang điện áp thực tế (Volts) */
    meas.vmax = ((int)mx - ADC_ZERO_CODE) * vpc;
    meas.vmin = ((int)mn - ADC_ZERO_CODE) * vpc;
    meas.vpp  = meas.valid ? (float)pp * vpc : 0.0f;

    // =========================================================
    // NHIỆM VỤ 2: TÌM ĐIỂM TRIGGER (CẠNH LÊN VỚI HYSTERESIS)
    // =========================================================
    /* Tính toán Hysteresis (độ trễ chống nhiễu) - Dựa trên biên độ tín hiệu */
    int hy = pp / 8;
    if (hy < HYST_MIN) hy = HYST_MIN;
    if (hy > 40)       hy = 40;

    meas.trig_idx  = 0;
    meas.triggered = 0;
    uint8_t armed = 0; // Cờ báo hiệu tín hiệu đã sụt xuống dưới ngưỡng châm ngòi
    
    for (int i = 0; i <= HALF_SIZE - WAVE_POINTS; i++) {
        int s = cap_buf[i];
        // Điều kiện 1: Tín hiệu phải tụt xuống DƯỚI (ngưỡng - độ trễ)
        if (s < g_trig_code - hy) {
            armed = 1; 
        } 
        // Điều kiện 2: Tín hiệu vọt LÊN cắt ngang ngưỡng (Rising Edge)
        else if (armed && s >= g_trig_code) {
            meas.trig_idx  = i; // Lưu lại vị trí Trigger để vẽ lên màn hình
            meas.triggered = 1;
            break;
        }
    }

    // =========================================================
    // NHIỆM VỤ 3: TÍNH TẦN SỐ (BẰNG CÁCH TÌM ĐIỂM CẮT NGƯỠNG)
    // =========================================================
    meas.freq = 0.0f;
    if (meas.valid) {
        int thr = meas.mid_code; // Dùng điểm giữa làm ngưỡng cắt (Threshold)
        int hf  = pp / 8;
        if (hf < HYST_MIN) hf = HYST_MIN;
        
        uint8_t arm = (cap_buf[0] < thr - hf);
        int   edges = 0;
        float t_first = 0.0f, t_last = 0.0f;
        
        for (int i = 1; i < HALF_SIZE; i++) {
            int s = cap_buf[i];
            if (s < thr - hf) {
                arm = 1;
            } else if (arm && s >= thr) { // Bắt được 1 chu kỳ cắt ngưỡng
                /* Thuật toán Nội suy tuyến tính (Linear Interpolation) */
                float prev = (float)cap_buf[i - 1];
                float den  = (float)s - prev;
                // Tính phần thập phân của thời gian giữa 2 điểm lấy mẫu
                float frac = (den > 0.0f) ? ((float)thr - prev) / den : 0.0f; 
                float t    = (float)(i - 1) + frac; // Vị trí thời gian chính xác
                
                if (edges == 0) t_first = t; // Thời điểm cắt ngưỡng đầu tiên
                t_last = t;                  // Thời điểm cắt ngưỡng cuối cùng
                edges++;
                arm = 0;
            }
        }
        
        /* Tính chu kỳ trung bình và suy ra tần số */
        if (edges >= 2) {
            // Khoảng cách trung bình giữa các cạnh lên (đơn vị: số mẫu)
            meas.p_smp = (t_last - t_first) / (float)(edges - 1); 
            
            if (meas.p_smp >= MIN_SMP_PERIOD)
                // f = 1 / T. Đổi số mẫu thành thời gian thực tế (us) dựa theo Time/Div
                meas.freq = 1.0e6f / (meas.p_smp * (float)ts_us_table[cap_ts_idx]);
        }
    }
}
