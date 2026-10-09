
/* =========================================================
 * NHIỆM VỤ 1 & 3: TÍNH VMAX, VMIN, VPP VÀ TẦN SỐ
 * ========================================================= */
static void process_measurements(void)
{
    // 1. Tìm Đỉnh (Max) và Đáy (Min) của mã ADC
    uint16_t adc_mn = 4095, adc_mx = 0;
    for (int i = 0; i < FFT_N; i++) {
        if (snap[i] < adc_mn) adc_mn = snap[i];
        if (snap[i] > adc_mx) adc_mx = snap[i];
    }
    
    // 2. Đổi mã ADC sang điện áp thực tế (Volt)
    v_min = adc_to_v(adc_mn);
    v_max = adc_to_v(adc_mx);
    v_pp  = v_max - v_min;
    sig_freq_hz = 0;
    if (v_pp < 0.05f) return; // Bỏ qua nếu nhiễu (biên độ quá bé < 0.05V)
    // 3. Tính tần số (Đếm số chu kỳ bằng Hysteresis)
    uint16_t mid  = adc_mn + (adc_mx - adc_mn) / 2;  // Điểm giữa
    uint16_t hyst = (adc_mx - adc_mn) / 10;          // Độ trễ bằng 1/10 biên độ sóng
    if (hyst < 20) hyst = 20;                        // Trễ tối thiểu là 20 để chống rung
    int first = -1, last = -1;
    uint8_t edges = 0, state = (snap[0] >= mid) ? 1 : 0;
    for (int i = 1; i < FFT_N; i++) {
        if (state == 0 && snap[i] >= (uint16_t)(mid + hyst)) {
            state = 1; // Sóng vọt lên qua ngưỡng trên -> tính là 1 lần cắt
            if (first < 0) first = i;
            last = i; 
            edges++;
        }
        else if (state == 1 && snap[i] <= (uint16_t)(mid - hyst)) {
            state = 0; // Sóng tụt xuống qua ngưỡng dưới -> chuẩn bị đếm nhịp mới
        }
    }
    // 4. Quy đổi ra Tần số (Hz) bằng Toán học số nguyên
    if (edges >= 2 && last > first) {
        uint32_t fs  = 1000000u / ((uint32_t)tb_arr[snap_tb] + 1u); // Tần số lấy mẫu
        uint32_t num = fs * (uint32_t)(edges - 1);
        uint32_t den = (uint32_t)(last - first);
        sig_freq_hz  = (num + den / 2u) / den; // Tần số tín hiệu (Hz)
    }
}
/* =========================================================
 * NHIỆM VỤ 2: TÌM ĐIỂM TRIGGER KHI VẼ MÀN HÌNH
 * ========================================================= */
static uint16_t find_trigger(void)
{
    // Tính dải trễ (Hysteresis) cho riêng phần Trigger
    int hyst = (trig_lvl < 60) ? ((int)trig_lvl / 2) : 30;
    
    // Tìm điểm mà sóng đi từ DƯỚI (ngưỡng - trễ) vọt LÊN TỚI ngưỡng
    for (int i = LCD_W / 2; i < (FFT_N - LCD_W / 2 - 1); i++) {
        if ((int)snap[i-1] < ((int)trig_lvl - hyst) && (int)snap[i]  >= (int)trig_lvl)
            return (uint16_t)i; // Bắt được điểm Trigger
    }
    return FFT_N / 2; // Nếu không tìm thấy, trả về điểm giữa (sóng tự do)
}

