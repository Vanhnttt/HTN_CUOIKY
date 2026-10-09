Tìm hiểu :

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

  
2. Giải thích thuật toán chi tiết

1. Khởi tạo và đi tìm Đỉnh (Max) - Đáy (Min) của sóng
Code quét qua toàn bộ mảng dữ liệu lấy mẫu snap (dài 256 phần tử). Ban đầu gán đáy (adc_mn) bằng số to nhất (4095) và đỉnh (adc_mx) bằng số nhỏ nhất (0). Sau đó dùng vòng lặp for, so sánh từng điểm dữ liệu: cứ thấy thằng nào nhỏ hơn adc_mn thì cập nhật lại đáy, to hơn adc_mx thì cập nhật đỉnh.

2. Dịch các con số máy tính ra số Vôn (Volts) thật
Mã ADC thu được chỉ là các con số vô nghĩa. Ta dùng hàm adc_to_v() để chuyển đổi nó thành số Volt thực tế. Nếu điện áp Đỉnh-Đỉnh (v_pp) quá nhỏ (nhỏ hơn 0.05V), máy sẽ hiểu đó là nhiễu rác (chưa cắm que đo) và bỏ qua không tính toán tần số nữa.

3. Tìm các điểm cắt ngang để tính Tần số (Có Hysteresis chống nhiễu)
Lấy điểm chính giữa của sóng làm ranh giới (mid). Code đặt ra một vùng cách ly (hyst) bằng 1/10 biên độ sóng.
Quét từ đầu đến cuối mảng:  
Khi sóng vọt LÊN qua ngưỡng trên (mid + hyst), code tính là bắt được một cạnh lên (edges++). Nó ghi nhớ vị trí đầu tiên cắt ngưỡng (first) và vị trí cuối cùng cắt ngưỡng (last).
Tín hiệu bắt buộc phải tụt XUỐNG DƯỚI ngưỡng dưới (mid - hyst) thì mới được tính là chuẩn bị cho chu kỳ tiếp theo (biến state về 0). Điều này gọi là Thuật toán Schmitt Trigger, giúp loại bỏ hoàn toàn việc đếm nhầm tần số khi tín hiệu bị rung/nhiễu.

4. Tính ra Tần số cuối cùng (Hz)
Sau khi tìm được các điểm cắt, code tính tần số bằng công thức: f = Tần số lấy mẫu * (Số lượng chu kỳ / Khoảng cách thời gian). Ở code mới này, để tăng tốc độ xử lý cho chip STM32, toàn bộ công thức được tính bằng Toán học số nguyên (không dùng số thập phân), cộng thêm một lượng den / 2 ở tử số để làm tròn số cực kỳ chính xác.

5. Tìm vị trí Trigger (Điểm mỏ neo để bắt đầu vẽ sóng)
Nhiệm vụ này được chuyển ra hàm find_trigger(). Khi bắt đầu vẽ lên màn hình, máy hiện sóng không vẽ bừa, mà nó sẽ quét mảng để tìm xem lúc nào tín hiệu đi từ thấp vọt lên cắt ngang cái mốc trig_lvl (được chỉnh bằng nút vặn Encoder). Khi bắt trúng điểm đó, nó sẽ neo sóng lại, giúp sóng đứng im hiển thị rõ ràng trên màn hình chứ không bị chạy trôi tuột đi.
