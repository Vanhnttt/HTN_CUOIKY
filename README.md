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


Giải thích :
1. Khai báo cái "Hộp đựng kết quả" (Struct Meas_t).
typedef struct { ... } Meas_t;
static Meas_t meas;

Đầu tiên, code tạo ra một cái hộp có tên là meas (viết tắt của measurement). Cái hộp này chứa nhiều ngăn để lưu lại mọi kết quả tính toán: vị trí trigger, điện áp Vmax, Vmin, Vpp, và Tần số. Các hàm vẽ lên màn hình sau này chỉ việc mở cái hộp này ra để lấy số liệu in lên màn hình.

2. Khởi tạo và đi tìm Đỉnh (Max) - Đáy (Min) của sóng
uint16_t mn = 0xFFFF, mx = 0;
for (int i = 0; i < HALF_SIZE; i++) {
    uint16_t s = cap_buf[i];
    if (s < mn) mn = s;
    if (s > mx) mx = s;
}

Khởi tạo biến đáy mn (min) bằng giá trị to nhất có thể, và biến đỉnh mx (max) bằng 0. Sau đó, vòng lặp for chạy qua toàn bộ mảng dữ liệu ADC thu được (cap_buf). Cứ thấy thằng nào nhỏ hơn mn thì cập nhật lại mn, thấy thằng nào to hơn mx thì cập nhật lại mx. Chạy xong vòng này, ta tìm được đúng Đáy và Đỉnh của sóng.

3. Kiểm tra xem có sóng thật không, hay chỉ là nhiễu
int pp = (int)mx - (int)mn;
meas.valid = (pp >= SIGNAL_MIN_PP);
meas.mid_code = ((int)mx + (int)mn) / 2;

Lấy Đỉnh trừ Đáy ta được Biên độ (pp). Nếu biên độ này quá bé (nhỏ hơn ngưỡng SIGNAL_MIN_PP), máy sẽ coi đây là nhiễu li ti do chưa cắm que đo (meas.valid = 0). Đồng thời, nó tính luôn điểm chính giữa của sóng (mid_code) bằng cách lấy trung bình cộng Đỉnh và Đáy.

4. Dịch các con số máy tính ra số Vôn (Volts) thật
meas.vmax = ((int)mx - ADC_ZERO_CODE) * vpc;
meas.vmin = ((int)mn - ADC_ZERO_CODE) * vpc;
meas.vpp  = meas.valid ? (float)pp * vpc : 0.0f;

Mã ADC chỉ là các con số vô nghĩa (từ 0-4095). Code tiến hành trừ đi mức gốc (mức 0V của mạch - ADC_ZERO_CODE), sau đó nhân với hệ số chuyển đổi (vpc). Ta thu được điện áp Vmax, Vmin và Vpp thực tế (đơn vị là Volt) để hiện lên màn hình.

5. Tạo độ trễ (Hysteresis) để chống bắt Trigger nhầm
int hy = pp / 8;
if (hy < HYST_MIN) hy = HYST_MIN;
if (hy > 40)       hy = 40;

Nếu sóng bị nhiễu răng cưa, việc tìm điểm bắt đầu vẽ (Trigger) sẽ bị rung. Code tính ra một khoảng trễ (hy) bằng 1/8 biên độ sóng (có chặn giới hạn min/max). Khoảng trễ này giống như một "vùng cách ly".

6. Đi tìm vị trí Trigger (Điểm bắt đầu vẽ sóng lên màn hình)
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

Mục tiêu là tìm ra lúc nào sóng đang có xu hướng đi lên (cạnh lên) và cắt ngang mức điện áp Trigger (g_trig_code). Vòng lặp dò từng điểm dữ liệu:

Nếu sóng tụt sâu xuống DƯỚI vùng cách ly, cờ armed được bật lên 1 (Tức là: Súng đã lên đạn, sẵn sàng bắn).
Chỉ khi súng ĐÃ LÊN ĐẠN (armed = 1) mà sóng vọt lên cắt ngang ngưỡng Trigger, thì mới tính là Bắn! (triggered = 1). Vị trí bắn (i) được lưu vào trig_idx để báo cho màn hình biết: "Hãy bắt đầu vẽ từ điểm này!".

7. Tìm các điểm cắt ngang để tính Tần số
int thr = meas.mid_code; 
// ...
for (int i = 1; i < HALF_SIZE; i++) {
    // ... logic tìm điểm cắt tương tự hàm trigger ...
    if (arm && s >= thr) { 
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

Lấy điểm chính giữa của sóng làm ranh giới (thr). Quét từ đầu đến cuối mảng, cứ mỗi lần sóng vọt từ dưới lên cắt ngang ranh giới này, ta tính đó là 1 chu kỳ (edges++). Nhưng để đo cực kỳ chính xác, code không lấy vị trí nguyên (i) mà dùng công thức tính khoảng cách thập phân frac. Ví dụ: Điểm lấy mẫu thứ 5 nằm dưới ngưỡng, điểm thứ 6 nằm trên ngưỡng. Công thức này tính ra điểm giao cắt thực sự nằm ở 5.4. Code lưu lại thời điểm giao cắt đầu tiên (t_first) và lần giao cắt cuối cùng (t_last).

8. Tính ra Tần số cuối cùng (Hz)
if (edges >= 2) {
    meas.p_smp = (t_last - t_first) / (float)(edges - 1); 
    if (meas.p_smp >= MIN_SMP_PERIOD)
        meas.freq = 1.0e6f / (meas.p_smp * (float)ts_us_table[cap_ts_idx]);
}

Nếu bắt được ít nhất 2 lần cắt ngang, code sẽ tính khoảng cách trung bình giữa 1 chu kỳ sóng (p_smp). Cuối cùng, áp dụng công thức Tần số = 1 / Chu kỳ. Nó nhân khoảng cách sóng với thang đo thời gian (ts_us_table) để quy đổi ra giây, rồi nghịch đảo lại để ra Tần số (Hz) cực chuẩn.
