# DỰ ÁN MÁY HIỆN SÓNG MINI (STM32F103C8T6 + TFT ST7735 1.8")
---
## 📂 CẤU TRÚC DỰ ÁN ĐÃ TÁCH MODULE HOÀN CHỈNH
```text
oscilloscope/
├── .vscode/
│   └── c_cpp_properties.json   (Cấu hình Include & IntelliSense cho VS Code)
├── Core/
│   ├── Inc/
│   │   │   /* --- 1. KHỐI TÍNH TOÁN & XỬ LÝ TÍN HIỆU (THEO ĐỀ BÀI) --- */
│   │   ├── signal_calc.h       (Tìm trigger, tính Vmin/Vmax/Vpp, tần số cắt ngưỡng)
│   │   │
│   │   │   /* --- 2. CÁC MODULE ỨNG DỤNG MÁY HIỆN SÓNG --- */
│   │   ├── scope_acquisition.h (Thu thập ADC1 kích TIM3 TRGO + DMA Double buffer)
│   │   ├── test_signal.h       (Phát xung vuông PWM 1 kHz test sóng tại PA8)
│   │   ├── st7735.h            (Driver màn hình màu TFT ST7735 qua SPI1)
│   │   ├── scope_gui.h         (Hiển thị lưới chia ô, dạng sóng, OSD thông số)
│   │   ├── scope_input.h       (Đọc Rotary Encoder TIM4 & nút bấm PB8, PB9)
│   │   ├── scope_fft.h         (Chế độ FFT hiển thị phổ tần số Radix-2)
│   │   │
│   │   │   /* --- 3. CÁC MODULE NGOẠI VI PHẦN CỨNG (TÁCH KHỎI MAIN) --- */
│   │   ├── sys_clock.h         (Cấu hình xung nhịp 72 MHz: HSE + PLL)
│   │   ├── gpio.h              (Cấu hình chân I/O màn hình, nút bấm, ADC)
│   │   ├── dma.h               (Cấu hình DMA1 Channel 1 cho ADC1)
│   │   ├── adc.h               (Cấu hình ADC1 kích hoạt bởi TIM3 TRGO)
│   │   ├── spi.h               (Cấu hình SPI1 Master 9 MHz)
│   │   ├── tim.h               (Cấu hình TIM1 PWM, TIM3 ADC TRGO, TIM4 Encoder)
│   │   ├── stm32f1xx_it.h      (Khai báo các ngắt hệ thống & ngoại vi)
│   │   └── main.h              (Khai báo thư viện HAL & Error_Handler)
│   │
│   └── Src/
│       ├── main.c              (Hàm main điều phối luồng chạy - RẤT NGẮN GỌN)
│       ├── signal_calc.c       (Thuật toán tính toán Vpp, Freq, Trigger)
│       ├── scope_acquisition.c
│       ├── test_signal.c
│       ├── st7735.c
│       ├── scope_gui.c
│       ├── scope_input.c
│       ├── scope_fft.c
│       ├── sys_clock.c
│       ├── gpio.c
│       ├── dma.c
│       ├── adc.c
│       ├── spi.c
│       ├── tim.c
│       ├── stm32f1xx_it.c      (Hàm phục vụ ngắt DMA, ngắt hệ thống)
│       └── stm32f1xx_hal_msp.c (Hàm MSP cấu hình chân & xung ngoại vi)
└── README.md
```

---

## 📌 CHI TIẾT CÁC HÀM TRONG KHỐI TÍNH TOÁN (`signal_calc.c`)

Khối này giải quyết trọn vẹn yêu cầu trong ảnh:
> *"Xử lý: tìm điểm trigger, tính Vmin/Vmax/Vpp, tần số (đếm cắt ngưỡng)"*

1. **`SigCalc_Tinh_Vmin_Vmax_Vpp(snap_buf)`**:
   - Quét 256 mẫu ADC để tìm mẫu cực tiểu và cực đại.
   - Tính toán điện áp chuẩn: $V_{\min}$, $V_{\max}$ và điện áp đỉnh - đỉnh $V_{\text{pp}} = V_{\max} - V_{\min}$.
2. **`SigCalc_TinhTanSo_DemCatNguong(snap_buf, snap_tb)`**:
   - Xác định ngưỡng trung bình `mid = (max + min)/2` và dải trễ `hyst`.
   - Đếm số lần tín hiệu cắt qua ngưỡng cạnh lên để tính chính xác chu kỳ và tần số tín hiệu $f = \frac{f_s \times (edges - 1)}{\Delta samples}$.
3. **`SigCalc_TimDiemTrigger(snap_buf, trig_lvl)`**:
   - Dò tìm điểm tín hiệu vượt ngưỡng `trig_lvl` theo cạnh lên có dải trễ chống nhiễu (hysteresis) để khóa dạng sóng đứng yên trên màn hình.
4. **`SigCalc_ChuanBiDangSong(...)`**:
   - Ánh xạ các điểm mẫu quanh điểm trigger sang tọa độ cột $X$ và hàng $Y$ trên màn hình ST7735.
