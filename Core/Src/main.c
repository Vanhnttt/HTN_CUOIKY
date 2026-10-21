/* ==============================================================================
 * DỰ ÁN MÁY HIỆN SÓNG MINI (MINI OSCILLOSCOPE STM32F103C8T6 + TFT ST7735 1.8")
 * Đề tài: V07
 * ============================================================================== */

#include "main.h"

/* 1. Các module ngoại vi phần cứng */
#include "sys_clock.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"

/* 2. Các module chức năng của máy hiện sóng */
#include "test_signal.h"        /* Bộ phát xung test 1 kHz (PA8 TIM1) */
#include "scope_acquisition.h"  /* Khối thu thập dữ liệu (ADC1 + TIM3 TRGO + DMA) */
#include "signal_calc.h"        /* KHỐI TÍNH TOÁN: Trigger, Vmin/Vmax/Vpp, Tần số */
#include "st7735.h"             /* Driver màn hình màu TFT ST7735 */
#include "scope_gui.h"          /* Khối hiển thị: Lưới, dạng sóng, thông số OSD */
#include "scope_input.h"        /* Khối điều khiển: Encoder TIM4 + Nút PB8/PB9 */
#include "scope_fft.h"          /* Khối phân tích phổ FFT Radix-2 */

/* Bộ đệm nhận dữ liệu snapshot từ DMA */
static uint16_t snap[SNAPSHOT_LEN];
static uint8_t  snap_tb = 5;

int main(void)
{
    /* --------------------------------------------------------------------------
     * 1. KHỞI TẠO TẤT CẢ NGOẠI VI PHẦN CỨNG (Tách riêng từng module .c/.h)
     * -------------------------------------------------------------------------- */
    HAL_Init();
    SystemClock_Config();       /* Khởi tạo xung nhịp 72 MHz (sys_clock.c) */
    MX_GPIO_Init();             /* Khởi tạo GPIO (gpio.c) */
    MX_DMA_Init();              /* Khởi tạo DMA1 Channel 1 (dma.c) */
    MX_ADC1_Init();             /* Khởi tạo ADC1 chuyển đổi kích bởi TIM3 (adc.c) */
    MX_SPI1_Init();             /* Khởi tạo SPI1 Master 9MHz cho LCD (spi.c) */
    MX_TIM1_Init();             /* Khởi tạo TIM1 phát PWM test 1kHz (tim.c) */
    MX_TIM3_Init();             /* Khởi tạo TIM3 TRGO kích lấy mẫu ADC (tim.c) */
    MX_TIM4_Init();             /* Khởi tạo TIM4 Encoder Mode (tim.c) */

    /* --------------------------------------------------------------------------
     * 2. KHỞI TẠO CÁC MODULE ỨNG DỤNG MÁY HIỆN SÓNG
     * -------------------------------------------------------------------------- */
    TestSignal_Start();         /* Bắt đầu phát xung vuông 1 kHz test tại chân PA8 */
    Scope_Input_Init();         /* Bắt đầu đọc xung Encoder TIM4 */
    Scope_FFT_Init();           /* Tính trước cửa sổ Hanning và hệ số Twiddle FFT */
    Scope_GUI_Init();           /* Xóa bộ đệm đồ họa hiển thị */

    ST7735_Init();              /* Khởi tạo màn hình ST7735 qua SPI1 */
    ST7735_Clear(C_BLACK);      /* Xóa màn hình về màu nền đen */

    Scope_Acquisition_Init();   /* Hiệu chuẩn ADC1 */
    Scope_Acquisition_SetTimebase(tb_idx); /* Thiết lập tốc độ lấy mẫu ban đầu */
    Scope_Acquisition_Start();  /* Bắt đầu thu thập dữ liệu ADC qua DMA Circular */

    /* --------------------------------------------------------------------------
     * 3. VÒNG LẶP CHÍNH (SUPER-LOOP)
     * -------------------------------------------------------------------------- */
    while (1)
    {
        /* Đọc núm xoay Encoder và các nút bấm PB8, PB9 */
        Scope_Input_Handle();

        /* Lấy 1 snapshot 256 mẫu mới nhất từ bộ đệm kép DMA (nếu không ở chế độ HOLD) */
        uint8_t fresh = (!hold) ? Scope_Acquisition_GrabSnapshot(snap, tb_idx, &snap_tb) : 0;

        /* Khi có mẫu mới hoặc khi người dùng vừa xoay núm / bấm nút */
        if (fresh || need_redraw)
        {
            need_redraw = 0;

            /* KHỐI TÍNH TOÁN: Tính Vmin, Vmax, Vpp và Tần số bằng đếm cắt ngưỡng */
            SigCalc_XuLyTatCa(snap, snap_tb);

            if (mode_fft)
            {
                /* Tính toán phổ tần số FFT Radix-2 và tìm đỉnh tần số */
                Scope_FFT_Compute(snap, Scope_Acquisition_GetSamplingFreq(snap_tb));
            }
            else
            {
                /* KHỐI TÍNH TOÁN: Tìm điểm Trigger và dựng tọa độ vẽ sóng */
                SigCalc_ChuanBiDangSong(snap, trig_lvl, center_mv, vd_idx);
            }

            /* Cập nhật thông số OSD và hộp chọn điều chỉnh */
            Scope_GUI_BuildOSD(snap_tb, vd_idx, trig_lvl, center_mv, adj_mode, mode_fft, hold);
            Scope_GUI_UpdateHighlight(adj_mode);

            /* Render hình ảnh (lưới, dạng sóng, dấu trigger, đất 0V) ra màn hình ST7735 */
            Scope_GUI_Render(trig_lvl, center_mv, vd_idx, adj_mode, mode_fft, hold);
        }
    }
}
