#ifndef __SCOPE_FFT_H
#define __SCOPE_FFT_H

#include <stdint.h>
#include "st7735.h"

#define FFT_N        256
#define FFT_FLOOR    1024.0f

extern uint8_t  fft_mag[LCD_W];
extern uint32_t fft_peak_hz;

void Scope_FFT_Init(void);
void Scope_FFT_Compute(const uint16_t *snap_data, uint32_t fs_hz);

#endif /* __SCOPE_FFT_H */
