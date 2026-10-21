#ifndef __SYS_CLOCK_H
#define __SYS_CLOCK_H

#include "main.h"

/* Cấu hình thạch anh ngoài HSE 8MHz + PLL 9 = 72 MHz, PCLK2/6 = 12 MHz cho ADC */
void SystemClock_Config(void);

#endif /* __SYS_CLOCK_H */
