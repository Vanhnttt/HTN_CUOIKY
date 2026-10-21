#ifndef __TEST_SIGNAL_H
#define __TEST_SIGNAL_H

#include "main.h"

/* Khởi tạo và phát xung PWM 1 kHz (Duty cycle 50%) tại chân PA8 */
void TestSignal_Init(void);
void TestSignal_Start(void);
void TestSignal_Stop(void);

#endif /* __TEST_SIGNAL_H */
