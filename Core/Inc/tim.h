#ifndef __TIM_H
#define __TIM_H

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Khởi tạo TIM1 (PWM phát xung test 1kHz), TIM3 (kích ADC), TIM4 (Encoder) */
void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

#endif /* __TIM_H */
