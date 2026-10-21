#ifndef __ADC_H
#define __ADC_H

#include "main.h"

extern ADC_HandleTypeDef hadc1;

/* Khởi tạo ADC1 lấy mẫu kích bởi TIM3 TRGO */
void MX_ADC1_Init(void);

#endif /* __ADC_H */
