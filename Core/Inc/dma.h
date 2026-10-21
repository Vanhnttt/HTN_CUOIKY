#ifndef __DMA_H
#define __DMA_H

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;

/* Khởi tạo DMA1 Channel 1 cho ADC1 */
void MX_DMA_Init(void);

#endif /* __DMA_H */
