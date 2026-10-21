#ifndef __SCOPE_ACQUISITION_H
#define __SCOPE_ACQUISITION_H

#include "main.h"

#define ADC_BUF_LEN      512
#define SNAPSHOT_LEN     256
#define TB_COUNT         10

/* Bảng giá trị Auto-Reload Register của TIM3 tương ứng với các mức Time/Div */
extern const uint16_t tb_arr[TB_COUNT];

void     Scope_Acquisition_Init(void);
void     Scope_Acquisition_Start(void);
void     Scope_Acquisition_SetTimebase(uint8_t tb_idx);
uint32_t Scope_Acquisition_GetSamplingFreq(uint8_t tb_idx);
uint8_t  Scope_Acquisition_GrabSnapshot(uint16_t *dest, uint8_t current_tb, uint8_t *snapshot_tb);

/* Callbacks DMA Half/Full Transfer */
void Scope_Acquisition_DMA_HalfCallback(void);
void Scope_Acquisition_DMA_CpltCallback(void);

#endif /* __SCOPE_ACQUISITION_H */
