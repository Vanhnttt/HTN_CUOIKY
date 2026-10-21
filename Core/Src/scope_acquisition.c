#include "scope_acquisition.h"
#include "adc.h"
#include "tim.h"
#include "dma.h"

/* ARR tương ứng: 500k, 200k, 100k, 50k, 20k, 10k, 5k, 2k, 1k, 500Hz */
const uint16_t tb_arr[TB_COUNT] = { 1, 4, 9, 19, 49, 99, 199, 499, 999, 1999 };

static volatile uint16_t adc_buf[ADC_BUF_LEN];
static volatile uint8_t  dma_half = 0;
static volatile uint32_t dma_seq  = 0;
static uint32_t          used_seq = 0;

void Scope_Acquisition_DMA_HalfCallback(void)
{
    dma_half = 0;
    dma_seq++;
}

void Scope_Acquisition_DMA_CpltCallback(void)
{
    dma_half = 1;
    dma_seq++;
}

void Scope_Acquisition_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}

void Scope_Acquisition_Start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
    HAL_TIM_Base_Start(&htim3);
}

void Scope_Acquisition_SetTimebase(uint8_t tb_idx)
{
    if (tb_idx >= TB_COUNT) tb_idx = TB_COUNT - 1;
    HAL_ADC_Stop_DMA(&hadc1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, tb_arr[tb_idx]);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    used_seq = dma_seq;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_BUF_LEN);
}

uint32_t Scope_Acquisition_GetSamplingFreq(uint8_t tb_idx)
{
    return 1000000u / ((uint32_t)tb_arr[tb_idx] + 1u);
}

uint8_t Scope_Acquisition_GrabSnapshot(uint16_t *dest, uint8_t current_tb, uint8_t *snapshot_tb)
{
    uint32_t s1 = dma_seq;
    if (s1 == used_seq) return 0;

    const volatile uint16_t *src = &adc_buf[dma_half ? SNAPSHOT_LEN : 0];
    for (int i = 0; i < SNAPSHOT_LEN; i++)
    {
        dest[i] = src[i];
    }

    if (dma_seq != s1) return 0;
    used_seq = s1;
    *snapshot_tb = current_tb;
    return 1;
}
