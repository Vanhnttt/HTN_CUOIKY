#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;

void HAL_ADC_MspInit(ADC_HandleTypeDef *h)
{
    if (h->Instance != ADC1) return;
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(h, DMA_Handle, hdma_adc1);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *h)
{
    if (h->Instance != SPI1) return;
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM4) return;
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *h)
{
    if (h->Instance != TIM1) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_8;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}
