#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* TIM1: Phát xung vuông PWM 1 kHz test tại chân PA8 */
void MX_TIM1_Init(void)
{
    TIM_MasterConfigTypeDef m = {0};
    TIM_OC_InitTypeDef      oc = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 71;           /* 72 MHz / 72 = 1 MHz */
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;          /* 1 MHz / 1000 = 1 kHz */
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_RESET;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &m) != HAL_OK) Error_Handler();

    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.Pulse       = 500;                        /* Duty 50% */
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState= TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(&htim1);
}

/* TIM3: Định thời kích chuyển đổi ADC (TRGO UPDATE) */
void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef  ck = {0};
    TIM_MasterConfigTypeDef m  = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 71;           /* 72 MHz / 72 = 1 MHz */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 99;           /* Mặc định lấy mẫu 10 kHz */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

    ck.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &ck) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_UPDATE;     /* Phát xung TRGO khi timer tràn */
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &m) != HAL_OK) Error_Handler();
}

/* TIM4: Đọc Rotary Encoder 2 kênh A/B trên PB6 và PB7 */
void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef e = {0};
    TIM_MasterConfigTypeDef m = {0};

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    e.EncoderMode  = TIM_ENCODERMODE_TI12;
    e.IC1Polarity  = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter    = 3;
    e.IC2Polarity  = TIM_ICPOLARITY_RISING;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC2Filter    = 3;
    if (HAL_TIM_Encoder_Init(&htim4, &e) != HAL_OK) Error_Handler();

    m.MasterOutputTrigger = TIM_TRGO_RESET;
    m.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &m) != HAL_OK) Error_Handler();
}
