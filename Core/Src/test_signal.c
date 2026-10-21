#include "test_signal.h"
#include "tim.h"

void TestSignal_Init(void)
{
    MX_TIM1_Init();
}

void TestSignal_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void TestSignal_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}
