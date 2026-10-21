#include "sys_clock.h"

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       o = {0};
    RCC_ClkInitTypeDef       c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};

    o.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    o.HSEState            = RCC_HSE_ON;
    o.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    o.HSIState            = RCC_HSI_ON;
    o.PLL.PLLState        = RCC_PLL_ON;
    o.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL          = RCC_PLL_MUL9;   /* 8 MHz * 9 = 72 MHz */
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                           | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider       = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider      = RCC_HCLK_DIV2;
    c.APB2CLKDivider      = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;  /* 72 MHz / 6 = 12 MHz cho ADC */
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}
