#include "bsp_system.h"

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* ST7735 Control Pins: DC=PB0, RST=PB1, CS=PB10 - start HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10, GPIO_PIN_SET);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* Encoder Switch PB8 */
    g.Pin = GPIO_PIN_8;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);

    /* HOLD/RUN Button PB9 */
    g.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    /* PA0 Analog Input (ADC1_IN0) */
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

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
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;  /* 72MHz / 6 = 12 MHz cho ADC */
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

void BSP_System_Init(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
