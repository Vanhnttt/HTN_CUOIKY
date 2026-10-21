#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Màn hình ST7735: DC=PB0, RST=PB1, CS=PB10 - mức cao ban đầu */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10, GPIO_PIN_SET);
    g.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* Nút ấn Encoder PB8 */
    g.Pin  = GPIO_PIN_8;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);

    /* Nút ấn HOLD/RUN PB9 */
    g.Pin  = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &g);

    /* Ngõ vào tương tự ADC PA0 */
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}
