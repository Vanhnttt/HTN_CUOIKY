#ifndef __SPI_H
#define __SPI_H

#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* Khởi tạo SPI1 Master 9 MHz (SCK=PA5, MOSI=PA7) cho màn hình ST7735 */
void MX_SPI1_Init(void);

#endif /* __SPI_H */
