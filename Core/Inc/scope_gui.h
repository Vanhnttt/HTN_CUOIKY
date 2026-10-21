#ifndef __SCOPE_GUI_H
#define __SCOPE_GUI_H

#include "main.h"
#include "st7735.h"
#include "signal_calc.h"
#include "scope_fft.h"

#define STRIP_W      16
#define STRIP_PIXELS (LCD_H * STRIP_W)

void Scope_GUI_Init(void);
void Scope_GUI_BuildOSD(uint8_t snap_tb, uint8_t vd_idx, uint16_t trig_lvl, int16_t center_mv, uint8_t adj_mode, uint8_t mode_fft, uint8_t hold);
void Scope_GUI_UpdateHighlight(uint8_t adj_mode);
void Scope_GUI_Render(uint16_t trig_lvl, int16_t center_mv, uint8_t vd_idx, uint8_t adj_mode, uint8_t mode_fft, uint8_t hold);

#endif /* __SCOPE_GUI_H */
