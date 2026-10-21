#ifndef __SCOPE_INPUT_H
#define __SCOPE_INPUT_H

#include "main.h"

#define ENC_DIR 1

typedef enum {
    MODE_TIME = 0,
    MODE_VOLT,
    MODE_TRIG,
    MODE_POS,
    MODE_COUNT
} AdjustMode_t;

extern const uint16_t mode_col[MODE_COUNT];

/* Biến trạng thái điều khiển */
extern uint8_t      tb_idx;
extern uint8_t      vd_idx;
extern int16_t      center_mv;
extern uint16_t     trig_lvl;
extern AdjustMode_t adj_mode;
extern uint8_t      hold;
extern uint8_t      mode_fft;
extern uint8_t      need_redraw;

void Scope_Input_Init(void);
void Scope_Input_PollButtons(void);
void Scope_Input_Handle(void);

#endif /* __SCOPE_INPUT_H */
