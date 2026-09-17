
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "driver/rmt.h"

#define PWM_INPUT_OFFSET 0

void pwm_read_rmt_init(uint8_t channelPins[], uint8_t numberOfPins);

int32_t pwm_read_rmt_dur(uint8_t channel);
void pwm_reset_readings();

#ifdef __cplusplus
}
#endif