/* ============================================================
 * servo.c: 4x hobby-servo PWM on TIM2 CH1..CH4 (PA0..PA3, AF1).
 * TIM2 pre-configured in main.c: 1 us tick (108 MHz / 108), 20 ms
 * period, PWM1: CCR value == pulse width in microseconds.
 * ============================================================ */
#include "app.h"

#define SERVO_MIN_US   500u
#define SERVO_MAX_US  2500u
#define SERVO_CTR_US  1500u

static const uint32_t s_chan[NUM_SERVO] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};
static uint16_t s_us[NUM_SERVO];         /* shadow of last commanded width */

int servo_init(void)
{
    for (uint8_t i = 0; i < NUM_SERVO; i++) {
        s_us[i] = SERVO_CTR_US;
        __HAL_TIM_SET_COMPARE(&htim2, s_chan[i], SERVO_CTR_US);
        if (HAL_TIM_PWM_Start(&htim2, s_chan[i]) != HAL_OK)
            return -1;
    }
    return 0;
}

void servo_set_us(uint8_t idx, uint16_t us)
{
    if (idx >= NUM_SERVO)
        return;
    if (us < SERVO_MIN_US)
        us = SERVO_MIN_US;
    else if (us > SERVO_MAX_US)
        us = SERVO_MAX_US;
    s_us[idx] = us;
    __HAL_TIM_SET_COMPARE(&htim2, s_chan[idx], us);
}

uint16_t servo_get_us(uint8_t idx)
{
    return (idx < NUM_SERVO) ? s_us[idx] : 0;
}
