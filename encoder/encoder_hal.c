#include "encoder_hal.h"
#include <stdlib.h>

void encoder_detect_event(encoder_handle_t * henc, encoder_event_t * enc_event, encoder_event_t * sw_event)
{
    encoder_mode_t mode = henc->mode;

    *sw_event = encoder_detect_sw(henc);

    switch (mode)
    {
    case ENCODER_MODE_POLL:
        *enc_event = encoder_detect_poll(henc);
        break;
    case ENCODER_MODE_EXTI:
        *enc_event = encoder_detect_exti(henc);
        break;
    case ENCODER_MODE_TIM:
        *enc_event = encoder_detect_tim(henc);
        break;
    default:
        *enc_event = ENCODER_EVENT_NONE;
    }
}

encoder_event_t encoder_detect_poll(encoder_handle_t * henc)
{
    // not available yet
    return ENCODER_EVENT_NONE;
}

encoder_event_t encoder_detect_exti(encoder_handle_t * henc)
{
    // not available yet
    return ENCODER_EVENT_NONE;
}

encoder_event_t encoder_detect_tim(encoder_handle_t * henc)
{
    henc->state.ticks = __HAL_TIM_GET_COUNTER(henc->htim);

    uint32_t t = HAL_GetTick();
    encoder_event_t event = ENCODER_EVENT_NONE;

    // check for rotation
    // determine exactly how many ticks spinning the encoder gives
    // add support for different encoders ( 2 PPT, 4PPT, etc)
    if (t - henc->state.prev_update > henc->t_update)
    {
        henc->state.delta_ticks = henc->state.ticks - henc->state.prev_ticks;

        // if (abs(henc->state.delta_ticks))
        if (abs(henc->state.delta_ticks) > 1)
        {
            // determine direction
            if (henc->state.ticks > henc->state.prev_ticks)
            {
                // add code for computing the number of spins it made to determine how many pages to move?
                henc->state.direction = ENCODER_DIR_CW;
                event = ENCODER_EVENT_CW_TURN;
            }
            else
            {
                henc->state.direction = ENCODER_DIR_CCW;
                event = ENCODER_EVENT_CCW_TURN;
            }

            // velocity notation - CW +, CCW -
            henc->state.velocity = (float)henc->state.delta_ticks / (t - henc->state.prev_update);
            henc->state.prev_update = t;
            henc->state.prev_ticks = henc->state.ticks;
        }
    }

    return event;
}

encoder_event_t encoder_detect_sw(encoder_handle_t * henc)
{
    encoder_event_t sw_event = ENCODER_SW_NO_CHANGE;

    // enable pull-ups
    // this sucks
    if (HAL_GPIO_ReadPin(henc->sw_gpio, henc->sw_pin) == GPIO_PIN_SET)
    {
        henc->state.sw = ENCODER_SW_RELEASE;
    }
    else
    {
        henc->state.sw = ENCODER_SW_PRESS;
    }

    if (henc->state.sw != henc->state.prev_sw)
    {
        if (henc->state.sw == ENCODER_SW_PRESS)
        {
            sw_event = ENCODER_EVENT_SW_PRESS;
        }
        else
        {
            sw_event = ENCODER_EVENT_SW_RELEASE;
        }
    }

    henc->state.prev_sw = henc->state.sw;

    return sw_event;
}
