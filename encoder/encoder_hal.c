#include "encoder_hal.h"
#include <stdlib.h>

encoder_status_t encoder_hal_detect_event(encoder_handle_t * henc, encoder_event_t * enc_event, encoder_event_t * sw_event)
{
    encoder_mode_t mode = henc->mode;

    *sw_event = encoder_hal_detect_sw(henc);

    switch (mode)
    {
    case ENCODER_MODE_POLL:
        *enc_event = encoder_hal_detect_poll(henc);
        break;
    case ENCODER_MODE_EXTI:
        *enc_event = encoder_hal_detect_exti(henc);
        break;
    case ENCODER_MODE_TIM:
        if (henc->htim == NULL)
            return ENCODER_NULL_CALLBACK;
        
        *enc_event = encoder_hal_detect_tim(henc);
        break;
    default:
        *enc_event = ENCODER_EVENT_NONE;
    }

    return ENCODER_OK;
}

encoder_event_t encoder_hal_detect_poll(encoder_handle_t * henc)
{
    // not available yet
    return ENCODER_EVENT_NONE;
}

encoder_event_t encoder_hal_detect_exti(encoder_handle_t * henc)
{
    // not available yet
    return ENCODER_EVENT_NONE;
}

encoder_event_t encoder_hal_detect_tim(encoder_handle_t * henc)
{    
    uint32_t ticks = __HAL_TIM_GET_COUNTER(henc->htim);
    uint32_t t = HAL_GetTick();

    encoder_event_t event = ENCODER_EVENT_NONE;

    // check for rotation
    // determine exactly how many ticks spinning the encoder gives
    // add support for different encoders ( 2 PPT, 4PPT, etc)
    if (t - henc->_state.last_update > henc->t_update)
    {
        int32_t delta = ticks - henc->_state.last_ticks;

        // fix this for when encoder is halfway turned between "full" turns
        if (abs(delta) > 1)
        {
            // determine direction
            if (ticks > henc->_state.last_ticks)
            {
                // add code for computing the number of spins it made to determine how many pages to move?
                henc->_state.direction = ENCODER_DIR_CW;
                event = ENCODER_EVENT_CW_TURN;
            }
            else
            {
                henc->_state.direction = ENCODER_DIR_CCW;
                event = ENCODER_EVENT_CCW_TURN;
            }

            // velocity notation - CW +, CCW -
            henc->_state.velocity = (float)delta / (t - henc->_state.last_update);
            henc->_state.last_update = t;
            henc->_state.last_ticks = ticks;
        }
    }

    return event;
}

encoder_event_t encoder_hal_detect_sw(encoder_handle_t * henc)
{
    encoder_event_t sw_event = ENCODER_EVENT_NONE;

    encoder_sw_state_t sw;
    
    // enable pull-ups
    if (HAL_GPIO_ReadPin(henc->sw_gpio, henc->sw_pin) == GPIO_PIN_SET)
    {
        sw = ENCODER_SW_RELEASE;
    }
    else
    {
        sw = ENCODER_SW_PRESS;
    }

    // if switch state has changed...
    if (sw != henc->_state.last_sw)
    {
        if (sw == ENCODER_SW_PRESS)
        {
            sw_event = ENCODER_EVENT_SW_PRESS;
        }
        else
        {
            sw_event = ENCODER_EVENT_SW_RELEASE;
        }
    }

    henc->_state.last_sw = sw;

    return sw_event;
}
