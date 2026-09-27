#include "encoder.h"
#include "encoder_hal.h"
#include <limits.h>

encoder_status_t encoder_init(encoder_handle_t * henc)
{
    // set defaults for private members
    henc->_state.last_sw = ENCODER_SW_RELEASE;
    henc->_state.position = 0;
    henc->_state.velocity = 0;
    henc->_state.direction = ENCODER_DIR_NONE;
    henc->_state.last_ticks = 0;
    henc->_state.last_update = 0;
    henc->_pos_mode = ENCODER_POS_CLAMP;
    henc->_position_max = LONG_MAX;
    henc->_position_min = LONG_MIN;

    return ENCODER_OK;
}

encoder_status_t encoder_constrain(encoder_handle_t * henc, int32_t min, int32_t max)
{
    if (max <= min)
        return ENCODER_INVALID_CFG;

    // set maximum and minimum positions
    henc->_position_max = max;
    henc->_position_min = min;

    return ENCODER_OK;
}

encoder_status_t encoder_set_pos_mode(encoder_handle_t * henc, encoder_pos_mode_t pos_mode)
{
    if (pos_mode != ENCODER_POS_CLAMP && pos_mode != ENCODER_POS_WRAP)
        return ENCODER_INVALID_CFG;

    henc->_pos_mode = pos_mode;

    return ENCODER_OK;
}

encoder_status_t encoder_update(encoder_handle_t * henc)
{
    encoder_event_t enc_event, sw_event;

    encoder_status_t status = encoder_hal_detect_event(henc, &enc_event, &sw_event);
    
    if (status != ENCODER_OK)
        return status;

    switch (enc_event)
    {
    case ENCODER_EVENT_NONE:
        break;

    case ENCODER_EVENT_CW_TURN:

        if (henc->_pos_mode == ENCODER_POS_CLAMP)
        {
            if (henc->_state.position < henc->_position_max)
                (henc->_state.position)++;
            /*
            int32_t new_pos = henc->_state.position + henc->_state.delta_ticks / 2; // configure for different encoders

            if (new_pos > henc->_position_max)
                henc->_state.position = henc->_position_max;
            else
                henc->_state.position = new_pos;
            */
        }
        else if (henc->_pos_mode == ENCODER_POS_WRAP)
        {
            if (++(henc->_state.position) > henc->_position_max)
                henc->_state.position = henc->_position_min;
            /*
            uint32_t rotations = henc->_state.delta_ticks / 2;
            uint32_t range = henc->_position_max - henc->_position_min + 1;
            int32_t new_pos = henc->_state.position + rotations;

            if (new_pos > henc->_position_max)
                henc->_state.position = ((henc->_state.position - henc->_position_min + rotations) % range + range) % range + henc->_position_min;
            else
                henc->_state.position = new_pos;
            */
        }
        break;

    case ENCODER_EVENT_CCW_TURN:
        if (henc->_pos_mode == ENCODER_POS_CLAMP)
        {
            if (henc->_state.position > henc->_position_min)
                (henc->_state.position)--;
            /*
            int32_t new_pos = henc->_state.position - henc->_state.delta_ticks / 2; // configure for different encoders

            if (new_pos < henc->_position_min)
                henc->_state.position = henc->_position_min;
            else
                henc->_state.position = new_pos;
            */
        }
        else if (henc->_pos_mode == ENCODER_POS_WRAP)
        {
            if (--(henc->_state.position) < henc->_position_min)
                henc->_state.position = henc->_position_max;
            /*
            uint32_t rotations = henc->_state.delta_ticks / 2;
            uint32_t range = henc->_position_max - henc->_position_min + 1;
            int32_t new_pos = henc->_state.position + rotations;

            if (new_pos < henc->_position_min)
                henc->_state.position = ((henc->_state.position - henc->_position_min + rotations) % range + range) % range + henc->_position_min;
            else
                henc->_state.position = new_pos;
            */
        }
        break;
    default:
    }

    if (henc->_callback == NULL)
        return ENCODER_NULL_CALLBACK;
    
    henc->_callback(henc, enc_event, sw_event);

    return ENCODER_OK;
}

encoder_status_t encoder_register_callback(encoder_handle_t * henc, encoder_callback_t callback)
{
    if (callback == NULL)
        return ENCODER_NULL_CALLBACK;

    henc->_callback = callback;
    return ENCODER_OK;
}

int32_t encoder_get_position(encoder_handle_t * henc)
{
    return henc->_state.position;
}

float encoder_get_velocity(encoder_handle_t * henc)
{
    return henc->_state.velocity;
}