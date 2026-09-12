#include "encoder.h"
#include "encoder_hal.h"
#include <limits.h>

status_t encoder_init(encoder_handle_t * henc)
{
    // set defaults for private members
    henc->_state.sw = henc->_state.prev_sw = ENCODER_SW_RELEASE;
    henc->_state.position = henc->_state.prev_position = 0;
    henc->_state.velocity = 0;
    henc->_state.direction = ENCODER_DIR_NONE;
    henc->_state.ticks = henc->_state.prev_ticks = ENCODER_SW_RELEASE;
    henc->_state.prev_update = 0;
    henc->_pos_mode = ENCODER_POS_CLAMP;
    henc->_position_max = LONG_MAX;
    henc->_position_min = LONG_MIN;

    return STATUS_OK;
}

status_t encoder_constrain(encoder_handle_t * henc, int32_t min, int32_t max)
{
    if (max <= min)
        return STATUS_INVALID;

    henc->_position_max = max;
    henc->_position_min = min;

    return STATUS_OK;
}

status_t encoder_set_pos_mode(encoder_handle_t * henc, encoder_pos_mode_t pos_mode)
{
    if (pos_mode != ENCODER_POS_CLAMP && pos_mode != ENCODER_POS_WRAP)
        return STATUS_INVALID;

    henc->_pos_mode = pos_mode;

    return STATUS_OK;
}

status_t encoder_update(encoder_handle_t * henc)
{
    encoder_event_t enc_event, sw_event;
    encoder_detect_event(henc, &enc_event, &sw_event);

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
        // henc->_callback(henc, enc_event);
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
        // henc->_callback(henc, enc_event);
        break;
    default:
    }

    switch (sw_event)
    {
    case ENCODER_EVENT_NONE:
    case ENCODER_EVENT_CW_TURN:
    case ENCODER_EVENT_CCW_TURN:
    default:
    }

    henc->_callback(henc, enc_event, sw_event);

    return STATUS_OK;
}

status_t encoder_register_callback(encoder_handle_t * henc, encoder_callback_t callback)
{
    henc->_callback = callback;
    return STATUS_OK;
}

int32_t encoder_get_position(encoder_handle_t * henc)
{
    return henc->_state.position;
}