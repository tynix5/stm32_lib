#include "encoder.h"
#include "encoder_hal.h"
#include <limits.h>

status_t encoder_init(encoder_handle_t * henc)
{
    // set defaults for these
    henc->state.sw = henc->state.prev_sw = ENCODER_SW_RELEASE;
    henc->state.position = henc->state.prev_position = 0;
    henc->state.velocity = 0;
    henc->state.direction = ENCODER_DIR_NONE;
    henc->state.ticks = henc->state.prev_ticks = ENCODER_SW_RELEASE;
    henc->state.prev_update = 0;
    henc->pos_mode = ENCODER_POS_CLAMP;
    henc->position_max = LONG_MAX;
    henc->position_min = LONG_MIN;

    return STATUS_OK;
}

status_t encoder_constrain(encoder_handle_t * henc, int32_t min, int32_t max)
{
    if (max <= min)
        return STATUS_INVALID_CFG;

    henc->position_max = max;
    henc->position_min = min;

    return STATUS_OK;
}

status_t encoder_set_pos_mode(encoder_handle_t * henc, encoder_pos_mode_t pos_mode)
{
    if (pos_mode != ENCODER_POS_CLAMP && pos_mode != ENCODER_POS_WRAP)
        return STATUS_INVALID_CFG;

    henc->pos_mode = pos_mode;

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

        if (henc->pos_mode == ENCODER_POS_CLAMP)
        {
            if (henc->state.position < henc->position_max)
                (henc->state.position)++;
            /*
            int32_t new_pos = henc->state.position + henc->state.delta_ticks / 2; // configure for different encoders

            if (new_pos > henc->position_max)
                henc->state.position = henc->position_max;
            else
                henc->state.position = new_pos;
            */
        }
        else if (henc->pos_mode == ENCODER_POS_WRAP)
        {
            if (++(henc->state.position) > henc->position_max)
                henc->state.position = henc->position_min;
            /*
            uint32_t rotations = henc->state.delta_ticks / 2;
            uint32_t range = henc->position_max - henc->position_min + 1;
            int32_t new_pos = henc->state.position + rotations;

            if (new_pos > henc->position_max)
                henc->state.position = ((henc->state.position - henc->position_min + rotations) % range + range) % range + henc->position_min;
            else
                henc->state.position = new_pos;
            */
        }
        // henc->callback(henc, enc_event);
        break;

    case ENCODER_EVENT_CCW_TURN:
        if (henc->pos_mode == ENCODER_POS_CLAMP)
        {
            if (henc->state.position > henc->position_min)
                (henc->state.position)--;
            /*
            int32_t new_pos = henc->state.position - henc->state.delta_ticks / 2; // configure for different encoders

            if (new_pos < henc->position_min)
                henc->state.position = henc->position_min;
            else
                henc->state.position = new_pos;
            */
        }
        else if (henc->pos_mode == ENCODER_POS_WRAP)
        {
            if (--(henc->state.position) < henc->position_min)
                henc->state.position = henc->position_max;
            /*
            uint32_t rotations = henc->state.delta_ticks / 2;
            uint32_t range = henc->position_max - henc->position_min + 1;
            int32_t new_pos = henc->state.position + rotations;

            if (new_pos < henc->position_min)
                henc->state.position = ((henc->state.position - henc->position_min + rotations) % range + range) % range + henc->position_min;
            else
                henc->state.position = new_pos;
            */
        }
        // henc->callback(henc, enc_event);
        break;
    default:
    }

    switch (sw_event)
    {
    case ENCODER_EVENT_NONE:
        break;
    case ENCODER_EVENT_CW_TURN:
        break;
    case ENCODER_EVENT_CCW_TURN:
        break;
    default:
    }

    henc->callback(henc, enc_event);
    henc->sw_callback(henc, sw_event);

    return STATUS_OK;
}

status_t encoder_register_callback(encoder_handle_t * henc, encoder_callback_t callback)
{
    henc->callback = callback;

    return STATUS_OK;
}

status_t encoder_register_sw_callback(encoder_handle_t * henc, encoder_callback_t callback)
{
    henc->sw_callback = callback;

    return STATUS_OK;
}

int32_t encoder_get_position(encoder_handle_t * henc)
{
    return henc->state.position;
}