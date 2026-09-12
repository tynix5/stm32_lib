#include "bms.h"

static float map(float x, float in_min, float in_max, float out_min, float out_max);

status_t bms_init(bms_handle_t * bms, bms_callback_t callback)
{
    if (callback == NULL)
        return STATUS_NULL;

    bms->_callback = callback;
    return STATUS_OK;
}
status_t bms_read_voltage(bms_handle_t * bms, float norm_adc, float * v)
{

    switch (bms->type)
    {
    case BMS_LIPO_TYPE:
        *v = bms->div * norm_adc * bms->v_ref;
        break;
    default:
        *v = -1.0;
        return STATUS_INVALID;
    }

    return STATUS_OK;
}
status_t bms_read_percent(bms_handle_t * bms, float norm_adc, float * percent)
{
    float v = 0;
    *percent = 0;

    if (bms_read_voltage(bms, norm_adc, &v) != STATUS_OK)
        return STATUS_ERR;

    switch (bms->type)
    {
    case BMS_LIPO_TYPE:
        const float lipo_max = LIPO_CELL_MAX_V * bms->cell_cnt;
        const float lipo_min = LIPO_CELL_MIN_V * bms->cell_cnt;
        *percent = map(v, lipo_min, lipo_max, 0.0, 100.0);
        break;
    default:
        return STATUS_ERR;
    }

    if (*percent > 100.0)
        *percent = 100.0;
    else if (*percent < 0.0)
        *percent = 0.0;

    return STATUS_OK;
}

status_t bms_update(bms_handle_t * bms, float norm_adc, float * percent)
{
    if (bms_read_percent(bms, norm_adc, percent) != STATUS_OK)
        return STATUS_ERR;

    if (bms->_callback == NULL)
        return STATUS_NULL;

    if (*percent == 100.0)
    {
        bms->_callback(bms, BMS_EVENT_CHARGE_FULL);
    }
    else if (*percent == 0.0)
    {
        bms->_callback(bms, BMS_EVENT_CHARGE_EMPTY);
    }

    return STATUS_OK;
}

static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    // prevent division by 0
    if (in_max - in_min == 0)
    {
        return 0;
    }

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}