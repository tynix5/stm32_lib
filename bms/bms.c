#include "bms.h"

/**************************************** Helper Functions *******************************************/
/* Linear conversion of numbers from one range to another range */
/* [in] x - input in in_range */
/* [in] in_min - minimum input */
/* [in] in_max - maximum input */
/* [in] out_min - minimum output */
/* [in] out_max - maximum output */
/* [return] converted number */
static float map(float x, float in_min, float in_max, float out_min, float out_max);
/*****************************************************************************************************/

bms_status_t bms_init(bms_handle_t * bms, bms_callback_t callback)
{
    if (callback == NULL)
        return BMS_NULL_CALLBACK;

    bms->_callback = callback;
    return BMS_OK;
}

bms_status_t bms_read_voltage(const bms_handle_t * bms, float norm_adc, float * v)
{

    switch (bms->type)
    {
    case BMS_LIPO_TYPE:
        *v = bms->div * norm_adc * bms->v_ref;
        break;
    default:
        *v = -1.0;
        return BMS_INVALID_CFG;
    }

    *v += bms->v_offset;        // add measured voltage offset from ADC

    return BMS_OK;
}

bms_status_t bms_read_percent(const bms_handle_t * bms, float norm_adc, float * percent)
{
    float v = 0;
    *percent = 0;

    bms_status_t status = bms_read_voltage(bms, norm_adc, &v);
    if (status != BMS_OK)
        return status;

    switch (bms->type)
    {
    case BMS_LIPO_TYPE:
        const float lipo_max = LIPO_CELL_MAX_V * bms->cell_cnt;
        const float lipo_min = LIPO_CELL_MIN_V * bms->cell_cnt;
        *percent = map(v, lipo_min, lipo_max, 0.0, 100.0);
        break;
    default:
        return BMS_INVALID_CFG;
    }

    if (*percent > 100.0)
        *percent = 100.0;
    else if (*percent < 0.0)
        *percent = 0.0;

    return BMS_OK;
}

bms_status_t bms_update(bms_handle_t * bms, float norm_adc, float * percent)
{
    bms_status_t status = bms_read_percent(bms, norm_adc, percent);
    if (status != BMS_OK)
        return status;

    if (bms->_callback == NULL)
        return BMS_NULL_CALLBACK;

    if (*percent == 100.0)
    {
        bms->_callback(bms, BMS_EVENT_CHARGE_FULL);
    }
    else if (*percent == 0.0)
    {
        bms->_callback(bms, BMS_EVENT_CHARGE_EMPTY);
    }

    return BMS_OK;
}

/**************************************** Helper Functions *******************************************/
static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    // prevent division by 0
    if (in_max - in_min == 0)
    {
        return 0;
    }

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*****************************************************************************************************/