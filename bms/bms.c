#include "bms.h"

static float map(float x, float in_min, float in_max, float out_min, float out_max);

void bms_init(bms_handle_t * bms)
{

}

float bms_read_voltage(bms_handle_t * bms, uint16_t raw_adc)
{
    float voltage = 0;
    uint32_t adc_states;

    switch (bms->res)
    {
        case BMS_ADC_RES_8BIT:
            adc_states = 256;
            break;
        case BMS_ADC_RES_12BIT:
        default:
            adc_states = 4096;
    }

    switch (bms->type)
    {
        case BMS_LIPO_TYPE:
            voltage = bms->div * (raw_adc / (float) adc_states) * bms->supply_v;
            break;
        default:
    }

    return voltage;
}

float bms_read_percent(bms_handle_t * bms, uint16_t raw_adc)
{
    float voltage = bms_read_voltage(bms, raw_adc);
    float percent = 0;

    switch (bms->type)
    {
        case BMS_LIPO_TYPE:
            const float lipo_max = LIPO_CELL_MAX_V * bms->cell_cnt;
            const float lipo_min = LIPO_CELL_MIN_V * bms->cell_cnt;
            percent = map(voltage, lipo_min, lipo_max, 0.0, 100.0);
            break;
        default:
    }

    if (percent > 100.0)
        percent = 100.0;
    else if (percent < 0.0)
        percent = 0.0;

    return percent;
}

float bms_update(bms_handle_t * bms, uint16_t raw_adc)
{
    float percent = bms_read_percent(bms, raw_adc);

    if (bms->callback == NULL)
        return 0.0;

    if (percent == 100.0)
    {
        bms->callback(bms, BMS_EVENT_CHARGE_FULL);
    }
    else if (percent == 0.0)
    {
        bms->callback(bms, BMS_EVENT_CHARGE_EMPTY);
    }

    return percent;
}

void bms_register_callback(bms_handle_t * bms, bms_callback callback)
{
    bms->callback = callback;
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