#ifndef BMS_H_
#define BMS_H_

#include <stdint.h>
#include <stddef.h>
#include "generic_types.h"

#define LIPO_CELL_MIN_V 3.75f
#define LIPO_CELL_MAX_V 4.20f

typedef struct bms_handle_t bms_handle_t;

typedef enum
{
    BMS_EVENT_CHARGE_FULL = 0,
    BMS_EVENT_CHARGE_EMPTY,

} bms_event_t;

typedef enum
{
    BMS_LIPO_TYPE = 0,

} bms_battery_type_t;

typedef void (*bms_callback_t)(bms_handle_t *, bms_event_t);

typedef struct bms_handle_t
{
    bms_battery_type_t type; // battery type
    uint8_t cell_cnt;        // # of cells
    float div;               // resistor divider factor (1K || 1K --> 2)
    float v_ref;             // ADC reference voltage
    bms_callback_t _callback;   // callback function for battery events

} bms_handle_t;

// norm_adc must be in range [0, 1]
status_t bms_init(bms_handle_t * bms, bms_callback_t callback);
status_t bms_read_voltage(bms_handle_t * bms, float norm_adc, float * v);
status_t bms_read_percent(bms_handle_t * bms, float norm_adc, float * percent);
status_t bms_update(bms_handle_t * bms, float norm_adc, float * percent);

#endif