#ifndef BMS_H_
#define BMS_H_

#include <stdint.h>
#include <stddef.h>

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

typedef enum
{
    BMS_OK = 0,
    BMS_INVALID_CFG,
    BMS_NULL_CALLBACK,
} bms_status_t;

typedef void (*bms_callback_t)(bms_handle_t *, bms_event_t);

typedef struct bms_handle_t
{
    bms_battery_type_t type;  // battery type
    uint8_t cell_cnt;         // # of cells
    float div;                // resistor divider factor (1K || 1K --> 2)
    float v_ref;              // ADC reference voltage
    float v_offset;           // voltage offset (must be measured)
    bms_callback_t _callback; // callback function for battery events

} bms_handle_t;

/* Initialize BMS and register event callback */
/* [in/out] bms - BMS structure handle */
/* [in] callback - callback function pointer */
/* [return] BMS_OK if valid callback, BMS_NULL_CALLBACK otherwise */
bms_status_t bms_init(bms_handle_t * bms, bms_callback_t callback);

/* Calculate voltage based on normalized ADC measurement */
/* [in] bms - BMS structure handle */
/* [in] norm_adc - normalized ADC input [0, 1] - divide raw ADC output by resolution */
/* [out] v - calculated output voltage */
/* [return] BMS_OK if successful, BMS_INVALID_CFG otherwise */
bms_status_t bms_read_voltage(const bms_handle_t * bms, float norm_adc, float * v);

/* Calculate percent full based on normalized ADC measurement and battery type */
/* [in] bms - BMS structure handle */
/* [in] norm_adc - normalized ADC input [0, 1] */
/* [out] percent - calculated battery percent [0, 100] */
/* [return] BMS_OK if successful, BMS_INVALID_CFG otherwise */
bms_status_t bms_read_percent(const bms_handle_t * bms, float norm_adc, float * percent);

/* Calculate battery percentage and automatically callback if special event */
/* [in] bms - BMS structure handle */
/* [in] norm_adc - normalized ADC input [0, 1] */
/* [out] percent - calculated battery percent [0, 100] */
/* [return] BMS_OK if successful */
bms_status_t bms_update(bms_handle_t * bms, float norm_adc, float * percent);

#endif