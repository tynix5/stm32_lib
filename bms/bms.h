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
    BMS_ADC_RES_8BIT = 0,
    BMS_ADC_RES_12BIT

} bms_adc_res_t;

typedef void (*bms_callback)(bms_handle_t *, bms_event_t);

typedef struct bms_handle_t
{
    bms_battery_type_t type; // battery type
    bms_adc_res_t res;       // ADC resolution
    uint8_t cell_cnt;        // # of cells
    float div;               // resistor divider factor (1K || 1K --> 2)
    float supply_v;          // ADC reference voltage
    bms_callback callback;   // callback function for battery events

} bms_handle_t;

void bms_init(bms_handle_t * bms);
float bms_read_voltage(bms_handle_t * bms, uint16_t raw_adc);
float bms_read_percent(bms_handle_t * bms, uint16_t raw_adc);
float bms_update(bms_handle_t * bms, uint16_t raw_adc);
void bms_register_callback(bms_handle_t * bms, bms_callback callback);

#endif