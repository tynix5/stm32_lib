#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32l4xx_hal.h"
#include "error_types.h"
#include <stdint.h>
#include <math.h>

typedef struct encoder_handle_t encoder_handle_t;

typedef enum
{
    ENCODER_DIR_NONE = 0,
    ENCODER_DIR_CW,
    ENCODER_DIR_CCW
} encoder_dir_t;

typedef enum
{
    ENCODER_SW_NO_CHANGE = 0,
    ENCODER_SW_RELEASE,
    ENCODER_SW_PRESS
} encoder_sw_state_t;

typedef enum
{
    ENCODER_MODE_POLL = 0,
    ENCODER_MODE_EXTI,
    ENCODER_MODE_TIM
} encoder_mode_t;

typedef enum
{
    ENCODER_POS_CLAMP = 0,
    ENCODER_POS_WRAP
} encoder_pos_mode_t;

typedef enum
{
    ENCODER_EVENT_NONE = 0,
    ENCODER_EVENT_CW_TURN,
    ENCODER_EVENT_CCW_TURN,
    ENCODER_EVENT_SW_PRESS,
    ENCODER_EVENT_SW_RELEASE
} encoder_event_t;

typedef struct
{

    encoder_sw_state_t sw, prev_sw;
    int32_t position, prev_position;
    float velocity;

    encoder_dir_t direction;

    uint32_t prev_update;
    int32_t ticks, prev_ticks, delta_ticks;

} encoder_state_t;

typedef void (*encoder_callback_t)(encoder_handle_t *, encoder_event_t);

typedef struct encoder_handle_t
{

    GPIO_TypeDef * ch_a_gpio;
    uint16_t ch_a_pin;

    GPIO_TypeDef * ch_b_gpio;
    uint16_t ch_b_pin;

    GPIO_TypeDef * sw_gpio;
    uint16_t sw_pin;

    encoder_mode_t mode;
    TIM_HandleTypeDef * htim;

    // "private" members (move into a constraints struct?)
    int32_t position_max;
    int32_t position_min;
    encoder_pos_mode_t pos_mode;

    encoder_callback_t callback;
    encoder_callback_t sw_callback;

    encoder_state_t state;

    uint32_t t_update;

} encoder_handle_t;

// possibly add improvements for acceleration handling?

status_t encoder_init(encoder_handle_t * henc);
status_t encoder_constrain(encoder_handle_t * henc, int32_t min, int32_t max);
status_t encoder_set_pos_mode(encoder_handle_t * henc, encoder_pos_mode_t pos_mode);
status_t encoder_start(encoder_handle_t * henc);
status_t encoder_stop(encoder_handle_t * henc);
status_t encoder_update(encoder_handle_t * henc);
status_t encoder_register_callback(encoder_handle_t * henc, encoder_callback_t callback);
status_t encoder_register_sw_callback(encoder_handle_t * henc, encoder_callback_t callback);
int32_t encoder_get_position(encoder_handle_t * henc);

#endif