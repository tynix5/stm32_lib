#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32l4xx_hal.h"
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
    ENCODER_SW_RELEASE = 0,
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

typedef enum
{
    ENCODER_OK = 0,
    ENCODER_INVALID_CFG,
    ENCODER_NULL_CALLBACK,
} encoder_status_t;

typedef struct
{
    encoder_sw_state_t last_sw;
    int32_t position;
    float velocity;

    encoder_dir_t direction;

    uint32_t last_update;
    int32_t last_ticks;

} encoder_state_t;

typedef void (*encoder_callback_t)(encoder_handle_t *, encoder_event_t, encoder_event_t);

typedef struct encoder_handle_t
{

    GPIO_TypeDef * ch_a_gpio;       // encoder channel A GPIO
    uint16_t ch_a_pin;              // encoder channel A pin

    GPIO_TypeDef * ch_b_gpio;       // encoder channel B GPIO
    uint16_t ch_b_pin;              // encoder channel B pin

    GPIO_TypeDef * sw_gpio;         // encoder pushbutton GPIO
    uint16_t sw_pin;                // encoder pushbutton pin

    encoder_mode_t mode;            // method for reading encoder
    TIM_HandleTypeDef * htim;       // timer handle for MODE_TIM

    uint32_t t_update;              // delta between position updates

    // private members
    int32_t _position_max;          // max encoder position
    int32_t _position_min;          // minimum encoder position
    encoder_pos_mode_t _pos_mode;   // clamp or wrap positions
    encoder_callback_t _callback;   // event callback
    encoder_state_t _state;         // internal encoder state structure

} encoder_handle_t;

// possibly add improvements for acceleration handling?

/* Initialize all parameters in encoder structure */
encoder_status_t encoder_init(encoder_handle_t * henc);
/* Constrain encoder position to be within [min, max] */
encoder_status_t encoder_constrain(encoder_handle_t * henc, int32_t min, int32_t max);
/* Set wrap-around logic - if clamped, position will remain at extreme until moved in other direction; if wrapped, position will transfer from one
 * extreme to another when at boundary */
encoder_status_t encoder_set_pos_mode(encoder_handle_t * henc, encoder_pos_mode_t pos_mode);
/* Start timers, enable EXTI (nothing for polling) */
encoder_status_t encoder_start(encoder_handle_t * henc);
/* Stop timer, disable EXTI (nothing for polling) */
encoder_status_t encoder_stop(encoder_handle_t * henc);
/* Detect if any movements were made or button has been accessed */
encoder_status_t encoder_update(encoder_handle_t * henc);
/* Register callback function for encoder movement and switch state */
encoder_status_t encoder_register_callback(encoder_handle_t * henc, encoder_callback_t callback);
/* Get current position from internal structure */
int32_t encoder_get_position(encoder_handle_t * henc);
/* Get current velocity from internal structure */
float encoder_get_velocity(encoder_handle_t * henc);
/* Set encoder position manually */
void encoder_set_position(encoder_handle_t * henc, int32_t new_position);

#endif