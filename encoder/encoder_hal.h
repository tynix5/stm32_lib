#ifndef ENCODER_HAL_H_
#define ENCODER_HAL_H_

#include "encoder.h"

void encoder_detect_event(encoder_handle_t * henc, encoder_event_t * enc_event, encoder_event_t * sw_event);
encoder_event_t encoder_detect_poll(encoder_handle_t * henc);
encoder_event_t encoder_detect_exti(encoder_handle_t * henc);
encoder_event_t encoder_detect_tim(encoder_handle_t * henc);
encoder_event_t encoder_detect_sw(encoder_handle_t * henc);

#endif