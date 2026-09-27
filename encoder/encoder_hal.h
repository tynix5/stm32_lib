#ifndef ENCODER_HAL_H_
#define ENCODER_HAL_H_

#include "encoder.h"

/* Update encoder status struct - detect turns and read button state */
encoder_status_t encoder_detect_event(encoder_handle_t * henc, encoder_event_t * enc_event, encoder_event_t * sw_event);
/* Detect turns by polling GPIO */
encoder_event_t encoder_detect_poll(encoder_handle_t * henc);
/* Detect turns using EXTI */
encoder_event_t encoder_detect_exti(encoder_handle_t * henc);
/* Detect turns using STM32 Timer encoder modes */
encoder_event_t encoder_detect_tim(encoder_handle_t * henc);
/* Read switch state */
encoder_event_t encoder_detect_sw(encoder_handle_t * henc);

#endif