#ifndef DISCOBOT_INTERNAL_H_
#define DISCOBOT_INTERNAL_H_

/*
 * Internal interfaces shared by the core implementation and the selected
 * hardware adapter. This header is not part of the public API.
 */

#include "6_generated_code.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Internal hardware abstraction layer. */

void discobot_hw_init(void);

void discobot_hw_systick_config(uint32_t reload_value);
void discobot_hw_systick_delay(uint32_t milliseconds);

void discobot_hw_motor_write(uint8_t pa1,
                             uint8_t pa2,
                             uint8_t pa3,
                             uint8_t pa4);

void discobot_hw_led_init(void);
void discobot_hw_led_write(uint8_t led, bool enabled);
bool discobot_hw_led_read(uint8_t led);

uint32_t discobot_hw_gpioa_read(void);
uint32_t discobot_hw_gpiod_read(void);

void discobot_hw_usart_init(void);
bool discobot_hw_usart_rx_ready(void);
uint8_t discobot_hw_usart_read_byte(void);
void discobot_hw_usart_write_byte(uint8_t byte);

void discobot_hw_adc_init(void);
uint16_t discobot_hw_adc_read_raw(uint8_t channel);

void discobot_hw_accelerometer_init(void);
void discobot_hw_accelerometer_read_xyz(int16_t* x,
                                        int16_t* y,
                                        int16_t* z);

void discobot_hw_rng_init(void);
uint32_t discobot_hw_rng_read(void);

/* Core state shared by interrupt handlers and the main-loop implementation. */

extern volatile uint32_t discobot_ms_ticks;
extern volatile uint8_t discobot_button_state;
extern volatile uint8_t discobot_button_candidate;
extern volatile uint8_t discobot_button_stable_ticks;

/* Core implementation helpers. */

void init_all_fixed_api_objects(void);
void dispatch_uart_command(uint8_t command);
void run_idle_1s_logic_if_due(void);

/* Host-only observable hardware model and test injection interfaces. */

#ifndef DISCOBOT_TARGET

extern uint32_t GPIOA_output;
extern uint32_t GPIOD_output;
extern uint32_t GPIOA_IDR;
extern uint32_t SysTick_reload;

void host_reset_model(void);

void host_set_GPIOA_IDR(uint32_t value);
uint32_t host_get_GPIOA_output(void);
uint32_t host_get_GPIOD_output(void);
uint32_t host_get_GPIOA_IDR(void);
uint32_t host_get_SysTick_reload(void);
uint32_t host_get_ms_ticks(void);

void host_systick_tick(uint32_t count);

void host_uart_rx_inject(uint8_t byte);
size_t host_usart_tx_log_size(void);
uint8_t host_usart_tx_log_at(size_t index);
void host_usart_tx_log_clear(void);

void host_adc_inject(uint8_t channel, uint16_t raw_value);

void host_accel_inject(int16_t x, int16_t y, int16_t z);

void host_rng_inject(uint32_t value);
void host_rng_clear(void);

#endif  /* DISCOBOT_TARGET */

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* DISCOBOT_INTERNAL_H_ */
