#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

/*
 * Public frozen API for the Discobot C11 library.
 *
 * Host mode is the default when DISCOBOT_TARGET is not defined. This header is
 * intentionally limited to C standard headers, so host tests do not require
 * STM32, SPL, CMSIS, or ARM-specific include paths.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Observable host/target state. */
extern uint32_t GPIOA_output;
extern uint32_t GPIOD_output;
extern uint32_t GPIOA_IDR;

extern uint32_t SystemCoreClock;
extern uint32_t SysTick_reload;
extern volatile uint32_t msTicks;

/* Fixed-size byte ring buffer used by the USART model and host tests. */
typedef struct CircArray {
  uint8_t *data;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
  int initialized;
} CircArray;

/* System lifecycle. */
void init_system(void);
void main_loop_iteration(void);

/* SysTick 1 ms tick handler. */
void SysTick_Handler(void);

/* USART1 interrupt handler and raw byte I/O. */
void USART1_IRQHandler(void);
void usart1_write(uint8_t byte);
int usart1_read(void);
int usart1_readc(void);

/* Host-test USART helpers. */
void discobot_usart1_inject_rx(uint8_t byte);
size_t discobot_usart1_tx_count(void);
uint8_t discobot_usart1_tx_at(size_t index);
void discobot_usart1_tx_clear(void);

/* Bare byte command dispatcher. */
void command_dispatch(uint8_t command);

/* GPIO motor control. The speed argument is retained for API compatibility. */
void motor_forward(uint32_t speed);
void motor_backward(uint32_t speed);
void motor_left(uint32_t speed);
void motor_right(uint32_t speed);
void motor_stop(void);

/* LED control for STM32F407 Discovery LEDs mapped to PD12..PD15. */
void led_on(uint8_t led);
void led_off(uint8_t led);
void led_toggle(uint8_t led);

/* Button input on PA0, active high. */
int button_read(void);
int button_debounced_read(void);
void discobot_set_button_level(int high);

/* Accelerometer injection and readout. */
void discobot_set_accel_raw(int16_t x, int16_t y, int16_t z);
void accelerometer_read_raw(int16_t *x, int16_t *y, int16_t *z);
void accelerometer_read_g(float *x, float *y, float *z);

/* ADC-backed temperature model. */
void discobot_set_adc_raw(uint16_t raw);
float temperature_read_celsius(void);

/* RNG injection and readout. */
void discobot_set_rng(uint32_t value, int ready);
uint32_t rng_get(void);

/* Fixed 25-slot cooperative scheduler. */
void scheduler_init(void);
int scheduler_add(void (*callback)(void *), void *context, uint32_t period_ms);
void scheduler_run_pending(void);

/* CircArray API. */
int circarray_init(CircArray *q, size_t capacity);
void circarray_free(CircArray *q);
int circarray_push(CircArray *q, uint8_t value);
int circarray_pop(CircArray *q, uint8_t *value);
int circarray_peek(const CircArray *q, uint8_t *value);
size_t circarray_size(const CircArray *q);
size_t circarray_capacity(const CircArray *q);
int circarray_is_empty(const CircArray *q);
int circarray_is_full(const CircArray *q);
void circarray_clear(CircArray *q);

/* ARM assembly compatibility API. */
int func1(int value);
int func2(int a, int b);

#ifdef __cplusplus
}
#endif

#endif  /* SIX_GENERATED_CODE_H_ */
