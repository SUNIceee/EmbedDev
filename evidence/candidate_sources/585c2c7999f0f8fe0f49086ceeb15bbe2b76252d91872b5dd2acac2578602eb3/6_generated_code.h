#ifndef SIX_GENERATED_CODE_H_
#define SIX_GENERATED_CODE_H_

/*
 * 6_generated_code.h
 *
 * Public ABI boundary for the DiscoBot firmware.
 *
 * This header intentionally contains declarations only. It does not define
 * main(), create an infinite loop, or depend on STM32-specific headers.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Build and hardware constants                                              */
/* ------------------------------------------------------------------------- */

enum {
  DISCOBOT_SYSTICK_RELOAD_VALUE = 168000u,
  DISCOBOT_SYSTICK_TICK_MS = 1u,

  DISCOBOT_BUTTON_PIN = 0u,
  DISCOBOT_BUTTON_ACTIVE_LEVEL = 1u,

  DISCOBOT_MOTOR_PA1_PIN = 1u,
  DISCOBOT_MOTOR_PA2_PIN = 2u,
  DISCOBOT_MOTOR_PA3_PIN = 3u,
  DISCOBOT_MOTOR_PA4_PIN = 4u,

  DISCOBOT_LED1_PIN = 12u,
  DISCOBOT_LED2_PIN = 13u,
  DISCOBOT_LED3_PIN = 14u,
  DISCOBOT_LED4_PIN = 15u,

  DISCOBOT_ADC_CHANNEL_COUNT = 16u,

  DISCOBOT_USART_RX_CAPACITY = 64u,
  DISCOBOT_USART_TX_LOG_CAPACITY = 256u,
  DISCOBOT_RNG_QUEUE_CAPACITY = 32u,

  DISCOBOT_SCHEDULER_MAX_TASKS = 16u
};

/*
 * UART command values are deliberately represented as raw byte values.
 * The protocol has no framing or checksum layer.
 */
typedef enum DiscoBotCommand {
  DISCOBOT_COMMAND_NONE = 0x00u,
  DISCOBOT_COMMAND_FORWARD = 0x01u,
  DISCOBOT_COMMAND_BACKWARD = 0x02u,
  DISCOBOT_COMMAND_LEFT = 0x03u,
  DISCOBOT_COMMAND_RIGHT = 0x04u,
  DISCOBOT_COMMAND_STOP = 0x05u,
  DISCOBOT_COMMAND_LED_ON = 0x06u,
  DISCOBOT_COMMAND_LED_OFF = 0x07u,
  DISCOBOT_COMMAND_LED_TOGGLE = 0x08u,
  DISCOBOT_COMMAND_READ_TEMPERATURE = 0x09u,
  DISCOBOT_COMMAND_READ_ACCELEROMETER = 0x0Au,
  DISCOBOT_COMMAND_READ_RANDOM = 0x0Bu
} DiscoBotCommand;

typedef enum DiscoBotLed {
  DISCOBOT_LED1 = 1u,
  DISCOBOT_LED2 = 2u,
  DISCOBOT_LED3 = 3u,
  DISCOBOT_LED4 = 4u
} DiscoBotLed;

/* ------------------------------------------------------------------------- */
/* Public data structures                                                     */
/* ------------------------------------------------------------------------- */

/*
 * Fixed-capacity byte ring buffer.
 *
 * Push silently discards a value when the queue is full.
 * Pop returns 0 when the queue is empty.
 */
typedef struct CircArray {
  uint8_t* data;
  uint16_t capacity;
  uint16_t head;
  uint16_t tail;
  uint16_t count;
} CircArray;

/*
 * Cooperative periodic task.
 *
 * A task is disabled after initialization. Enabling a task does not add it
 * to the scheduler; scheduler_add() must be called separately.
 */
typedef struct TimedTask {
  uint32_t interval_ms;
  uint32_t last_run_ms;
  void (*callback)(void);
  uint8_t enabled;
} TimedTask;

/* ------------------------------------------------------------------------- */
/* Shared global state                                                        */
/* ------------------------------------------------------------------------- */

extern volatile uint32_t msTicks;

/*
 * The scheduler table is exposed for compatibility with the generated API.
 * Entries contain pointers to caller-owned TimedTask objects.
 */
extern TimedTask task_table[DISCOBOT_SCHEDULER_MAX_TASKS];
extern uint16_t task_count;

/* ------------------------------------------------------------------------- */
/* Core lifecycle and interrupt handlers                                      */
/* ------------------------------------------------------------------------- */

void discobot_init(void);
void main_loop_iteration(void);

void SysTick_Handler(void);
void USART1_IRQHandler(void);

void assert_param(int expression);
void delay(uint32_t milliseconds);

/* ------------------------------------------------------------------------- */
/* Motor API                                                                  */
/* ------------------------------------------------------------------------- */

void motor_forward(int speed);
void motor_backward(int speed);
void motor_left(int speed);
void motor_right(int speed);
void motor_stop(void);

void set_motor_raw(uint8_t pa1,
                   uint8_t pa2,
                   uint8_t pa3,
                   uint8_t pa4);

/* ------------------------------------------------------------------------- */
/* LED API                                                                    */
/* ------------------------------------------------------------------------- */

void led_init(void);
void led_on(uint8_t led);
void led_off(uint8_t led);
void led_toggle(uint8_t led);

/* ------------------------------------------------------------------------- */
/* Button API                                                                 */
/* ------------------------------------------------------------------------- */

void button_init(void);
int button_read_raw(void);
int button_read_debounced(void);
void button_debounce_tick_250ms(void);

/* ------------------------------------------------------------------------- */
/* SysTick API                                                                */
/* ------------------------------------------------------------------------- */

void systick_config_168000(void);

/* ------------------------------------------------------------------------- */
/* USART1 API                                                                 */
/* ------------------------------------------------------------------------- */

void USART1_Init(void);
void USART1_SendByte(uint8_t byte);
uint8_t USART1_ReadByte(void);
void USART1_SendString(const char* text);

/* ------------------------------------------------------------------------- */
/* CircArray API                                                              */
/* ------------------------------------------------------------------------- */

void CircArray_Init(CircArray* queue,
                    uint8_t* storage,
                    uint16_t capacity);

void CircArray_Push(CircArray* queue, uint8_t value);
uint8_t CircArray_Pop(CircArray* queue);
uint8_t CircArray_IsEmpty(CircArray* queue);
uint8_t CircArray_IsFull(CircArray* queue);
uint16_t CircArray_Count(CircArray* queue);

/* ------------------------------------------------------------------------- */
/* TimedTask and scheduler API                                                */
/* ------------------------------------------------------------------------- */

void TimedTask_Init(TimedTask* task,
                    uint32_t interval_ms,
                    void (*callback)(void));

void TimedTask_Enable(TimedTask* task);
void TimedTask_Disable(TimedTask* task);
void TimedTask_RunIfDue(TimedTask* task, uint32_t now_ms);

void scheduler_init(void);
void scheduler_add(TimedTask* task);
void scheduler_run_once(void);

/* ------------------------------------------------------------------------- */
/* ADC and temperature API                                                    */
/* ------------------------------------------------------------------------- */

void ADC1_Init(void);
uint16_t ADC1_ReadRaw(uint8_t channel);
float temperature_read_celsius(void);

/* ------------------------------------------------------------------------- */
/* LIS3DSH accelerometer API                                                  */
/* ------------------------------------------------------------------------- */

void LIS3DSH_Init(void);

void accelerometer_read_xyz(int16_t* x, int16_t* y, int16_t* z);
int16_t accelerometer_read_x(void);
int16_t accelerometer_read_y(void);
int16_t accelerometer_read_z(void);

/* ------------------------------------------------------------------------- */
/* RNG API                                                                    */
/* ------------------------------------------------------------------------- */

void RNG_Init(void);
uint32_t RNG_GetRandomNumber(void);

/* ------------------------------------------------------------------------- */
/* Host hardware model                                                        */
/* ------------------------------------------------------------------------- */

#ifndef DISCOBOT_TARGET

/*
 * Host-observable GPIO and SysTick state.
 *
 * GPIOA_output contains the simulated GPIOA output register.
 * GPIOD_output contains the simulated GPIOD output register.
 * GPIOA_IDR contains the simulated GPIOA input register.
 */
extern uint32_t GPIOA_output;
extern uint32_t GPIOD_output;
extern uint32_t GPIOA_IDR;
extern uint32_t SysTick_reload;

void host_reset_model(void);

void host_set_GPIOA_IDR(uint32_t value);
uint32_t host_get_GPIOA_output(void);
uint32_t host_get_GPIOD_output(void);

void host_systick_tick(uint32_t count);

void host_uart_rx_inject(uint8_t byte);
size_t host_usart_tx_log_size(void);
uint8_t host_usart_tx_log_at(size_t index);
void host_usart_tx_log_clear(void);

void host_adc_inject(uint8_t channel, uint16_t raw_value);

void host_accel_inject(int16_t x, int16_t y, int16_t z);

void host_rng_inject(uint32_t value);
void host_rng_clear(void);

uint16_t host_scheduler_task_count(void);
TimedTask* host_scheduler_task_at(uint16_t index);

#endif  /* DISCOBOT_TARGET */

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* SIX_GENERATED_CODE_H_ */
