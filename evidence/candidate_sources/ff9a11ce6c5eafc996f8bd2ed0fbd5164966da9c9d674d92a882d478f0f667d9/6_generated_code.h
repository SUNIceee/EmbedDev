/* 6_generated_code.h */
#ifndef DISCOBOT_6_GENERATED_CODE_H_
#define DISCOBOT_6_GENERATED_CODE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*task_fn)(void);

typedef struct CircArray {
  uint8_t *buffer;
  uint16_t head;
  uint16_t tail;
  uint16_t capacity;
  uint8_t overflow;
} CircArray;

void CircArray_init(CircArray *c, uint8_t *buf, uint16_t size);
uint8_t CircArray_isFull(const CircArray *c);
uint8_t CircArray_isEmpty(const CircArray *c);
uint16_t CircArray_available(const CircArray *c);
int CircArray_resize(CircArray *c, uint16_t newsize);
int CircArray_delete(CircArray *c);
int CircArray_clear(CircArray *c);
int CircArray_push(CircArray *c, uint8_t byte);
int CircArray_pop(CircArray *c, uint8_t *byte);

typedef struct TaskSlot {
  task_fn fn;
  uint32_t msinterval;
  uint32_t last_called;
  uint8_t active;
  uint32_t call_count;
} TaskSlot;

typedef struct DeviceState {
  uint8_t GPIOA_output[5];
  uint8_t GPIOD_output[16];
  uint8_t GPIOA_IDR;
  char usart1_tx_log[256];
  uint16_t usart1_tx_length;
  uint8_t usart1_rx_buffer[200];
  CircArray usart1_rx;
  TaskSlot tasks[25];
  volatile uint32_t msTicks;
  uint16_t debounce_counter;
  uint8_t button_laststate;
  uint8_t button_is_pressed;
  uint32_t rng_ready;
  uint32_t rng_value;
  uint16_t adc1_raw;
  int16_t accel_raw[3];
  uint8_t halted;
  uint8_t init_complete;
} DeviceState;

void discobot_init(void);
void main_loop_iteration(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
int usart1_send(const char *data, uint16_t len);
int usart1_readc(void);
int usart1_read(void);
uint16_t usart1_available(void);
uint8_t ButtonIsPressed(void);
void discobot_set_button_level(uint8_t level);
void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z);
void discobot_set_adc_raw(uint16_t raw);
void discobot_set_rng(uint32_t ready, uint32_t value);
void discobot_usart1_inject_rx(uint8_t byte);
float temperature_read(void);
uint32_t random_read(void);
void motor_command_fn(uint8_t cmd);
void task_add(task_fn fn, uint32_t msinterval);
void task_run_scheduler(void);
int func1(int a, int b);
int func2(int a, int b);

#ifdef __cplusplus
}
#endif

#endif /* DISCOBOT_6_GENERATED_CODE_H_ */
