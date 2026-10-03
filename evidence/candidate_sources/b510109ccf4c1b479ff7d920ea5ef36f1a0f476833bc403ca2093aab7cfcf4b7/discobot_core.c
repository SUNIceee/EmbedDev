/* discobot_core.c */

#include "6_generated_code.h"
#include "discobot_internal.h"

#include <stddef.h>
#include <stdint.h>

#ifndef DISCOBOT_MOTOR_PA1_PIN
#define DISCOBOT_MOTOR_PA1_PIN 1u
#endif

#ifndef DISCOBOT_MOTOR_PA2_PIN
#define DISCOBOT_MOTOR_PA2_PIN 2u
#endif

#ifndef DISCOBOT_MOTOR_PA3_PIN
#define DISCOBOT_MOTOR_PA3_PIN 3u
#endif

#ifndef DISCOBOT_MOTOR_PA4_PIN
#define DISCOBOT_MOTOR_PA4_PIN 4u
#endif

#ifndef DISCOBOT_LED_GREEN
#define DISCOBOT_LED_GREEN 12u
#endif

#ifndef DISCOBOT_LED_ORANGE
#define DISCOBOT_LED_ORANGE 13u
#endif

#ifndef DISCOBOT_LED_RED
#define DISCOBOT_LED_RED 14u
#endif

#ifndef DISCOBOT_LED_BLUE
#define DISCOBOT_LED_BLUE 15u
#endif

#ifndef DISCOBOT_BUTTON_PIN
#define DISCOBOT_BUTTON_PIN 0u
#endif

#ifndef DISCOBOT_SYSTICK_RELOAD
#define DISCOBOT_SYSTICK_RELOAD 168000u
#endif

#ifndef DISCOBOT_BUTTON_DEBOUNCE_MS
#define DISCOBOT_BUTTON_DEBOUNCE_MS 250u
#endif

#ifndef DISCOBOT_IDLE_INTERVAL_MS
#define DISCOBOT_IDLE_INTERVAL_MS 1000u
#endif

#ifndef DISCOBOT_USART_RX_CAPACITY
#define DISCOBOT_USART_RX_CAPACITY 64u
#endif

#ifndef DISCOBOT_SCHEDULER_MAX_TASKS
#define DISCOBOT_SCHEDULER_MAX_TASKS 16u
#endif

#ifndef DISCOBOT_ADC_TEMPERATURE_CHANNEL
#define DISCOBOT_ADC_TEMPERATURE_CHANNEL 16u
#endif

#ifndef DISCOBOT_UART_CMD_FORWARD
#define DISCOBOT_UART_CMD_FORWARD ((uint8_t)'F')
#endif

#ifndef DISCOBOT_UART_CMD_BACKWARD
#define DISCOBOT_UART_CMD_BACKWARD ((uint8_t)'B')
#endif

#ifndef DISCOBOT_UART_CMD_LEFT
#define DISCOBOT_UART_CMD_LEFT ((uint8_t)'L')
#endif

#ifndef DISCOBOT_UART_CMD_RIGHT
#define DISCOBOT_UART_CMD_RIGHT ((uint8_t)'R')
#endif

#ifndef DISCOBOT_UART_CMD_STOP
#define DISCOBOT_UART_CMD_STOP ((uint8_t)'S')
#endif

#ifndef DISCOBOT_UART_CMD_LED_ON
#define DISCOBOT_UART_CMD_LED_ON ((uint8_t)'O')
#endif

#ifndef DISCOBOT_UART_CMD_LED_OFF
#define DISCOBOT_UART_CMD_LED_OFF ((uint8_t)'X')
#endif

#ifndef DISCOBOT_UART_CMD_LED_TOGGLE
#define DISCOBOT_UART_CMD_LED_TOGGLE ((uint8_t)'T')
#endif

#ifndef DISCOBOT_UART_CMD_TEMPERATURE
#define DISCOBOT_UART_CMD_TEMPERATURE ((uint8_t)'C')
#endif

#ifndef DISCOBOT_UART_CMD_ACCELEROMETER
#define DISCOBOT_UART_CMD_ACCELEROMETER ((uint8_t)'A')
#endif

#ifndef DISCOBOT_UART_CMD_RNG
#define DISCOBOT_UART_CMD_RNG ((uint8_t)'N')
#endif

#ifndef DISCOBOT_DEFAULT_MOTOR_SPEED
#define DISCOBOT_DEFAULT_MOTOR_SPEED 0
#endif

#ifndef DISCOBOT_TEMPERATURE_V25_MV
#define DISCOBOT_TEMPERATURE_V25_MV 760.0f
#endif

#ifndef DISCOBOT_TEMPERATURE_AVG_SLOPE_MV_PER_C
#define DISCOBOT_TEMPERATURE_AVG_SLOPE_MV_PER_C 2.5f
#endif

#ifndef DISCOBOT_ADC_REFERENCE_MV
#define DISCOBOT_ADC_REFERENCE_MV 3000.0f
#endif

#ifndef DISCOBOT_ADC_MAX_RAW
#define DISCOBOT_ADC_MAX_RAW 4095.0f
#endif

#if DISCOBOT_BUTTON_DEBOUNCE_MS == 0
#error "DISCOBOT_BUTTON_DEBOUNCE_MS must be greater than zero"
#endif

#if DISCOBOT_USART_RX_CAPACITY == 0
#error "DISCOBOT_USART_RX_CAPACITY must be greater than zero"
#endif

#if DISCOBOT_SCHEDULER_MAX_TASKS == 0
#error "DISCOBOT_SCHEDULER_MAX_TASKS must be greater than zero"
#endif

volatile uint32_t msTicks = 0u;

TimedTask task_table[DISCOBOT_SCHEDULER_MAX_TASKS];
uint16_t task_count = 0u;

static uint8_t usart1_rx_storage[DISCOBOT_USART_RX_CAPACITY];
static CircArray usart1_rx_queue;

static TimedTask* registered_task_sources[DISCOBOT_SCHEDULER_MAX_TASKS];

static uint8_t button_debounced_state = 0u;
static uint8_t button_last_sample = 0u;
static uint32_t last_idle_run_ms = 0u;

static uint32_t PinMask(uint8_t pin) {
  if (pin >= 32u) {
    return 0u;
  }

  return UINT32_C(1) << pin;
}

static uint8_t NormalizeLevel(uint8_t value) {
  return value == 0u ? 0u : 1u;
}

static void SendUint16LittleEndian(uint16_t value) {
  USART1_SendByte((uint8_t)(value & UINT16_C(0xff)));
  USART1_SendByte((uint8_t)((value >> 8u) & UINT16_C(0xff)));
}

static void SendInt16LittleEndian(int16_t value) {
  SendUint16LittleEndian((uint16_t)value);
}

static void SendUint32LittleEndian(uint32_t value) {
  USART1_SendByte((uint8_t)(value & UINT32_C(0xff)));
  USART1_SendByte((uint8_t)((value >> 8u) & UINT32_C(0xff)));
  USART1_SendByte((uint8_t)((value >> 16u) & UINT32_C(0xff)));
  USART1_SendByte((uint8_t)((value >> 24u) & UINT32_C(0xff)));
}

static void SendTemperatureCelsius(float celsius) {
  const int16_t centi_celsius = (int16_t)(celsius * 100.0f);

  SendInt16LittleEndian(centi_celsius);
}

static uint8_t ReadButtonLevelFromHardware(void) {
  const uint32_t button_mask = PinMask(DISCOBOT_BUTTON_PIN);

  return (discobot_hw_gpioa_read_input() & button_mask) != 0u ? 1u : 0u;
}

static void MirrorRegisteredTask(TimedTask* task) {
  uint16_t index;

  if (task == NULL) {
    return;
  }

  for (index = 0u; index < task_count; ++index) {
    if (registered_task_sources[index] == task) {
      task_table[index] = *task;
      return;
    }
  }
}

void assert_param(int expression) {
  (void)expression;
}

void delay(uint32_t ms) {
  const uint32_t start_ms = msTicks;

  while ((uint32_t)(msTicks - start_ms) < ms) {
  }
}

void systick_config_168000(void) {
  discobot_hw_systick_config(DISCOBOT_SYSTICK_RELOAD);
}

void SysTick_Handler(void) {
  ++msTicks;

  if ((msTicks % DISCOBOT_BUTTON_DEBOUNCE_MS) == 0u) {
    button_debounce_tick_250ms();
  }
}

void discobot_init(void) {
  msTicks = 0u;
  button_debounced_state = 0u;
  button_last_sample = 0u;
  last_idle_run_ms = 0u;

  systick_config_168000();
  led_init();
  motor_stop();
  button_init();
  USART1_Init();
  ADC1_Init();
  LIS3DSH_Init();
  RNG_Init();
  scheduler_init();
}

void main_loop_iteration(void) {
  const uint8_t command = USART1_ReadByte();

  if (command != 0u) {
    dispatch_uart_command(command);
  }

  (void)button_read_debounced();
  scheduler_run_once();
  run_idle_1s_logic_if_due();
}

void dispatch_uart_command(uint8_t command) {
  switch (command) {
    case DISCOBOT_UART_CMD_FORWARD:
      motor_forward(DISCOBOT_DEFAULT_MOTOR_SPEED);
      break;

    case DISCOBOT_UART_CMD_BACKWARD:
      motor_backward(DISCOBOT_DEFAULT_MOTOR_SPEED);
      break;

    case DISCOBOT_UART_CMD_LEFT:
      motor_left(DISCOBOT_DEFAULT_MOTOR_SPEED);
      break;

    case DISCOBOT_UART_CMD_RIGHT:
      motor_right(DISCOBOT_DEFAULT_MOTOR_SPEED);
      break;

    case DISCOBOT_UART_CMD_STOP:
      motor_stop();
      break;

    case DISCOBOT_UART_CMD_LED_ON:
      led_on(DISCOBOT_LED_GREEN);
      break;

    case DISCOBOT_UART_CMD_LED_OFF:
      led_off(DISCOBOT_LED_GREEN);
      break;

    case DISCOBOT_UART_CMD_LED_TOGGLE:
      led_toggle(DISCOBOT_LED_GREEN);
      break;

    case DISCOBOT_UART_CMD_TEMPERATURE:
      SendTemperatureCelsius(temperature_read_celsius());
      break;

    case DISCOBOT_UART_CMD_ACCELEROMETER: {
      int16_t x = 0;
      int16_t y = 0;
      int16_t z = 0;

      accelerometer_read_xyz(&x, &y, &z);
      SendInt16LittleEndian(x);
      SendInt16LittleEndian(y);
      SendInt16LittleEndian(z);
      break;
    }

    case DISCOBOT_UART_CMD_RNG:
      SendUint32LittleEndian(RNG_GetRandomNumber());
      break;

    default:
      break;
  }
}

void run_idle_1s_logic_if_due(void) {
  if ((uint32_t)(msTicks - last_idle_run_ms) < DISCOBOT_IDLE_INTERVAL_MS) {
    return;
  }

  last_idle_run_ms = msTicks;
}

void set_motor_raw(uint8_t pa1,
                   uint8_t pa2,
                   uint8_t pa3,
                   uint8_t pa4) {
  const uint32_t clear_mask =
      PinMask(DISCOBOT_MOTOR_PA1_PIN) |
      PinMask(DISCOBOT_MOTOR_PA2_PIN) |
      PinMask(DISCOBOT_MOTOR_PA3_PIN) |
      PinMask(DISCOBOT_MOTOR_PA4_PIN);

  uint32_t set_mask = 0u;

  pa1 = NormalizeLevel(pa1);
  pa2 = NormalizeLevel(pa2);
  pa3 = NormalizeLevel(pa3);
  pa4 = NormalizeLevel(pa4);

  /*
   * A motor direction pair is mutually exclusive. A conflicting command
   * disables both inputs for that motor to provide a fail-safe state.
   */
  if (pa1 != 0u && pa2 != 0u) {
    pa1 = 0u;
    pa2 = 0u;
  }

  if (pa3 != 0u && pa4 != 0u) {
    pa3 = 0u;
    pa4 = 0u;
  }

  if (pa1 != 0u) {
    set_mask |= PinMask(DISCOBOT_MOTOR_PA1_PIN);
  }

  if (pa2 != 0u) {
    set_mask |= PinMask(DISCOBOT_MOTOR_PA2_PIN);
  }

  if (pa3 != 0u) {
    set_mask |= PinMask(DISCOBOT_MOTOR_PA3_PIN);
  }

  if (pa4 != 0u) {
    set_mask |= PinMask(DISCOBOT_MOTOR_PA4_PIN);
  }

  discobot_hw_gpioa_write_mask(clear_mask, set_mask);
}

void motor_forward(int speed) {
  (void)speed;
  set_motor_raw(1u, 0u, 1u, 0u);
}

void motor_backward(int speed) {
  (void)speed;
  set_motor_raw(0u, 1u, 0u, 1u);
}

void motor_left(int speed) {
  (void)speed;
  set_motor_raw(0u, 1u, 1u, 0u);
}

void motor_right(int speed) {
  (void)speed;
  set_motor_raw(1u, 0u, 0u, 1u);
}

void motor_stop(void) {
  set_motor_raw(0u, 0u, 0u, 0u);
}

void led_init(void) {
  discobot_hw_led_init();
}

void led_on(uint8_t led) {
  const uint32_t led_mask = PinMask(led);

  discobot_hw_gpiod_write_mask(led_mask, led_mask);
}

void led_off(uint8_t led) {
  discobot_hw_gpiod_write_mask(PinMask(led), 0u);
}

void led_toggle(uint8_t led) {
  discobot_hw_gpiod_toggle(PinMask(led));
}

void button_init(void) {
  button_last_sample = ReadButtonLevelFromHardware();
  button_debounced_state = button_last_sample;
}

int button_read_raw(void) {
  return (int)ReadButtonLevelFromHardware();
}

int button_read_debounced(void) {
  return (int)button_debounced_state;
}

void button_debounce_tick_250ms(void) {
  const uint8_t sample = ReadButtonLevelFromHardware();

  if (sample == button_last_sample) {
    button_debounced_state = sample;
  }

  button_last_sample = sample;
}

void USART1_Init(void) {
  CircArray_Init(
      &usart1_rx_queue,
      usart1_rx_storage,
      (uint16_t)DISCOBOT_USART_RX_CAPACITY);

  discobot_hw_usart1_init();
}

void USART1_IRQHandler(void) {
  while (discobot_hw_usart1_rx_ready()) {
    CircArray_Push(
        &usart1_rx_queue,
        discobot_hw_usart1_read_byte());
  }
}

void USART1_SendByte(uint8_t byte) {
  discobot_hw_usart1_send_byte(byte);
}

uint8_t USART1_ReadByte(void) {
  return CircArray_Pop(&usart1_rx_queue);
}

void USART1_SendString(const char* text) {
  if (text == NULL) {
    return;
  }

  while (*text != '\0') {
    USART1_SendByte((uint8_t)*text);
    ++text;
  }
}

void discobot_usart1_receive_byte(uint8_t byte) {
  CircArray_Push(&usart1_rx_queue, byte);
}

void CircArray_Init(CircArray* q, uint8_t* storage, uint16_t capacity) {
  if (q == NULL) {
    return;
  }

  q->data = storage;
  q->capacity = capacity;
  q->head = 0u;
  q->tail = 0u;
  q->count = 0u;
}

void CircArray_Push(CircArray* q, uint8_t value) {
  if (q == NULL || q->data == NULL || q->capacity == 0u) {
    return;
  }

  if (q->count >= q->capacity) {
    return;
  }

  q->data[q->tail] = value;
  q->tail = (uint16_t)((q->tail + 1u) % q->capacity);
  ++q->count;
}

uint8_t CircArray_Pop(CircArray* q) {
  uint8_t value;

  if (q == NULL || q->data == NULL || q->capacity == 0u ||
      q->count == 0u) {
    return 0u;
  }

  value = q->data[q->head];
  q->head = (uint16_t)((q->head + 1u) % q->capacity);
  --q->count;

  return value;
}

uint8_t CircArray_IsEmpty(CircArray* q) {
  if (q == NULL) {
    return 1u;
  }

  return q->count == 0u ? 1u : 0u;
}

uint8_t CircArray_IsFull(CircArray* q) {
  if (q == NULL) {
    return 0u;
  }

  return q->capacity != 0u && q->count >= q->capacity ? 1u : 0u;
}

uint16_t CircArray_Count(CircArray* q) {
  if (q == NULL) {
    return 0u;
  }

  return q->count;
}

void TimedTask_Init(TimedTask* task,
                    uint32_t interval_ms,
                    void (*callback)(void)) {
  if (task == NULL) {
    return;
  }

  task->interval_ms = interval_ms;
  task->last_run_ms = msTicks;
  task->callback = callback;
  task->enabled = 0u;

  MirrorRegisteredTask(task);
}

void TimedTask_Enable(TimedTask* task) {
  if (task == NULL) {
    return;
  }

  task->enabled = 1u;
  MirrorRegisteredTask(task);
}

void TimedTask_Disable(TimedTask* task) {
  if (task == NULL) {
    return;
  }

  task->enabled = 0u;
  MirrorRegisteredTask(task);
}

void TimedTask_RunIfDue(TimedTask* task, uint32_t now_ms) {
  if (task == NULL || task->enabled == 0u ||
      task->callback == NULL) {
    return;
  }

  if ((uint32_t)(now_ms - task->last_run_ms) < task->interval_ms) {
    return;
  }

  task->last_run_ms = now_ms;
  task->callback();
}

void scheduler_init(void) {
  uint16_t index;

  task_count = 0u;

  for (index = 0u; index < DISCOBOT_SCHEDULER_MAX_TASKS; ++index) {
    registered_task_sources[index] = NULL;
    TimedTask_Init(&task_table[index], 0u, NULL);
  }
}

void scheduler_add(TimedTask* task) {
  if (task == NULL || task_count >= DISCOBOT_SCHEDULER_MAX_TASKS) {
    return;
  }

  registered_task_sources[task_count] = task;
  task_table[task_count] = *task;
  ++task_count;
}

void scheduler_run_once(void) {
  uint16_t index;

  for (index = 0u; index < task_count; ++index) {
    TimedTask* task = registered_task_sources[index];

    if (task == NULL) {
      task = &task_table[index];
    }

    TimedTask_RunIfDue(task, msTicks);
    task_table[index] = *task;
  }
}

uint16_t host_scheduler_task_count(void) {
  return task_count;
}

TimedTask* host_scheduler_task_at(uint16_t index) {
  if (index >= task_count) {
    return NULL;
  }

  if (registered_task_sources[index] != NULL) {
    return registered_task_sources[index];
  }

  return &task_table[index];
}

void ADC1_Init(void) {
  discobot_hw_adc1_init();
}

uint16_t ADC1_ReadRaw(uint8_t channel) {
  return discobot_hw_adc1_read_raw(channel);
}

float temperature_read_celsius(void) {
  const uint16_t raw_value =
      ADC1_ReadRaw(DISCOBOT_ADC_TEMPERATURE_CHANNEL);
  const float voltage_mv =
      ((float)raw_value * DISCOBOT_ADC_REFERENCE_MV) /
      DISCOBOT_ADC_MAX_RAW;

  return ((voltage_mv - DISCOBOT_TEMPERATURE_V25_MV) /
          DISCOBOT_TEMPERATURE_AVG_SLOPE_MV_PER_C) +
         25.0f;
}

void LIS3DSH_Init(void) {
  discobot_hw_lis3dsh_init();
}

void accelerometer_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
  int16_t local_x = 0;
  int16_t local_y = 0;
  int16_t local_z = 0;

  discobot_hw_lis3dsh_read_xyz(
      &local_x,
      &local_y,
      &local_z);

  if (x != NULL) {
    *x = local_x;
  }

  if (y != NULL) {
    *y = local_y;
  }

  if (z != NULL) {
    *z = local_z;
  }
}

int16_t accelerometer_read_x(void) {
  int16_t x = 0;

  accelerometer_read_xyz(&x, NULL, NULL);
  return x;
}

int16_t accelerometer_read_y(void) {
  int16_t y = 0;

  accelerometer_read_xyz(NULL, &y, NULL);
  return y;
}

int16_t accelerometer_read_z(void) {
  int16_t z = 0;

  accelerometer_read_xyz(NULL, NULL, &z);
  return z;
}

void RNG_Init(void) {
  discobot_hw_rng_init();
}

uint32_t RNG_GetRandomNumber(void) {
  return discobot_hw_rng_get_random_number();
}
