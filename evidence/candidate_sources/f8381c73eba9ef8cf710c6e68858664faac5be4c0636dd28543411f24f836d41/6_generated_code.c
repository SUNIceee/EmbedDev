/* 6_generated_code.c */

#include "6_generated_code.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef DISCOBOT_TARGET
#include <stdlib.h>
#endif

#define DISCOBOT_BIT(pin) (UINT32_C(1) << (pin))

#define DISCOBOT_BUTTON_PIN 0u

#define DISCOBOT_MOTOR_A_FORWARD_PIN 1u
#define DISCOBOT_MOTOR_A_BACKWARD_PIN 2u
#define DISCOBOT_MOTOR_B_FORWARD_PIN 3u
#define DISCOBOT_MOTOR_B_BACKWARD_PIN 4u

#define DISCOBOT_MOTOR_MASK                                      \
  (DISCOBOT_BIT(DISCOBOT_MOTOR_A_FORWARD_PIN) |                  \
   DISCOBOT_BIT(DISCOBOT_MOTOR_A_BACKWARD_PIN) |                 \
   DISCOBOT_BIT(DISCOBOT_MOTOR_B_FORWARD_PIN) |                  \
   DISCOBOT_BIT(DISCOBOT_MOTOR_B_BACKWARD_PIN))

#define DISCOBOT_LED_BASE_PIN 12u
#define DISCOBOT_LED_COUNT 4u

#define DISCOBOT_TASK_CAPACITY 25u
#define DISCOBOT_USART_RX_CAPACITY 128u
#define DISCOBOT_USART_TX_CAPACITY 1024u

#define DISCOBOT_CIRCARRAY_OWNED_CAPACITY 32u

/*
 * Target mode uses a fixed storage pool because no allocator is required
 * or assumed by the target build. Queues larger than this limit are rejected.
 */
#define DISCOBOT_TARGET_CIRCARRAY_MAX_CAPACITY 1024u

#define DISCOBOT_DEFAULT_SYSTEM_CORE_CLOCK UINT32_C(16000000)
#define DISCOBOT_DEFAULT_ADC_RAW UINT16_C(760)

#define DISCOBOT_ADC_MAX_VALUE 4095.0f
#define DISCOBOT_ADC_REFERENCE_VOLTAGE 3.0f
#define DISCOBOT_TEMP_V25 0.76f
#define DISCOBOT_TEMP_AVG_SLOPE 0.0025f
#define DISCOBOT_TEMP_REFERENCE_CELSIUS 25.0f

#define DISCOBOT_ACCEL_MG_PER_G 1000.0f
#define DISCOBOT_DEBOUNCE_REQUIRED_REPEATS 1u

typedef struct {
  void (*callback)(void *context);
  void *context;
  uint32_t period_ms;
  uint32_t next_run_ms;
  uint8_t active;
} DiscobotTask;

typedef struct {
  CircArray *queue;
  uint8_t *data;
} CircArrayOwnership;

uint32_t GPIOA_output = 0u;
uint32_t GPIOD_output = 0u;
uint32_t GPIOA_IDR = 0u;
uint32_t SystemCoreClock = DISCOBOT_DEFAULT_SYSTEM_CORE_CLOCK;
uint32_t SysTick_reload =
    (DISCOBOT_DEFAULT_SYSTEM_CORE_CLOCK / UINT32_C(1000)) - UINT32_C(1);
volatile uint32_t msTicks = 0u;

static DiscobotTask g_tasks[DISCOBOT_TASK_CAPACITY];

static uint8_t g_usart_rx_storage[DISCOBOT_USART_RX_CAPACITY];
static CircArray g_usart_rx_queue = {
    g_usart_rx_storage,
    DISCOBOT_USART_RX_CAPACITY,
    0u,
    0u,
    0u,
    1,
};

static uint8_t g_usart_tx_log[DISCOBOT_USART_TX_CAPACITY];
static size_t g_usart_tx_count = 0u;

static int16_t g_accel_x_raw = 0;
static int16_t g_accel_y_raw = 0;
static int16_t g_accel_z_raw = 0;
static uint16_t g_adc_raw = DISCOBOT_DEFAULT_ADC_RAW;
static uint32_t g_rng_value = 0u;
static int g_rng_ready = 0;

static CircArrayOwnership g_circarray_owned[
    DISCOBOT_CIRCARRAY_OWNED_CAPACITY];

#ifdef DISCOBOT_TARGET
static uint8_t g_target_circarray_storage[
    DISCOBOT_CIRCARRAY_OWNED_CAPACITY]
    [DISCOBOT_TARGET_CIRCARRAY_MAX_CAPACITY];
#endif

static int g_debounce_last_sample = 0;
static int g_debounce_stable_value = 0;
static uint8_t g_debounce_repeat_count = 0u;

static void memory_zero(void *data, size_t size) {
  if (data != NULL) {
    (void)memset(data, 0, size);
  }
}

static void gpioa_write_mask(uint32_t mask, uint32_t value) {
  GPIOA_output = (GPIOA_output & ~mask) | (value & mask);
}

static void gpiod_write_mask(uint32_t mask, uint32_t value) {
  GPIOD_output = (GPIOD_output & ~mask) | (value & mask);
}

static void gpioa_set_input_pin(uint8_t pin, int high) {
  if (high != 0) {
    GPIOA_IDR |= DISCOBOT_BIT(pin);
  } else {
    GPIOA_IDR &= ~DISCOBOT_BIT(pin);
  }
}

static int gpioa_read_input_pin(uint8_t pin) {
  return (GPIOA_IDR & DISCOBOT_BIT(pin)) != 0u ? 1 : 0;
}

static int led_to_pin(uint8_t led, uint8_t *pin) {
  if (pin == NULL) {
    return 0;
  }

  if (led < DISCOBOT_LED_COUNT) {
    *pin = (uint8_t)(DISCOBOT_LED_BASE_PIN + led);
    return 1;
  }

  /*
   * Accept physical pin numbers as well as logical LED indices. This
   * preserves the existing API behavior for both calling conventions.
   */
  if (led >= DISCOBOT_LED_BASE_PIN &&
      led < DISCOBOT_LED_BASE_PIN + DISCOBOT_LED_COUNT) {
    *pin = led;
    return 1;
  }

  return 0;
}

static int task_is_due(uint32_t now, uint32_t due_time) {
  /*
   * Signed subtraction preserves correct behavior across uint32_t tick
   * rollover, provided task intervals stay within the signed range.
   */
  return (int32_t)(now - due_time) >= 0 ? 1 : 0;
}

static void circarray_set_empty(CircArray *queue) {
  if (queue == NULL) {
    return;
  }

  queue->data = NULL;
  queue->capacity = 0u;
  queue->head = 0u;
  queue->tail = 0u;
  queue->count = 0u;
  queue->initialized = 0;
}

static int circarray_ownership_find(const CircArray *queue) {
  size_t index;

  for (index = 0u; index < DISCOBOT_CIRCARRAY_OWNED_CAPACITY; ++index) {
    if (g_circarray_owned[index].queue == queue) {
      return (int)index;
    }
  }

  return -1;
}

static int circarray_ownership_find_free(void) {
  size_t index;

  for (index = 0u; index < DISCOBOT_CIRCARRAY_OWNED_CAPACITY; ++index) {
    if (g_circarray_owned[index].queue == NULL) {
      return (int)index;
    }
  }

  return -1;
}

static void circarray_ownership_remove_at(size_t index) {
  if (index >= DISCOBOT_CIRCARRAY_OWNED_CAPACITY) {
    return;
  }

  g_circarray_owned[index].queue = NULL;
  g_circarray_owned[index].data = NULL;
}

static void circarray_release_owned(CircArray *queue) {
  int owned_index;

  owned_index = circarray_ownership_find(queue);
  if (owned_index < 0) {
    return;
  }

#ifndef DISCOBOT_TARGET
  free(g_circarray_owned[owned_index].data);
#endif

  circarray_ownership_remove_at((size_t)owned_index);
}

static uint8_t *circarray_allocate_storage(size_t capacity, int slot_index) {
#ifdef DISCOBOT_TARGET
  if (slot_index < 0 ||
      capacity > DISCOBOT_TARGET_CIRCARRAY_MAX_CAPACITY) {
    return NULL;
  }

  return g_target_circarray_storage[(size_t)slot_index];
#else
  (void)slot_index;
  return (uint8_t *)malloc(capacity);
#endif
}

static void circarray_discard_storage(uint8_t *data) {
#ifndef DISCOBOT_TARGET
  free(data);
#else
  (void)data;
#endif
}

static void usart_rx_reset(void) {
  g_usart_rx_queue.data = g_usart_rx_storage;
  g_usart_rx_queue.capacity = DISCOBOT_USART_RX_CAPACITY;
  g_usart_rx_queue.head = 0u;
  g_usart_rx_queue.tail = 0u;
  g_usart_rx_queue.count = 0u;
  g_usart_rx_queue.initialized = 1;
}

static void reset_debounce_state(void) {
  const int level = button_read();

  g_debounce_last_sample = level;
  g_debounce_stable_value = level;
  g_debounce_repeat_count = 0u;
}

void init_system(void) {
  GPIOA_output = 0u;
  GPIOD_output = 0u;
  GPIOA_IDR = 0u;

  SystemCoreClock = DISCOBOT_DEFAULT_SYSTEM_CORE_CLOCK;
  SysTick_reload = (SystemCoreClock / UINT32_C(1000)) - UINT32_C(1);
  msTicks = 0u;

  usart_rx_reset();
  discobot_usart1_tx_clear();
  scheduler_init();

  g_accel_x_raw = 0;
  g_accel_y_raw = 0;
  g_accel_z_raw = 0;
  g_adc_raw = DISCOBOT_DEFAULT_ADC_RAW;
  g_rng_value = 0u;
  g_rng_ready = 0;

  reset_debounce_state();
}

void main_loop_iteration(void) {
  scheduler_run_pending();
}

void SysTick_Handler(void) {
  ++msTicks;
  scheduler_run_pending();
}

void motor_forward(uint32_t speed) {
  (void)speed;

  gpioa_write_mask(
      DISCOBOT_MOTOR_MASK,
      DISCOBOT_BIT(DISCOBOT_MOTOR_A_FORWARD_PIN) |
          DISCOBOT_BIT(DISCOBOT_MOTOR_B_FORWARD_PIN));
}

void motor_backward(uint32_t speed) {
  (void)speed;

  gpioa_write_mask(
      DISCOBOT_MOTOR_MASK,
      DISCOBOT_BIT(DISCOBOT_MOTOR_A_BACKWARD_PIN) |
          DISCOBOT_BIT(DISCOBOT_MOTOR_B_BACKWARD_PIN));
}

void motor_left(uint32_t speed) {
  (void)speed;

  gpioa_write_mask(
      DISCOBOT_MOTOR_MASK,
      DISCOBOT_BIT(DISCOBOT_MOTOR_A_BACKWARD_PIN) |
          DISCOBOT_BIT(DISCOBOT_MOTOR_B_FORWARD_PIN));
}

void motor_right(uint32_t speed) {
  (void)speed;

  gpioa_write_mask(
      DISCOBOT_MOTOR_MASK,
      DISCOBOT_BIT(DISCOBOT_MOTOR_A_FORWARD_PIN) |
          DISCOBOT_BIT(DISCOBOT_MOTOR_B_BACKWARD_PIN));
}

void motor_stop(void) {
  gpioa_write_mask(DISCOBOT_MOTOR_MASK, 0u);
}

void command_dispatch(uint8_t command) {
  switch (command) {
    case 'F':
    case 'f':
    case 'W':
    case 'w':
    case 1u:
      motor_forward(0u);
      break;

    case 'B':
    case 'b':
    case 'S':
    case 's':
    case 2u:
      motor_backward(0u);
      break;

    case 'L':
    case 'l':
    case 'A':
    case 'a':
    case 3u:
      motor_left(0u);
      break;

    case 'R':
    case 'r':
    case 'D':
    case 'd':
    case 4u:
      motor_right(0u);
      break;

    case 'X':
    case 'x':
    case '0':
    case 0u:
      motor_stop();
      break;

    default:
      break;
  }
}

int button_read(void) {
  return gpioa_read_input_pin(DISCOBOT_BUTTON_PIN);
}

int button_debounced_read(void) {
  const int sample = button_read();

  if (sample == g_debounce_last_sample) {
    if (g_debounce_repeat_count < UINT8_MAX) {
      ++g_debounce_repeat_count;
    }
  } else {
    g_debounce_last_sample = sample;
    g_debounce_repeat_count = 0u;
  }

  if (g_debounce_repeat_count >= DISCOBOT_DEBOUNCE_REQUIRED_REPEATS) {
    g_debounce_stable_value = sample;
  }

  return g_debounce_stable_value;
}

void led_on(uint8_t led) {
  uint8_t pin;

  if (led_to_pin(led, &pin) != 0) {
    gpiod_write_mask(DISCOBOT_BIT(pin), DISCOBOT_BIT(pin));
  }
}

void led_off(uint8_t led) {
  uint8_t pin;

  if (led_to_pin(led, &pin) != 0) {
    gpiod_write_mask(DISCOBOT_BIT(pin), 0u);
  }
}

void led_toggle(uint8_t led) {
  uint8_t pin;

  if (led_to_pin(led, &pin) != 0) {
    GPIOD_output ^= DISCOBOT_BIT(pin);
  }
}

void usart1_write(uint8_t byte) {
  if (g_usart_tx_count >= DISCOBOT_USART_TX_CAPACITY) {
    return;
  }

  g_usart_tx_log[g_usart_tx_count] = byte;
  ++g_usart_tx_count;
}

int usart1_read(void) {
  uint8_t byte;

  if (circarray_pop(&g_usart_rx_queue, &byte) != 0) {
    return 0;
  }

  return (int)byte;
}

int usart1_readc(void) {
  uint8_t byte;

  if (circarray_pop(&g_usart_rx_queue, &byte) != 0) {
    return -1;
  }

  return (int)(int8_t)byte;
}

void discobot_usart1_inject_rx(uint8_t byte) {
  (void)circarray_push(&g_usart_rx_queue, byte);
}

size_t discobot_usart1_tx_count(void) {
  return g_usart_tx_count;
}

uint8_t discobot_usart1_tx_at(size_t index) {
  if (index >= g_usart_tx_count) {
    return 0u;
  }

  return g_usart_tx_log[index];
}

void discobot_usart1_tx_clear(void) {
  memory_zero(g_usart_tx_log, sizeof(g_usart_tx_log));
  g_usart_tx_count = 0u;
}

void USART1_IRQHandler(void) {
  uint8_t command;

  if (circarray_pop(&g_usart_rx_queue, &command) == 0) {
    command_dispatch(command);
  }
}

void discobot_set_button_level(int high) {
  gpioa_set_input_pin(DISCOBOT_BUTTON_PIN, high != 0);
}

void discobot_set_accel_raw(int16_t x, int16_t y, int16_t z) {
  g_accel_x_raw = x;
  g_accel_y_raw = y;
  g_accel_z_raw = z;
}

void accelerometer_read_raw(int16_t *x, int16_t *y, int16_t *z) {
  if (x != NULL) {
    *x = g_accel_x_raw;
  }

  if (y != NULL) {
    *y = g_accel_y_raw;
  }

  if (z != NULL) {
    *z = g_accel_z_raw;
  }
}

void accelerometer_read_g(float *x, float *y, float *z) {
  if (x != NULL) {
    *x = (float)g_accel_x_raw / DISCOBOT_ACCEL_MG_PER_G;
  }

  if (y != NULL) {
    *y = (float)g_accel_y_raw / DISCOBOT_ACCEL_MG_PER_G;
  }

  if (z != NULL) {
    *z = (float)g_accel_z_raw / DISCOBOT_ACCEL_MG_PER_G;
  }
}

void discobot_set_adc_raw(uint16_t raw) {
  g_adc_raw = raw;
}

float temperature_read_celsius(void) {
  const float voltage =
      ((float)g_adc_raw * DISCOBOT_ADC_REFERENCE_VOLTAGE) /
      DISCOBOT_ADC_MAX_VALUE;

  return ((DISCOBOT_TEMP_V25 - voltage) / DISCOBOT_TEMP_AVG_SLOPE) +
         DISCOBOT_TEMP_REFERENCE_CELSIUS;
}

void discobot_set_rng(uint32_t value, int ready) {
  g_rng_value = value;
  g_rng_ready = ready != 0 ? 1 : 0;
}

uint32_t rng_get(void) {
  return g_rng_ready != 0 ? g_rng_value : 0u;
}

void scheduler_init(void) {
  memory_zero(g_tasks, sizeof(g_tasks));
}

int scheduler_add(void (*callback)(void *), void *context,
                  uint32_t period_ms) {
  size_t index;

  if (callback == NULL) {
    return -1;
  }

  for (index = 0u; index < DISCOBOT_TASK_CAPACITY; ++index) {
    DiscobotTask *task = &g_tasks[index];

    if (task->active != 0u) {
      continue;
    }

    task->callback = callback;
    task->context = context;
    task->period_ms = period_ms;
    task->next_run_ms = msTicks + period_ms;
    task->active = 1u;

    return (int)index;
  }

  return -1;
}

void scheduler_run_pending(void) {
  const uint32_t now = msTicks;
  size_t index;

  for (index = 0u; index < DISCOBOT_TASK_CAPACITY; ++index) {
    DiscobotTask *task = &g_tasks[index];

    if (task->active == 0u || task->callback == NULL) {
      continue;
    }

    if (task_is_due(now, task->next_run_ms) == 0) {
      continue;
    }

    task->next_run_ms = now + task->period_ms;
    task->callback(task->context);
  }
}

int circarray_init(CircArray *queue, size_t capacity) {
  const int existing_index =
      queue == NULL ? -1 : circarray_ownership_find(queue);
  const int storage_index =
      existing_index >= 0 ? existing_index : circarray_ownership_find_free();
  uint8_t *data;

  if (queue == NULL || capacity == 0u || storage_index < 0) {
    return -1;
  }

  /*
   * Allocate and validate replacement storage before changing the existing
   * queue. A failed initialization therefore leaves a valid queue untouched.
   */
  data = circarray_allocate_storage(capacity, storage_index);
  if (data == NULL) {
    return -1;
  }

  circarray_release_owned(queue);
  circarray_set_empty(queue);

  g_circarray_owned[storage_index].queue = queue;
  g_circarray_owned[storage_index].data = data;

  queue->data = data;
  queue->capacity = capacity;
  queue->head = 0u;
  queue->tail = 0u;
  queue->count = 0u;
  queue->initialized = 1;

  return 0;
}

void circarray_free(CircArray *queue) {
  if (queue == NULL) {
    return;
  }

  circarray_release_owned(queue);
  circarray_set_empty(queue);
}

int circarray_push(CircArray *queue, uint8_t value) {
  if (queue == NULL || queue->initialized == 0 ||
      queue->data == NULL || queue->capacity == 0u ||
      queue->count >= queue->capacity) {
    return -1;
  }

  queue->data[queue->tail] = value;

  if (queue->tail + 1u == queue->capacity) {
    queue->tail = 0u;
  } else {
    ++queue->tail;
  }

  ++queue->count;
  return 0;
}

int circarray_pop(CircArray *queue, uint8_t *value) {
  if (queue == NULL || value == NULL || queue->initialized == 0 ||
      queue->data == NULL || queue->capacity == 0u || queue->count == 0u) {
    return -1;
  }

  *value = queue->data[queue->head];

  if (queue->head + 1u == queue->capacity) {
    queue->head = 0u;
  } else {
    ++queue->head;
  }

  --queue->count;
  return 0;
}

int circarray_peek(const CircArray *queue, uint8_t *value) {
  if (queue == NULL || value == NULL || queue->initialized == 0 ||
      queue->data == NULL || queue->capacity == 0u || queue->count == 0u) {
    return -1;
  }

  *value = queue->data[queue->head];
  return 0;
}

size_t circarray_size(const CircArray *queue) {
  if (queue == NULL || queue->initialized == 0) {
    return 0u;
  }

  return queue->count;
}

size_t circarray_capacity(const CircArray *queue) {
  if (queue == NULL || queue->initialized == 0) {
    return 0u;
  }

  return queue->capacity;
}

int circarray_is_empty(const CircArray *queue) {
  if (queue == NULL || queue->initialized == 0 ||
      queue->count == 0u) {
    return 1;
  }

  return 0;
}

int circarray_is_full(const CircArray *queue) {
  if (queue == NULL || queue->initialized == 0 ||
      queue->capacity == 0u) {
    return 0;
  }

  return queue->count >= queue->capacity ? 1 : 0;
}

void circarray_clear(CircArray *queue) {
  if (queue == NULL || queue->initialized == 0) {
    return;
  }

  queue->head = 0u;
  queue->tail = 0u;
  queue->count = 0u;
}

int func1(int value) {
  return value + 1;
}

int func2(int a, int b) {
  return a + b;
}
