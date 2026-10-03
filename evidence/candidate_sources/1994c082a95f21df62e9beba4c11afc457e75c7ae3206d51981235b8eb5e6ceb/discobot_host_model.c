/*
 * discobot_host_model.c
 *
 * Deterministic C11 hardware model used when DISCOBOT_TARGET is not defined.
 * The model exposes observable GPIO, SysTick, USART, ADC, accelerometer, and
 * RNG state for host-side verification.
 */

#include "6_generated_code.h"
#include "discobot_internal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if !defined(DISCOBOT_TARGET)

enum {
  kHostUsartRxCapacity = 256U,
  kHostUsartTxLogCapacity = 4096U,
  kHostRngQueueCapacity = 64U,
  kHostAdcChannelCount = 16U,
};

/*
 * These symbols are intentionally global because they are part of the
 * observable host hardware model.
 */
uint32_t GPIOA_output = 0U;
uint32_t GPIOD_output = 0U;
uint32_t GPIOA_IDR = 0U;
uint32_t SysTick_reload = 0U;

static uint8_t g_usart_rx_storage[kHostUsartRxCapacity];
static CircArray g_usart_rx_queue;
static bool g_usart_rx_initialized = false;

static uint8_t g_usart_tx_log[kHostUsartTxLogCapacity];
static size_t g_usart_tx_log_size = 0U;

static uint16_t g_adc_values[kHostAdcChannelCount];

static int16_t g_accelerometer_x = 0;
static int16_t g_accelerometer_y = 0;
static int16_t g_accelerometer_z = 0;

/*
 * CircArray stores bytes, so the RNG queue uses a separate uint32_t FIFO.
 * Keeping this queue typed prevents truncation of injected random values.
 */
static uint32_t g_rng_storage[kHostRngQueueCapacity];
static uint16_t g_rng_head = 0U;
static uint16_t g_rng_tail = 0U;
static uint16_t g_rng_count = 0U;
static uint32_t g_rng_default_value = 0U;

extern void SysTick_Handler(void);

/*
 * USART RX model
 */

static void host_usart_rx_reset(void) {
  CircArray_Init(
      &g_usart_rx_queue,
      g_usart_rx_storage,
      (uint16_t)kHostUsartRxCapacity);
  g_usart_rx_initialized = true;
}

static void host_usart_rx_ensure_initialized(void) {
  if (!g_usart_rx_initialized) {
    host_usart_rx_reset();
  }
}

static bool host_usart_rx_is_empty(void) {
  host_usart_rx_ensure_initialized();
  return CircArray_IsEmpty(&g_usart_rx_queue) != 0U;
}

static void host_usart_rx_push(uint8_t value) {
  host_usart_rx_ensure_initialized();
  CircArray_Push(&g_usart_rx_queue, value);
}

static uint8_t host_usart_rx_pop(void) {
  host_usart_rx_ensure_initialized();
  return CircArray_Pop(&g_usart_rx_queue);
}

/*
 * RNG model
 */

static void host_rng_reset(void) {
  g_rng_head = 0U;
  g_rng_tail = 0U;
  g_rng_count = 0U;
  g_rng_default_value = 0U;
  memset(g_rng_storage, 0, sizeof(g_rng_storage));
}

static bool host_rng_is_empty(void) {
  return g_rng_count == 0U;
}

static bool host_rng_is_full(void) {
  return g_rng_count >= (uint16_t)kHostRngQueueCapacity;
}

static void host_rng_push(uint32_t value) {
  if (host_rng_is_full()) {
    return;
  }

  g_rng_storage[g_rng_tail] = value;
  g_rng_tail = (uint16_t)(
      (g_rng_tail + 1U) % (uint16_t)kHostRngQueueCapacity);
  ++g_rng_count;
}

static uint32_t host_rng_pop(void) {
  uint32_t value;

  if (host_rng_is_empty()) {
    return g_rng_default_value;
  }

  value = g_rng_storage[g_rng_head];
  g_rng_head = (uint16_t)(
      (g_rng_head + 1U) % (uint16_t)kHostRngQueueCapacity);
  --g_rng_count;

  return value;
}

/*
 * Public host model controls
 */

void host_reset_model(void) {
  GPIOA_output = 0U;
  GPIOD_output = 0U;
  GPIOA_IDR = 0U;
  SysTick_reload = 0U;

  host_usart_rx_reset();

  g_usart_tx_log_size = 0U;
  memset(g_usart_tx_log, 0, sizeof(g_usart_tx_log));

  memset(g_adc_values, 0, sizeof(g_adc_values));

  g_accelerometer_x = 0;
  g_accelerometer_y = 0;
  g_accelerometer_z = 0;

  host_rng_reset();
}

void host_set_GPIOA_IDR(uint32_t value) {
  GPIOA_IDR = value;
}

uint32_t host_get_GPIOA_output(void) {
  return GPIOA_output;
}

uint32_t host_get_GPIOD_output(void) {
  return GPIOD_output;
}

void host_uart_rx_inject(uint8_t byte) {
  host_usart_rx_push(byte);
}

size_t host_usart_tx_log_size(void) {
  return g_usart_tx_log_size;
}

uint8_t host_usart_tx_log_at(size_t index) {
  if (index >= g_usart_tx_log_size) {
    return 0U;
  }

  return g_usart_tx_log[index];
}

void host_usart_tx_log_clear(void) {
  g_usart_tx_log_size = 0U;
  memset(g_usart_tx_log, 0, sizeof(g_usart_tx_log));
}

void host_adc_inject(uint8_t channel, uint16_t raw_value) {
  if ((size_t)channel >= kHostAdcChannelCount) {
    return;
  }

  g_adc_values[channel] = raw_value;
}

void host_accel_inject(int16_t x, int16_t y, int16_t z) {
  g_accelerometer_x = x;
  g_accelerometer_y = y;
  g_accelerometer_z = z;
}

void host_rng_inject(uint32_t value) {
  host_rng_push(value);
}

void host_rng_clear(void) {
  host_rng_reset();
}

/*
 * SysTick model
 */

void systick_config_168000(void) {
  SysTick_reload = 168000U;
}

void host_systick_tick(uint32_t count) {
  uint32_t tick_index;

  for (tick_index = 0U; tick_index < count; ++tick_index) {
    SysTick_Handler();
  }
}

/*
 * GPIO adapter
 */

void disco_hw_gpioa_write(uint32_t value) {
  GPIOA_output = value;
}

uint32_t disco_hw_gpioa_read(void) {
  return GPIOA_IDR;
}

void disco_hw_gpiod_write(uint32_t value) {
  GPIOD_output = value;
}

uint32_t disco_hw_gpiod_read(void) {
  return GPIOD_output;
}

void disco_hw_gpio_init(void) {
  GPIOA_output = 0U;
  GPIOD_output = 0U;
}

void disco_hw_button_init(void) {
  /*
   * GPIOA_IDR is externally controlled through host_set_GPIOA_IDR().
   */
}

/*
 * USART adapter
 */

void disco_hw_usart_init(void) {
  host_usart_rx_reset();
  host_usart_tx_log_clear();
}

void disco_hw_usart_send_byte(uint8_t byte) {
  if (g_usart_tx_log_size >= kHostUsartTxLogCapacity) {
    return;
  }

  g_usart_tx_log[g_usart_tx_log_size] = byte;
  ++g_usart_tx_log_size;
}

bool disco_hw_usart_rx_available(void) {
  return !host_usart_rx_is_empty();
}

uint8_t disco_hw_usart_read_byte(void) {
  return host_usart_rx_pop();
}

void disco_hw_usart_rx_inject(uint8_t byte) {
  host_usart_rx_push(byte);
}

/*
 * ADC adapter
 */

void disco_hw_adc_init(void) {
  memset(g_adc_values, 0, sizeof(g_adc_values));
}

uint16_t disco_hw_adc_read(uint8_t channel) {
  if ((size_t)channel >= kHostAdcChannelCount) {
    return 0U;
  }

  return g_adc_values[channel];
}

/*
 * Accelerometer adapter
 */

void disco_hw_accelerometer_init(void) {
  g_accelerometer_x = 0;
  g_accelerometer_y = 0;
  g_accelerometer_z = 0;
}

void disco_hw_accelerometer_read(
    int16_t* x,
    int16_t* y,
    int16_t* z) {
  if (x != NULL) {
    *x = g_accelerometer_x;
  }

  if (y != NULL) {
    *y = g_accelerometer_y;
  }

  if (z != NULL) {
    *z = g_accelerometer_z;
  }
}

/*
 * RNG adapter
 */

void disco_hw_rng_init(void) {
  host_rng_reset();
}

uint32_t disco_hw_rng_get_random_number(void) {
  return host_rng_pop();
}

/*
 * Delay adapter
 *
 * Host delays are modeled as deterministic SysTick events so that
 * time-dependent behavior remains observable during tests.
 */

void disco_hw_delay(uint32_t milliseconds) {
  host_systick_tick(milliseconds);
}

/*
 * Interrupt adapters
 */

void disco_hw_usart_irq_handler(void) {
  /*
   * Injected bytes are already stored in the RX CircArray. The core USART
   * handler consumes them through disco_hw_usart_read_byte().
   */
}

void disco_hw_systick_irq_handler(void) {
  /*
   * SysTick_Handler() owns the actual timekeeping behavior in the core
   * module. This function remains an extension point for target parity.
   */
}

#endif  /* !defined(DISCOBOT_TARGET) */
