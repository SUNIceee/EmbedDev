/* 6_generated_code.c
 *
 * Production translation unit for the frozen DiscoBot API. The public
 * contract lives in 6_generated_code.h and must not be changed by this file.
 *
 * When DISCOBOT_TARGET is defined, the code targets an STM32F407 using the
 * CMSIS/SPL headers and real peripheral registers. Otherwise it builds a
 * deterministic host model with no external dependencies.
 */

#include "6_generated_code.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#endif

/* ------------------------------------------------------------------------- */
/* Host-only SysTick                                         */
/* ------------------------------------------------------------------------- */

#ifndef DISCOBOT_TARGET
static uint32_t SysTick_Config(uint32_t ticks) {
  (void)ticks;
  return 0u;
}
#endif

/* ------------------------------------------------------------------------- */
/* Private device state                                      */
/* ------------------------------------------------------------------------- */

static DeviceState s_device;

/* Public banner. The terminator must not be transmitted or logged. */
static const char kInitBanner[] = "UART1 Initialized. @9600bps\r\n";
static const uint16_t kInitBannerLength =
    (uint16_t)(sizeof(kInitBanner) - 1u);

static const uint32_t kTaskCount =
    (uint32_t)(sizeof(s_device.tasks) / sizeof(s_device.tasks[0]));

static const uint32_t kDebounceThreshold = 250u;

/* Raw-byte command dispatch table. Each byte 0..8 maps to one private
 * dispatcher function so the public motor_command_fn remains a single
 * truth-table writer. */
static void command_0(void) { motor_command_fn(0u); }
static void command_1(void) { motor_command_fn(1u); }
static void command_2(void) { motor_command_fn(2u); }
static void command_3(void) { motor_command_fn(3u); }
static void command_4(void) { motor_command_fn(4u); }
static void command_5(void) { motor_command_fn(5u); }
static void command_6(void) { motor_command_fn(6u); }
static void command_7(void) { motor_command_fn(7u); }
static void command_8(void) { motor_command_fn(8u); }

static void (*const flookup[9])(void) = {
    command_0, command_1, command_2, command_3, command_4,
    command_5, command_6, command_7, command_8};

/* ------------------------------------------------------------------------- */
/* Circular buffer implementation                            */
/* ------------------------------------------------------------------------- */

void CircArray_init(CircArray* c, uint8_t* buf, uint16_t size) {
  if (c == NULL) {
    return;
  }
  c->buffer = buf;
  c->capacity = size;
  c->head = 0u;
  c->tail = 0u;
  c->overflow = 0u;
}

uint8_t CircArray_isFull(const CircArray* c) {
  if (c == NULL || c->capacity == 0u) {
    return 1u;
  }
  return ((uint16_t)(c->head + 1u) % c->capacity) == c->tail ? 1u : 0u;
}

uint8_t CircArray_isEmpty(const CircArray* c) {
  if (c == NULL || c->capacity == 0u) {
    return 1u;
  }
  return c->head == c->tail ? 1u : 0u;
}

uint16_t CircArray_available(const CircArray* c) {
  if (c == NULL || c->capacity == 0u) {
    return 0u;
  }
  if (c->head >= c->tail) {
    return (uint16_t)(c->head - c->tail);
  }
  return (uint16_t)(c->capacity - (c->tail - c->head));
}

int CircArray_resize(CircArray* c, uint16_t newsize) {
  if (c == NULL || newsize == 0u || c->buffer == NULL) {
    return -1;
  }
  c->capacity = newsize;
  c->head = 0u;
  c->tail = 0u;
  c->overflow = 0u;
  return 0;
}

int CircArray_delete(CircArray* c) {
  if (c == NULL) {
    return -1;
  }
  c->buffer = NULL;
  c->capacity = 0u;
  c->head = 0u;
  c->tail = 0u;
  c->overflow = 0u;
  return 0;
}

int CircArray_clear(CircArray* c) {
  if (c == NULL) {
    return -1;
  }
  c->head = 0u;
  c->tail = 0u;
  c->overflow = 0u;
  return 0;
}

int CircArray_push(CircArray* c, uint8_t byte) {
  if (c == NULL || c->buffer == NULL || c->capacity == 0u) {
    return 0;
  }
  if (CircArray_isFull(c)) {
    c->overflow = 1u;
    return 0;
  }
  c->buffer[c->head] = byte;
  c->head = (uint16_t)((c->head + 1u) % c->capacity);
  return 1;
}

int CircArray_pop(CircArray* c, uint8_t* byte) {
  if (c == NULL || byte == NULL || c->buffer == NULL || c->capacity == 0u ||
      CircArray_isEmpty(c)) {
    return 0;
  }
  *byte = c->buffer[c->tail];
  c->tail = (uint16_t)((c->tail + 1u) % c->capacity);
  return 1;
}

/* ------------------------------------------------------------------------- */
/* Public initialization                                     */
/* ------------------------------------------------------------------------- */

void discobot_init(void) {
  (void)memset(s_device.GPIOA_output, 0, sizeof(s_device.GPIOA_output));
  (void)memset(s_device.GPIOD_output, 0, sizeof(s_device.GPIOD_output));
  s_device.GPIOA_IDR = 0u;

  (void)memset(s_device.usart1_tx_log, 0, sizeof(s_device.usart1_tx_log));
  s_device.usart1_tx_length = 0u;

  CircArray_init(&s_device.usart1_rx, s_device.usart1_rx_buffer,
                 (uint16_t)sizeof(s_device.usart1_rx_buffer));

  for (uint32_t i = 0u; i < kTaskCount; ++i) {
    s_device.tasks[i].fn = NULL;
    s_device.tasks[i].msinterval = 0u;
    s_device.tasks[i].last_called = 0u;
    s_device.tasks[i].active = 0u;
    s_device.tasks[i].call_count = 0u;
  }

  s_device.msTicks = 0u;
  s_device.debounce_counter = 0u;
  s_device.button_laststate = 0u;
  s_device.button_is_pressed = 0u;
  s_device.rng_ready = 0u;
  s_device.rng_value = 0u;
  s_device.adc1_raw = 0u;
  s_device.accel_raw[0] = 0;
  s_device.accel_raw[1] = 0;
  s_device.accel_raw[2] = 0;
  s_device.halted = 0u;
  s_device.init_complete = 0u;

#ifdef DISCOBOT_TARGET
  const uint32_t usart_baud_rate = 9600u;
  const uint32_t apb2_clock_hz = 84000000u;

  /* Enable GPIOA, GPIOD, USART1, ADC1, and RNG clocks. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

  /* Configure GPIOA pins 1..4 as outputs. */
  GPIOA->MODER &= ~((3u << 2) | (3u << 4) | (3u << 6) | (3u << 8));
  GPIOA->MODER |= (1u << 2) | (1u << 4) | (1u << 6) | (1u << 8);

  /* Configure GPIOD pins 12..15 as outputs. */
  GPIOD->MODER &= ~(0xFFu << 24);
  GPIOD->MODER |= (0x55u << 24);

  /* Configure USART1 GPIO: PA9=TX, PA10=RX as AF7. */
  GPIOA->MODER &= ~((3u << 18) | (3u << 20));
  GPIOA->MODER |= (2u << 18) | (2u << 20);
  GPIOA->AFR[1] &= ~(0xFFu << 4);
  GPIOA->AFR[1] |= (0x77u << 4);

  /* 9600 baud from APB2 clock; assumes PCLK2 = 84 MHz. */
  USART1->BRR = apb2_clock_hz / usart_baud_rate;
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  USART1->CR1 |= USART_CR1_RXNEIE;
  NVIC_EnableIRQ(USART1_IRQn);

  /* Enable the hardware random number generator. */
  RNG->CR |= RNG_CR_RNGEN;

  /* Enable ADC1. More elaborate conversion setup is application-specific. */
  ADC1->CR2 |= ADC_CR2_ADON;
#endif

  if (0u == SysTick_Config(168000u)) {
    usart1_send(kInitBanner, kInitBannerLength);
    s_device.init_complete = 1u;
  } else {
    s_device.halted = 1u;
  }
}

/* ------------------------------------------------------------------------- */
/* Main-loop behavior                                        */
/* ------------------------------------------------------------------------- */

void main_loop_iteration(void) {
  /* Dispatch all currently queued USART RX bytes. */
  while (CircArray_available(&s_device.usart1_rx) > 0u) {
    uint8_t byte = 0u;
    if (CircArray_pop(&s_device.usart1_rx, &byte) == 0) {
      break;
    }
    if (byte <= 8u) {
      flookup[byte]();
    }
  }

  /* Button debounce. */
  uint8_t level = 0u;
#ifdef DISCOBOT_TARGET
  level = (GPIOA->IDR & 0x1u) != 0u ? 1u : 0u;
#else
  level = s_device.GPIOA_IDR != 0u ? 1u : 0u;
#endif

  s_device.button_laststate = level;

  if (level != 0u) {
    if (s_device.debounce_counter < 0xFFFFu) {
      ++s_device.debounce_counter;
    }
    if (s_device.debounce_counter >= kDebounceThreshold) {
      s_device.button_is_pressed = 1u;
    }
  } else {
    s_device.debounce_counter = 0u;
    s_device.button_is_pressed = 0u;
  }

  task_run_scheduler();
}

/* ------------------------------------------------------------------------- */
/* Interrupt handlers                                        */
/* ------------------------------------------------------------------------- */

void SysTick_Handler(void) {
  s_device.msTicks = s_device.msTicks + 1u;
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
  if ((USART1->SR & USART_SR_RXNE) != 0u) {
    uint8_t byte = (uint8_t)(USART1->DR & 0xFFu);
    (void)CircArray_push(&s_device.usart1_rx, byte);
  }
#else
  /* The host model has no pending USART interrupt flag. RX injection is
   * performed explicitly through discobot_usart1_inject_rx(). */
  return;
#endif
}

/* ------------------------------------------------------------------------- */
/* USART public API                                          */
/* ------------------------------------------------------------------------- */

int usart1_send(const char* data, uint16_t len) {
  if (data == NULL) {
    return -1;
  }

#ifdef DISCOBOT_TARGET
  for (uint16_t i = 0u; i < len; ++i) {
    while ((USART1->SR & USART_SR_TXE) == 0u) {
      /* Wait until TX data register is empty. */
    }
    USART1->DR = (uint8_t)data[i];
  }
  return 0;
#else
  if ((uint32_t)s_device.usart1_tx_length + (uint32_t)len >
      sizeof(s_device.usart1_tx_log)) {
    return -1;
  }

  for (uint16_t i = 0u; i < len; ++i) {
    s_device.usart1_tx_log[s_device.usart1_tx_length + i] = data[i];
  }
  s_device.usart1_tx_length = (uint16_t)(s_device.usart1_tx_length + len);
  return 0;
#endif
}

int usart1_readc(void) {
  uint8_t byte = 0u;
  if (CircArray_pop(&s_device.usart1_rx, &byte) == 0) {
    return -1;
  }
  return (int)byte;
}

int usart1_read(void) {
  uint8_t byte = 0u;
  (void)CircArray_pop(&s_device.usart1_rx, &byte);
  return (int)byte;
}

uint16_t usart1_available(void) {
  return CircArray_available(&s_device.usart1_rx);
}

/* ------------------------------------------------------------------------- */
/* Sensor injection and read helpers                         */
/* ------------------------------------------------------------------------- */

uint8_t ButtonIsPressed(void) {
  return s_device.button_is_pressed;
}

void discobot_set_button_level(uint8_t level) {
  s_device.GPIOA_IDR = level != 0u ? 1u : 0u;
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
  s_device.accel_raw[0] = x;
  s_device.accel_raw[1] = y;
  s_device.accel_raw[2] = z;
}

void discobot_set_adc_raw(uint16_t raw) {
  s_device.adc1_raw = raw;
}

void discobot_set_rng(uint32_t ready, uint32_t value) {
  s_device.rng_ready = ready;
  s_device.rng_value = value;
}

void discobot_usart1_inject_rx(uint8_t byte) {
  (void)CircArray_push(&s_device.usart1_rx, byte);
}

float temperature_read(void) {
#ifdef DISCOBOT_TARGET
  uint16_t raw = (uint16_t)(ADC1->DR & 0xFFFFu);
#else
  uint16_t raw = s_device.adc1_raw;
#endif
  return ((float)raw / 4095.0f) * 3.3f -
         (0.760f / 0.0025f) + 25.0f;
}

uint32_t random_read(void) {
#ifdef DISCOBOT_TARGET
  while ((RNG->SR & RNG_SR_DRDY) == 0u) {
    /* Wait for the hardware RNG to become ready. */
  }
  return RNG->DR;
#else
  if (s_device.rng_ready != 0u) {
    return s_device.rng_value;
  }
  return 0xFFFFFFFFu; /* Non-blocking host sentinel. */
#endif
}

/* ------------------------------------------------------------------------- */
/* Motor output and scheduler                                 */
/* ------------------------------------------------------------------------- */

void motor_command_fn(uint8_t cmd) {
  if (cmd > 8u) {
    return;
  }

  static const uint8_t kMotorTruth[9][4] = {
      {0u, 0u, 0u, 0u}, /* 0: stop */
      {1u, 0u, 1u, 0u}, /* 1: forward */
      {0u, 1u, 0u, 1u}, /* 2: reverse */
      {0u, 1u, 1u, 0u}, /* 3: left */
      {1u, 0u, 0u, 1u}, /* 4: right */
      {1u, 0u, 1u, 0u}, /* 5: forward */
      {0u, 1u, 0u, 1u}, /* 6: reverse */
      {0u, 0u, 0u, 0u}, /* 7: stop */
      {0u, 0u, 0u, 0u}  /* 8: stop */
  };

#ifdef DISCOBOT_TARGET
  uint32_t pin_bits = 0u;
  if (kMotorTruth[cmd][0] != 0u) {
    pin_bits |= (1u << 1);
  }
  if (kMotorTruth[cmd][1] != 0u) {
    pin_bits |= (1u << 2);
  }
  if (kMotorTruth[cmd][2] != 0u) {
    pin_bits |= (1u << 3);
  }
  if (kMotorTruth[cmd][3] != 0u) {
    pin_bits |= (1u << 4);
  }
  GPIOA->ODR = (GPIOA->ODR & ~0x1Eu) | (uint16_t)pin_bits;
#else
  s_device.GPIOA_output[1] = kMotorTruth[cmd][0];
  s_device.GPIOA_output[2] = kMotorTruth[cmd][1];
  s_device.GPIOA_output[3] = kMotorTruth[cmd][2];
  s_device.GPIOA_output[4] = kMotorTruth[cmd][3];
#endif
}

void task_add(task_fn fn, uint32_t msinterval) {
  if (fn == NULL) {
    return;
  }

  for (uint32_t i = 0u; i < kTaskCount; ++i) {
    if (s_device.tasks[i].active == 0u) {
      s_device.tasks[i].fn = fn;
      s_device.tasks[i].msinterval = msinterval;
      s_device.tasks[i].last_called = s_device.msTicks;
      s_device.tasks[i].active = 1u;
      s_device.tasks[i].call_count = 0u;
      return;
    }
  }
}

void task_run_scheduler(void) {
  uint32_t now = s_device.msTicks;

  for (uint32_t i = 0u; i < kTaskCount; ++i) {
    TaskSlot* task = &s_device.tasks[i];
    if (task->active == 0u || task->fn == NULL) {
      continue;
    }

    uint32_t elapsed = now - task->last_called;
    if (task->msinterval == 0u || elapsed >= task->msinterval) {
      task->last_called = now;
      ++task->call_count;
      task->fn();
    }
  }
}

/* ------------------------------------------------------------------------- */
/* ARM funcs1.s host equivalents                              */
/* ------------------------------------------------------------------------- */

int func1(int a, int b) {
  return (int)((uint32_t)a + (uint32_t)b);
}

int func2(int a, int b) {
  return (int)((uint32_t)a - (uint32_t)b);
}
