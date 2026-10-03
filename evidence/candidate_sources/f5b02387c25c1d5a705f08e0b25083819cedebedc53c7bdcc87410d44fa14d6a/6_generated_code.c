/* 6_generated_code.c
 *
 * Implementation of the frozen DiscoBot API plus host-injection helpers.
 * Host mode keeps deterministic observable state. Target mode conditionally
 * compiles STM32F4xx SPL mappings when DISCOBOT_TARGET is defined.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "6_generated_code.h"

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#endif

/* ------------------------------------------------------------------------- */
/* Constants and defaults                                                    */
/* ------------------------------------------------------------------------- */

#define MOTOR_LEFT_FORWARD_PIN  1U
#define MOTOR_LEFT_BACKWARD_PIN 2U
#define MOTOR_RIGHT_FORWARD_PIN 3U
#define MOTOR_RIGHT_BACKWARD_PIN 4U

#define LED_BASE_PIN            12U
#define LED_COUNT               4U
#define BUTTON_PIN              0U

#define USART_FIFO_CAPACITY     64U
#define USART_RX_CAPACITY       USART_FIFO_CAPACITY
#define USART_TX_CAPACITY       USART_FIFO_CAPACITY
#define USART_LOG_CAPACITY      256U
#define MAX_TASKS               25U
#define BUTTON_DEBOUNCE_MS      20U
#define HOST_FLAG_COUNT         8U

#define TEMP_OFFSET_VOLTAGE     0.75f
#define TEMP_SCALE_C_PER_VOLT   100.0f

enum DiscoHostFlagId {
  kDiscoHostFlagRngDrdy = 0U,
  kDiscoHostFlagUartTxe = 1U,
  kDiscoHostFlagUartRxne = 2U
};

/* ------------------------------------------------------------------------- */
/* Internal data structures                                                  */
/* ------------------------------------------------------------------------- */

/* Complete definition for the opaque CircArray type exposed in the header. */
struct CircArray {
  uint8_t* data;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
};

typedef struct {
  TaskFunction func;
  uint32_t period_ms;
  uint32_t last_called;
  uint32_t numcalls;
  bool active;
} TimedTask;

typedef struct {
  uint8_t data[USART_FIFO_CAPACITY];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile bool overrun;
} RingBuffer;

/* ------------------------------------------------------------------------- */
/* Observable host state                                                     */
/* ------------------------------------------------------------------------- */

static uint32_t GPIOA_output[16] = {0U};
static uint32_t GPIOD_output[16] = {0U};
static uint32_t GPIOA_IDR[16] = {0U};

static uint8_t usart1_tx_log[USART_LOG_CAPACITY] = {0U};
static size_t usart1_tx_length = 0U;

static volatile uint8_t button_raw_state = 0U;
static volatile uint32_t rng_value = 0U;
static volatile bool rng_drdy = false;
static volatile bool usart_txe = true;
static volatile bool usart_rxne = false;
static volatile uint32_t msTicks = 0U;

static float injected_accel_x = 0.0f;
static float injected_accel_y = 0.0f;
static float injected_accel_z = 0.0f;
static float injected_adc_voltage = 0.0f;

static volatile bool host_flag_ready[HOST_FLAG_COUNT] = {false};

/* Button debounce state machine. */
static volatile uint8_t debounced_button_state = 0U;
static uint8_t last_raw_button_state = 0U;
static uint32_t last_button_change_ms = 0U;
static bool button_command_pending = false;
static uint8_t current_command = 0U;

/* Task scheduler state. */
static TimedTask task_slots[MAX_TASKS] = {0};
static uint32_t last_execute_ms = 0U;

/* USART software FIFOs. */
static RingBuffer usart_rx_ring = {{0U}, 0U, 0U, false};
static RingBuffer usart_tx_ring = {{0U}, 0U, 0U, false};

/* ------------------------------------------------------------------------- */
/* Internal helpers                                                          */
/* ------------------------------------------------------------------------- */

static void RingBuffer_Reset(RingBuffer* rb) {
  size_t i;

  if (rb == NULL) {
    return;
  }

  for (i = 0U; i < USART_FIFO_CAPACITY; ++i) {
    rb->data[i] = 0U;
  }
  rb->head = 0U;
  rb->tail = 0U;
  rb->overrun = false;
}

static bool RingBuffer_IsEmpty(const RingBuffer* rb) {
  if (rb == NULL) {
    return true;
  }
  return rb->head == rb->tail;
}

static bool RingBuffer_IsFull(const RingBuffer* rb) {
  if (rb == NULL) {
    return true;
  }
  return (uint16_t)((rb->tail + 1U) % USART_FIFO_CAPACITY) == rb->head;
}

static bool RingBuffer_Push(RingBuffer* rb, uint8_t value) {
  if (rb == NULL || RingBuffer_IsFull(rb)) {
    return false;
  }

  rb->data[rb->tail] = value;
  rb->tail = (uint16_t)((rb->tail + 1U) % USART_FIFO_CAPACITY);
  return true;
}

static bool RingBuffer_Pop(RingBuffer* rb, uint8_t* value) {
  if (rb == NULL || value == NULL || RingBuffer_IsEmpty(rb)) {
    return false;
  }

  *value = rb->data[rb->head];
  rb->head = (uint16_t)((rb->head + 1U) % USART_FIFO_CAPACITY);
  return true;
}

static void BoardSetMotorPin(uint8_t pin, uint8_t value) {
  if (pin >= 16U) {
    return;
  }

#ifdef DISCOBOT_TARGET
  uint16_t pin_mask = (uint16_t)(1U << pin);
  if (value != 0U) {
    GPIO_SetBits(GPIOA, pin_mask);
  } else {
    GPIO_ResetBits(GPIOA, pin_mask);
  }
#else
  GPIOA_output[pin] = (value != 0U) ? 1U : 0U;
#endif
}

static void BoardSetLedPin(uint8_t led, uint8_t value) {
  uint8_t pin;

  if (led < 1U || led > LED_COUNT) {
    return;
  }

  pin = (uint8_t)(LED_BASE_PIN + (led - 1U));
  if (pin >= 16U) {
    return;
  }

#ifdef DISCOBOT_TARGET
  uint16_t pin_mask = (uint16_t)(1U << pin);
  if (value != 0U) {
    GPIO_SetBits(GPIOD, pin_mask);
  } else {
    GPIO_ResetBits(GPIOD, pin_mask);
  }
#else
  GPIOD_output[pin] = (value != 0U) ? 1U : 0U;
#endif
}

static uint8_t BoardReadButtonRaw(void) {
#ifdef DISCOBOT_TARGET
  /* The button is connected to ground with an internal pull-up, so a pressed
   * button reads as 0 on the IDR line. Normalize it to 1 = pressed. */
  return (GPIO_ReadInputDataBit(GPIOA, (uint16_t)(1U << BUTTON_PIN)) ==
          Bit_RESET)
             ? 1U
             : 0U;
#else
  return button_raw_state != 0U ? 1U : 0U;
#endif
}

static void BoardToggleLedPin(uint8_t led) {
  uint8_t pin;

  if (led < 1U || led > LED_COUNT) {
    return;
  }

  pin = (uint8_t)(LED_BASE_PIN + (led - 1U));
  if (pin >= 16U) {
    return;
  }

#ifdef DISCOBOT_TARGET
  uint16_t pin_mask = (uint16_t)(1U << pin);
  if (GPIO_ReadOutputDataBit(GPIOD, pin_mask) != Bit_RESET) {
    GPIO_ResetBits(GPIOD, pin_mask);
  } else {
    GPIO_SetBits(GPIOD, pin_mask);
  }
#else
  GPIOD_output[pin] = (GPIOD_output[pin] != 0U) ? 0U : 1U;
#endif
}

/* ------------------------------------------------------------------------- */
/* System, motor, LED and command APIs                                       */
/* ------------------------------------------------------------------------- */

void DiscoBot_SystemInit(void) {
  size_t i;

  for (i = 0U; i < 16U; ++i) {
    GPIOA_output[i] = 0U;
    GPIOD_output[i] = 0U;
    GPIOA_IDR[i] = 0U;
  }

  usart1_tx_length = 0U;
  for (i = 0U; i < USART_LOG_CAPACITY; ++i) {
    usart1_tx_log[i] = 0U;
  }

  button_raw_state = 0U;
  debounced_button_state = 0U;
  last_raw_button_state = 0U;
  last_button_change_ms = 0U;
  button_command_pending = false;
  current_command = 0U;

  rng_value = 0U;
  rng_drdy = false;
  usart_txe = true;
  usart_rxne = false;
  msTicks = 0U;

  injected_accel_x = 0.0f;
  injected_accel_y = 0.0f;
  injected_accel_z = 0.0f;
  injected_adc_voltage = 0.0f;

  for (i = 0U; i < HOST_FLAG_COUNT; ++i) {
    host_flag_ready[i] = false;
  }

#ifdef DISCOBOT_TARGET
  SystemInit();

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef gpio_init;

  gpio_init.GPIO_Pin =
      (uint16_t)((1U << MOTOR_LEFT_FORWARD_PIN) |
                 (1U << MOTOR_LEFT_BACKWARD_PIN) |
                 (1U << MOTOR_RIGHT_FORWARD_PIN) |
                 (1U << MOTOR_RIGHT_BACKWARD_PIN));
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_init);

  gpio_init.GPIO_Pin =
      (uint16_t)((1U << 12U) | (1U << 13U) | (1U << 14U) | (1U << 15U));
  GPIO_Init(GPIOD, &gpio_init);

  gpio_init.GPIO_Pin = (uint16_t)(1U << BUTTON_PIN);
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_init);

  (void)SysTick_Config(SystemCoreClock / 1000U);
#else
  host_flag_ready[kDiscoHostFlagUartTxe] = true;
  usart_txe = true;
#endif

  DiscoBot_LEDInit();
  DiscoBot_AccelInit();
  DiscoBot_TempInit();
  DiscoBot_RNGInit();
  DiscoBot_USARTInit();
  DiscoBot_TaskInit();
}

void DiscoBot_CommandDispatch(uint8_t cmd) {
  switch (cmd) {
    case 0U:
      DiscoBot_CarStop();
      break;
    case 1U:
      DiscoBot_CarForward();
      break;
    case 2U:
      DiscoBot_CarBackward();
      break;
    case 3U:
      DiscoBot_CarLeft();
      break;
    case 4U:
      DiscoBot_CarRight();
      break;
    case 5U:
      DiscoBot_LEDOn(1U);
      DiscoBot_LEDOn(2U);
      DiscoBot_LEDOn(3U);
      DiscoBot_LEDOn(4U);
      break;
    case 6U:
      DiscoBot_LEDOff(1U);
      DiscoBot_LEDOff(2U);
      DiscoBot_LEDOff(3U);
      DiscoBot_LEDOff(4U);
      break;
    case 7U:
      DiscoBot_LEDToggle(1U);
      DiscoBot_LEDToggle(2U);
      DiscoBot_LEDToggle(3U);
      DiscoBot_LEDToggle(4U);
      break;
    case 8U:
      DiscoBot_CarStop();
      DiscoBot_LEDOff(1U);
      DiscoBot_LEDOff(2U);
      DiscoBot_LEDOff(3U);
      DiscoBot_LEDOff(4U);
      break;
    default:
      break;
  }
}

void DiscoBot_CarForward(void) {
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_FORWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_BACKWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_FORWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_BACKWARD_PIN, 0U);
}

void DiscoBot_CarBackward(void) {
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_BACKWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_BACKWARD_PIN, 1U);
}

void DiscoBot_CarLeft(void) {
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_BACKWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_FORWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_BACKWARD_PIN, 0U);
}

void DiscoBot_CarRight(void) {
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_FORWARD_PIN, 1U);
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_BACKWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_BACKWARD_PIN, 1U);
}

void DiscoBot_CarStop(void) {
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_LEFT_BACKWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_FORWARD_PIN, 0U);
  BoardSetMotorPin((uint8_t)MOTOR_RIGHT_BACKWARD_PIN, 0U);
}

void DiscoBot_LEDInit(void) {
  uint8_t led;

  for (led = 1U; led <= LED_COUNT; ++led) {
    BoardSetLedPin(led, 0U);
  }
}

void DiscoBot_LEDOn(uint8_t led) {
  BoardSetLedPin(led, 1U);
}

void DiscoBot_LEDOff(uint8_t led) {
  BoardSetLedPin(led, 0U);
}

void DiscoBot_LEDToggle(uint8_t led) {
  BoardToggleLedPin(led);
}

/* ------------------------------------------------------------------------- */
/* Button APIs                                                               */
/* ------------------------------------------------------------------------- */

uint8_t DiscoBot_ButtonGetState(void) {
  return debounced_button_state;
}

void DiscoBot_ButtonDebounce(void) {
  uint8_t raw = BoardReadButtonRaw();
  bool stable;

  if (raw != last_raw_button_state) {
    last_raw_button_state = raw;
    last_button_change_ms = msTicks;
  }

  stable = (msTicks - last_button_change_ms) >= BUTTON_DEBOUNCE_MS;
  if (!stable) {
    return;
  }

  if (raw != debounced_button_state) {
    debounced_button_state = raw;
#ifndef DISCOBOT_TARGET
    GPIOA_IDR[BUTTON_PIN] = raw != 0U ? 1U : 0U;
#endif
    if (raw != 0U) {
      button_command_pending = true;
    }
  }
}

/* ------------------------------------------------------------------------- */
/* Sensor, temperature and RNG APIs                                          */
/* ------------------------------------------------------------------------- */

void DiscoBot_AccelInit(void) {
  injected_accel_x = 0.0f;
  injected_accel_y = 0.0f;
  injected_accel_z = 0.0f;
}

void DiscoBot_AccelRead(float* x, float* y, float* z) {
  if (x == NULL || y == NULL || z == NULL) {
    return;
  }

  *x = injected_accel_x;
  *y = injected_accel_y;
  *z = injected_accel_z;
}

void DiscoBot_TempInit(void) {
  injected_adc_voltage = 0.0f;
}

float DiscoBot_TempRead(void) {
  return (injected_adc_voltage - TEMP_OFFSET_VOLTAGE) * TEMP_SCALE_C_PER_VOLT;
}

void DiscoBot_RNGInit(void) {
  rng_value = 0U;
  rng_drdy = false;
  host_flag_ready[kDiscoHostFlagRngDrdy] = false;

#ifdef DISCOBOT_TARGET
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  RNG_Cmd(ENABLE);

  NVIC_InitTypeDef rng_nvic;
  rng_nvic.NVIC_IRQChannel = RNG_IRQn;
  rng_nvic.NVIC_IRQChannelPreemptionPriority = 0U;
  rng_nvic.NVIC_IRQChannelSubPriority = 0U;
  rng_nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&rng_nvic);
#endif
}

bool DiscoBot_RNGIsReady(void) {
  return rng_drdy;
}

uint32_t DiscoBot_RNGRead(void) {
  if (!DiscoBot_RNGIsReady()) {
    return 0U;
  }
  return rng_value;
}

void DiscoBot_RNGIRQHandler(void) {
#ifdef DISCOBOT_TARGET
  if (RNG_GetFlagStatus(RNG_FLAG_DRDY) != RESET) {
    rng_value = RNG_GetRandomNumber();
    rng_drdy = true;
    host_flag_ready[kDiscoHostFlagRngDrdy] = true;
    RNG_ClearFlag(RNG_FLAG_DRDY);
  }
#else
  if (host_flag_ready[kDiscoHostFlagRngDrdy]) {
    rng_drdy = true;
  }
#endif
}

/* ------------------------------------------------------------------------- */
/* Scheduler APIs                                                            */
/* ------------------------------------------------------------------------- */

void DiscoBot_TaskInit(void) {
  size_t i;

  for (i = 0U; i < MAX_TASKS; ++i) {
    task_slots[i].func = NULL;
    task_slots[i].period_ms = 0U;
    task_slots[i].last_called = 0U;
    task_slots[i].numcalls = 0U;
    task_slots[i].active = false;
  }
  last_execute_ms = 0U;
}

bool DiscoBot_TaskAdd(TaskFunction func, uint32_t period_ms) {
  size_t i;

  if (func == NULL || period_ms == 0U) {
    return false;
  }

  for (i = 0U; i < MAX_TASKS; ++i) {
    if (!task_slots[i].active) {
      task_slots[i].func = func;
      task_slots[i].period_ms = period_ms;
      task_slots[i].last_called = msTicks;
      task_slots[i].numcalls = 0U;
      task_slots[i].active = true;
      return true;
    }
  }

  return false;
}

void DiscoBot_TaskExecute(void) {
  size_t i;

  last_execute_ms = msTicks;

  for (i = 0U; i < MAX_TASKS; ++i) {
    TimedTask* task = &task_slots[i];

    if (!task->active || task->func == NULL) {
      continue;
    }

    if ((msTicks - task->last_called) >= task->period_ms) {
      task->last_called = msTicks;
      ++task->numcalls;
      task->func();
    }
  }
}

/* ------------------------------------------------------------------------- */
/* USART APIs                                                                */
/* ------------------------------------------------------------------------- */

void DiscoBot_USARTInit(void) {
  RingBuffer_Reset(&usart_rx_ring);
  RingBuffer_Reset(&usart_tx_ring);

  usart_rxne = false;
  usart_txe = true;
  usart1_tx_length = 0U;

  host_flag_ready[kDiscoHostFlagUartTxe] = true;
  host_flag_ready[kDiscoHostFlagUartRxne] = false;

#ifdef DISCOBOT_TARGET
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_init);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  USART_InitTypeDef usart_init;
  usart_init.USART_BaudRate = 115200U;
  usart_init.USART_WordLength = USART_WordLength_8b;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &usart_init);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_Cmd(USART1, ENABLE);

  NVIC_InitTypeDef usart_nvic;
  usart_nvic.NVIC_IRQChannel = USART1_IRQn;
  usart_nvic.NVIC_IRQChannelPreemptionPriority = 0U;
  usart_nvic.NVIC_IRQChannelSubPriority = 0U;
  usart_nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&usart_nvic);
#endif
}

void DiscoBot_USARTIRQHandler(void) {
#ifdef DISCOBOT_TARGET
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    uint8_t rx_byte = (uint8_t)USART_ReceiveData(USART1);
    if (!RingBuffer_Push(&usart_rx_ring, rx_byte)) {
      usart_rx_ring.overrun = true;
    } else {
      usart_rxne = true;
      host_flag_ready[kDiscoHostFlagUartRxne] = true;
    }
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }

  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
    uint8_t tx_byte;
    if (!RingBuffer_IsEmpty(&usart_tx_ring) &&
        RingBuffer_Pop(&usart_tx_ring, &tx_byte)) {
      USART_SendData(USART1, tx_byte);
    }

    if (RingBuffer_IsEmpty(&usart_tx_ring)) {
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      usart_txe = true;
      host_flag_ready[kDiscoHostFlagUartTxe] = true;
    }
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
  }
#else
  if (usart_txe && !RingBuffer_IsEmpty(&usart_tx_ring)) {
    uint8_t tx_byte;
    if (RingBuffer_Pop(&usart_tx_ring, &tx_byte)) {
      usart_txe = false;
      host_flag_ready[kDiscoHostFlagUartTxe] = false;
    }
  }

  if (usart_rxne && RingBuffer_IsEmpty(&usart_rx_ring)) {
    usart_rxne = false;
    host_flag_ready[kDiscoHostFlagUartRxne] = false;
  }
#endif
}

bool DiscoBot_USARTReadByte(uint8_t* byte) {
  uint8_t value;

  if (byte == NULL) {
    return false;
  }

  if (usart_rx_ring.overrun) {
    usart_rx_ring.overrun = false;
  }

  if (!RingBuffer_Pop(&usart_rx_ring, &value)) {
    usart_rxne = false;
    host_flag_ready[kDiscoHostFlagUartRxne] = false;
    return false;
  }

  *byte = value;

  if (RingBuffer_IsEmpty(&usart_rx_ring)) {
    usart_rxne = false;
    host_flag_ready[kDiscoHostFlagUartRxne] = false;
  }

  return true;
}

void DiscoBot_USARTSendByte(uint8_t byte) {
  if (usart1_tx_length < USART_LOG_CAPACITY) {
    usart1_tx_log[usart1_tx_length] = byte;
    ++usart1_tx_length;
  }

#ifdef DISCOBOT_TARGET
  USART_SendData(USART1, byte);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
  }
  usart_txe = true;
#else
  if (!RingBuffer_Push(&usart_tx_ring, byte)) {
    usart_tx_ring.overrun = true;
  }

  if (usart_txe && !RingBuffer_IsEmpty(&usart_tx_ring)) {
    uint8_t outgoing_byte;
    if (RingBuffer_Pop(&usart_tx_ring, &outgoing_byte)) {
      usart_txe = false;
      host_flag_ready[kDiscoHostFlagUartTxe] = false;
    }
  }
#endif
}

void DiscoBot_USARTSendString(const char* s) {
  if (s == NULL) {
    return;
  }

  while (*s != '\0') {
    DiscoBot_USARTSendByte((uint8_t)*s);
    ++s;
  }
}

/* ------------------------------------------------------------------------- */
/* CircArray APIs                                                            */
/* ------------------------------------------------------------------------- */

bool DiscoBot_CircArrayInit(CircArray* ca, size_t capacity) {
  if (ca == NULL || capacity == 0U) {
    return false;
  }

  ca->data = (uint8_t*)malloc(capacity * sizeof(uint8_t));
  if (ca->data == NULL) {
    return false;
  }

  ca->capacity = capacity;
  ca->head = 0U;
  ca->tail = 0U;
  ca->count = 0U;
  return true;
}

bool DiscoBot_CircArrayPush(CircArray* ca, uint8_t value) {
  if (ca == NULL || ca->data == NULL || ca->count >= ca->capacity) {
    return false;
  }

  ca->data[ca->tail] = value;
  ca->tail = (ca->tail + 1U) % ca->capacity;
  ++ca->count;
  return true;
}

bool DiscoBot_CircArrayPop(CircArray* ca, uint8_t* value) {
  if (ca == NULL || ca->data == NULL || value == NULL || ca->count == 0U) {
    return false;
  }

  *value = ca->data[ca->head];
  ca->head = (ca->head + 1U) % ca->capacity;
  --ca->count;
  return true;
}

bool DiscoBot_CircArrayIsFull(const CircArray* ca) {
  if (ca == NULL) {
    return true;
  }
  return ca->count >= ca->capacity;
}

bool DiscoBot_CircArrayIsEmpty(const CircArray* ca) {
  if (ca == NULL) {
    return true;
  }
  return ca->count == 0U;
}

void DiscoBot_CircArrayFree(CircArray* ca) {
  if (ca == NULL) {
    return;
  }

  free(ca->data);
  ca->data = NULL;
  ca->capacity = 0U;
  ca->head = 0U;
  ca->tail = 0U;
  ca->count = 0U;
}

/* ------------------------------------------------------------------------- */
/* SysTick and main loop                                                     */
/* ------------------------------------------------------------------------- */

void SysTick_Handler(void) {
  ++msTicks;
}

void main_loop_iteration(void) {
  DiscoBot_ButtonDebounce();

  if (button_command_pending) {
    button_command_pending = false;
    current_command = (uint8_t)((current_command + 1U) % 9U);
    DiscoBot_CommandDispatch(current_command);
  }

  DiscoBot_TaskExecute();
}

/* ------------------------------------------------------------------------- */
/* Host-injection APIs                                                       */
/* ------------------------------------------------------------------------- */

void DiscoHost_InjectButton(uint8_t state) {
  uint8_t normalized = (state != 0U) ? 1U : 0U;
  button_raw_state = normalized;

#ifndef DISCOBOT_TARGET
  GPIOA_IDR[BUTTON_PIN] = (uint32_t)normalized;
#endif
}

void DiscoHost_InjectAccel(float x, float y, float z) {
  injected_accel_x = x;
  injected_accel_y = y;
  injected_accel_z = z;
}

void DiscoHost_InjectADC(float voltage) {
  injected_adc_voltage = voltage;
}

void DiscoHost_InjectRNG(uint32_t value, bool ready) {
  rng_value = value;
  rng_drdy = ready;
  host_flag_ready[kDiscoHostFlagRngDrdy] = ready;
}

void DiscoHost_InjectUARTByte(uint8_t byte) {
  if (RingBuffer_IsFull(&usart_rx_ring)) {
    usart_rx_ring.overrun = true;
    usart_rxne = true;
    host_flag_ready[kDiscoHostFlagUartRxne] = true;
    return;
  }

  if (!RingBuffer_Push(&usart_rx_ring, byte)) {
    usart_rx_ring.overrun = true;
    usart_rxne = true;
    host_flag_ready[kDiscoHostFlagUartRxne] = true;
    return;
  }

  usart_rxne = true;
  host_flag_ready[kDiscoHostFlagUartRxne] = true;
}

void DiscoHost_SetFlagReady(uint8_t flag_id, bool ready) {
  if (flag_id >= HOST_FLAG_COUNT) {
    return;
  }

  host_flag_ready[flag_id] = ready;

  switch (flag_id) {
    case kDiscoHostFlagRngDrdy:
      rng_drdy = ready;
      break;
    case kDiscoHostFlagUartTxe:
      usart_txe = ready;
      break;
    case kDiscoHostFlagUartRxne:
      usart_rxne = ready;
      break;
    default:
      break;
  }
}
