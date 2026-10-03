/*
 * discobot_target_spl.c
 *
 * STM32F407 target hardware adaptation layer.
 *
 * This translation unit is active only when DISCOBOT_TARGET is defined.
 * The public API remains implemented by discobot_core.c; this file exposes
 * the internal hardware operations consumed by the core module.
 */

#if defined(DISCOBOT_TARGET)

#include "6_generated_code.h"
#include "discobot_internal.h"

#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_usart.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef DISCOBOT_TARGET_CORE_CLOCK_HZ
#define DISCOBOT_TARGET_CORE_CLOCK_HZ 168000000UL
#endif

#ifndef DISCOBOT_TARGET_USART_BAUDRATE
#define DISCOBOT_TARGET_USART_BAUDRATE 115200UL
#endif

/*
 * STM32F4-Discovery commonly uses PA0 as the user button. Avoid configuring
 * PA0 as analog by default. PB0 / ADC channel 8 is a safe default that does
 * not conflict with PA0 button, PA1-PA4 motor pins, PA5-PA7 SPI1, PA9/PA10
 * USART1, or PD12-PD15 LEDs.
 *
 * Target projects may override all four ADC macros below to match their board
 * schematic and frozen API contract.
 */
#ifndef DISCOBOT_TARGET_ADC_CHANNEL
#define DISCOBOT_TARGET_ADC_CHANNEL ADC_Channel_8
#endif

#ifndef DISCOBOT_TARGET_ADC_GPIO_PORT
#define DISCOBOT_TARGET_ADC_GPIO_PORT GPIOB
#endif

#ifndef DISCOBOT_TARGET_ADC_GPIO_PIN
#define DISCOBOT_TARGET_ADC_GPIO_PIN GPIO_Pin_0
#endif

#ifndef DISCOBOT_TARGET_ADC_GPIO_CLOCK
#define DISCOBOT_TARGET_ADC_GPIO_CLOCK RCC_AHB1Periph_GPIOB
#endif

#define MOTOR_PIN_MASK (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4)
#define LED_PIN_MASK \
  (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)

#define BUTTON_PIN GPIO_Pin_0

#define LIS3DSH_SPI SPI1
#define LIS3DSH_CS_PORT GPIOE
#define LIS3DSH_CS_PIN GPIO_Pin_3

#define LIS3DSH_REG_WHO_AM_I 0x0FU
#define LIS3DSH_REG_CTRL_REG4 0x20U
#define LIS3DSH_REG_CTRL_REG5 0x24U
#define LIS3DSH_REG_OUT_X_L 0x28U

#define LIS3DSH_SPI_TIMEOUT 1000000UL

static uint32_t gpioa_output_shadow;
static uint32_t gpiod_output_shadow;
static uint32_t systick_reload_value;

static void enable_gpio_clocks(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
                             RCC_AHB1Periph_GPIOD |
                             RCC_AHB1Periph_GPIOE |
                             DISCOBOT_TARGET_ADC_GPIO_CLOCK,
                         ENABLE);
}

static void configure_motor_gpio(void) {
  GPIO_InitTypeDef gpio_init;

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = MOTOR_PIN_MASK;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &gpio_init);
  GPIO_ResetBits(GPIOA, MOTOR_PIN_MASK);
  gpioa_output_shadow = 0U;
}

static void configure_led_gpio(void) {
  GPIO_InitTypeDef gpio_init;

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = LED_PIN_MASK;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOD, &gpio_init);
  GPIO_ResetBits(GPIOD, LED_PIN_MASK);
  gpiod_output_shadow = 0U;
}

static void configure_button_gpio(void) {
  GPIO_InitTypeDef gpio_init;

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = BUTTON_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
  gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpio_init.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_Init(GPIOA, &gpio_init);
}

static void configure_adc_gpio(void) {
  GPIO_InitTypeDef gpio_init;

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = DISCOBOT_TARGET_ADC_GPIO_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(DISCOBOT_TARGET_ADC_GPIO_PORT, &gpio_init);
}

static void configure_lis3dsh_gpio(void) {
  GPIO_InitTypeDef gpio_init;

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &gpio_init);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = LIS3DSH_CS_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LIS3DSH_CS_PORT, &gpio_init);
  GPIO_SetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
}

static void configure_lis3dsh_spi(void) {
  SPI_InitTypeDef spi_init;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_StructInit(&spi_init);
  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_CPOL = SPI_CPOL_High;
  spi_init.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init.SPI_CRCPolynomial = 7U;

  SPI_Init(LIS3DSH_SPI, &spi_init);
  SPI_Cmd(LIS3DSH_SPI, ENABLE);
}

static uint8_t lis3dsh_transfer(uint8_t value) {
  uint32_t timeout = LIS3DSH_SPI_TIMEOUT;

  while (SPI_I2S_GetFlagStatus(LIS3DSH_SPI, SPI_I2S_FLAG_TXE) == RESET) {
    if (timeout == 0U) {
      return 0U;
    }
    --timeout;
  }

  SPI_I2S_SendData(LIS3DSH_SPI, value);

  timeout = LIS3DSH_SPI_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(LIS3DSH_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
    if (timeout == 0U) {
      return 0U;
    }
    --timeout;
  }

  return (uint8_t)SPI_I2S_ReceiveData(LIS3DSH_SPI);
}

static void lis3dsh_write_register(uint8_t address, uint8_t value) {
  GPIO_ResetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
  (void)lis3dsh_transfer(address & 0x7FU);
  (void)lis3dsh_transfer(value);
  GPIO_SetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
}

static uint8_t lis3dsh_read_register(uint8_t address) {
  uint8_t value;

  GPIO_ResetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
  (void)lis3dsh_transfer(address | 0x80U);
  value = lis3dsh_transfer(0U);
  GPIO_SetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);

  return value;
}

static void lis3dsh_read_registers(uint8_t address,
                                   uint8_t *buffer,
                                   size_t length) {
  size_t index;

  if (buffer == NULL) {
    return;
  }

  GPIO_ResetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
  (void)lis3dsh_transfer(address | 0xC0U);

  for (index = 0U; index < length; ++index) {
    buffer[index] = lis3dsh_transfer(0U);
  }

  GPIO_SetBits(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN);
}

static bool is_internal_adc_channel(uint8_t channel) {
  return (channel == ADC_Channel_16) || (channel == ADC_Channel_17);
}

void disco_hw_gpio_init(void) {
  enable_gpio_clocks();
  configure_motor_gpio();
  configure_led_gpio();
  configure_button_gpio();
  configure_adc_gpio();
}

void disco_hw_gpioa_write(uint32_t value) {
  uint32_t set_bits;
  uint32_t reset_bits;

  value &= MOTOR_PIN_MASK;
  set_bits = value & MOTOR_PIN_MASK;
  reset_bits = (~value) & MOTOR_PIN_MASK;

  if (set_bits != 0U) {
    GPIO_SetBits(GPIOA, (uint16_t)set_bits);
  }

  if (reset_bits != 0U) {
    GPIO_ResetBits(GPIOA, (uint16_t)reset_bits);
  }

  gpioa_output_shadow = (gpioa_output_shadow & ~MOTOR_PIN_MASK) | value;
}

uint32_t disco_hw_gpioa_read(void) {
  return GPIOA->IDR;
}

void disco_hw_gpiod_write(uint32_t value) {
  uint32_t set_bits;
  uint32_t reset_bits;

  value &= LED_PIN_MASK;
  set_bits = value & LED_PIN_MASK;
  reset_bits = (~value) & LED_PIN_MASK;

  if (set_bits != 0U) {
    GPIO_SetBits(GPIOD, (uint16_t)set_bits);
  }

  if (reset_bits != 0U) {
    GPIO_ResetBits(GPIOD, (uint16_t)reset_bits);
  }

  gpiod_output_shadow = (gpiod_output_shadow & ~LED_PIN_MASK) | value;
}

uint32_t disco_hw_gpiod_read(void) {
  return gpiod_output_shadow;
}

uint32_t disco_hw_button_read(void) {
  return (GPIO_ReadInputDataBit(GPIOA, BUTTON_PIN) != Bit_RESET) ? 1U : 0U;
}

void disco_hw_systick_config(void) {
  systick_reload_value = DISCOBOT_TARGET_CORE_CLOCK_HZ / 1000UL;

  if (systick_reload_value == 0U) {
    return;
  }

  (void)SysTick_Config(systick_reload_value);
}

uint32_t disco_hw_systick_reload(void) {
  return systick_reload_value;
}

void disco_hw_usart_init(void) {
  GPIO_InitTypeDef gpio_init;
  USART_InitTypeDef usart_init;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_StructInit(&gpio_init);
  gpio_init.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &gpio_init);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

  USART_StructInit(&usart_init);
  usart_init.USART_BaudRate = DISCOBOT_TARGET_USART_BAUDRATE;
  usart_init.USART_WordLength = USART_WordLength_8b;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &usart_init);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);

  NVIC_EnableIRQ(USART1_IRQn);
}

bool disco_hw_usart_rx_available(void) {
  return USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET;
}

uint8_t disco_hw_usart_read_byte(void) {
  return (uint8_t)USART_ReceiveData(USART1);
}

void disco_hw_usart_send_byte(uint8_t value) {
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
  }

  USART_SendData(USART1, value);
}

void disco_hw_adc_init(void) {
  ADC_CommonInitTypeDef adc_common_init;
  ADC_InitTypeDef adc_init;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_CommonStructInit(&adc_common_init);
  adc_common_init.ADC_Mode = ADC_Mode_Independent;
  adc_common_init.ADC_Prescaler = ADC_Prescaler_Div4;
  adc_common_init.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  adc_common_init.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&adc_common_init);

  ADC_StructInit(&adc_init);
  adc_init.ADC_Resolution = ADC_Resolution_12b;
  adc_init.ADC_ScanConvMode = DISABLE;
  adc_init.ADC_ContinuousConvMode = DISABLE;
  adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init.ADC_NbrOfConversion = 1U;

  ADC_Init(ADC1, &adc_init);

  if (is_internal_adc_channel((uint8_t)DISCOBOT_TARGET_ADC_CHANNEL)) {
    ADC_TempSensorVrefintCmd(ENABLE);
  }

  ADC_Cmd(ADC1, ENABLE);
}

uint16_t disco_hw_adc_read(uint8_t channel) {
  uint8_t selected_channel = channel;

  if (selected_channel > ADC_Channel_18) {
    selected_channel = (uint8_t)DISCOBOT_TARGET_ADC_CHANNEL;
  }

  if (is_internal_adc_channel(selected_channel)) {
    ADC_TempSensorVrefintCmd(ENABLE);
  }

  ADC_RegularChannelConfig(ADC1,
                           selected_channel,
                           1U,
                           ADC_SampleTime_144Cycles);

  ADC_SoftwareStartConv(ADC1);

  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
  }

  return ADC_GetConversionValue(ADC1);
}

void disco_hw_lis3dsh_init(void) {
  enable_gpio_clocks();
  configure_lis3dsh_gpio();
  configure_lis3dsh_spi();

  (void)lis3dsh_read_register(LIS3DSH_REG_WHO_AM_I);

  lis3dsh_write_register(LIS3DSH_REG_CTRL_REG4, 0x67U);
  lis3dsh_write_register(LIS3DSH_REG_CTRL_REG5, 0x00U);
}

void disco_hw_lis3dsh_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
  uint8_t raw_data[6];

  if ((x == NULL) || (y == NULL) || (z == NULL)) {
    return;
  }

  lis3dsh_read_registers(LIS3DSH_REG_OUT_X_L, raw_data, sizeof(raw_data));

  *x = (int16_t)(((uint16_t)raw_data[1] << 8U) | raw_data[0]);
  *y = (int16_t)(((uint16_t)raw_data[3] << 8U) | raw_data[2]);
  *z = (int16_t)(((uint16_t)raw_data[5] << 8U) | raw_data[4]);
}

void disco_hw_rng_init(void) {
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
  RNG_Cmd(ENABLE);
}

uint32_t disco_hw_rng_read(void) {
  while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
  }

  return RNG_GetRandomNumber();
}

#endif  /* DISCOBOT_TARGET */
