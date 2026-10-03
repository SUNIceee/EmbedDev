#include "6_generated_code.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef DISCO_PI
#define DISCO_PI 3.14159265358979323846
#endif

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_flash.h"
#include "misc.h"

extern int TM_LIS302DL_LIS3DSH_Init(uint8_t sensitivity, uint8_t filter);
extern void TM_LIS302DL_LIS3DSH_ReadAxes(int16_t *x, int16_t *y, int16_t *z);
#endif

/* Public observable state. */
volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = NULL;

motor_command_fn flookup[9] = {
    move_forward,
    move_backward,
    move_forward_soft_left,
    move_forward_soft_right,
    move_backward_soft_left,
    move_backward_soft_right,
    move_spin_right,
    move_spin_left,
    stop
};

uint8_t GPIOA_output[16] = {0u};
uint8_t GPIOD_output[16] = {0u};
uint32_t GPIOA_IDR = 0u;
uint32_t SystemCoreClock = 0u;
uint32_t SysTick_reload = 0u;
uint32_t usart1_baud_used = 0u;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* Private shared state. */
static CircArray msg;
static int16_t disco_accel_raw_mg[3] = {0, 0, 0};
static uint16_t disco_adc_raw = 0u;
static bool disco_adc_eoc = false;
static volatile bool disco_rng_ready = false;
static uint32_t disco_rng_value = 0u;
static uint8_t disco_usart1_dr = 0u;
static bool disco_usart1_rxne = false;
static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0u;
static volatile uint32_t systick_config_failed = 0u;

#ifndef DISCOBOT_TARGET
#define DISCO_RNG_HOST_TIMEOUT 1000000u
#endif

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void);
static int SysTick_Config(uint32_t ticks);
#endif

static void disco_set_gpioa_pin(uint32_t pin, uint8_t level)
{
    if (pin >= 16u) {
        return;
    }
    GPIOA_output[pin] = (level != 0u) ? 1u : 0u;
#ifdef DISCOBOT_TARGET
    if (level != 0u) {
        GPIOA->BSRRL = (uint32_t)(1u << pin);
    } else {
        GPIOA->BSRRH = (uint32_t)(1u << pin);
    }
#endif
}

static void disco_set_gpiod_pin(uint32_t pin, uint8_t level)
{
    if (pin >= 16u) {
        return;
    }
    GPIOD_output[pin] = (level != 0u) ? 1u : 0u;
#ifdef DISCOBOT_TARGET
    if (level != 0u) {
        GPIOD->BSRRL = (uint32_t)(1u << pin);
    } else {
        GPIOD->BSRRH = (uint32_t)(1u << pin);
    }
#endif
}

void SystemInit(void)
{
#ifdef DISCOBOT_TARGET
    RCC_DeInit();

    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET) {
    }

    FLASH_SetLatency(FLASH_Latency_5);

    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK2Config(RCC_HCLK_Div2);
    RCC_PCLK1Config(RCC_HCLK_Div4);

    /* HSE = 8 MHz, PLL_VCO = 8 / 8 * 336 = 336 MHz, SYSCLK = 336 / 2 = 168 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
    }

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != (uint32_t)RCC_SYSCLKSource_PLLCLK) {
    }

    SystemCoreClockUpdate();
#else
    SystemCoreClock = 168000000u;
#endif
}

void init_systick(void)
{
    SystemCoreClockUpdate();

    uint32_t reload = SystemCoreClock / 1000u;
    SysTick_reload = reload;

    if (SysTick_Config(reload) != 0u) {
        systick_config_failed = 1u;
        while (1) {
        }
    }
}

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void)
{
    /* Host clock is already fixed by SystemInit. */
}

static int SysTick_Config(uint32_t ticks)
{
    (void)ticks;
    SysTick_reload = ticks;
    return 0;
}
#endif

void init_system(void)
{
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send((volatile char *)discobot_startup_banner);
}

void init_GPIO_A1A2A3A4_output(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 0u);

#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);

    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 0u);
#endif
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    uint8_t pa1 = 0u;
    uint8_t pa2 = 0u;

    if (direc == FORWARD) {
        pa1 = 1u;
        pa2 = 0u;
    } else if (direc == BACKWARD) {
        pa1 = 0u;
        pa2 = 1u;
    } else {
        pa1 = 0u;
        pa2 = 0u;
    }

    disco_set_gpioa_pin(1u, pa1);
    disco_set_gpioa_pin(2u, pa2);
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    uint8_t pa3 = 0u;
    uint8_t pa4 = 0u;

    if (direc == FORWARD) {
        pa3 = 1u;
        pa4 = 0u;
    } else if (direc == BACKWARD) {
        pa3 = 0u;
        pa4 = 1u;
    } else {
        pa3 = 0u;
        pa4 = 0u;
    }

    disco_set_gpioa_pin(3u, pa3);
    disco_set_gpioa_pin(4u, pa4);
}

void move_forward(void)
{
    disco_set_gpioa_pin(1u, 1u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 1u);
    disco_set_gpioa_pin(4u, 0u);
}

void move_backward(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 1u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 1u);
}

void move_forward_soft_left(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 1u);
    disco_set_gpioa_pin(4u, 0u);
}

void move_forward_soft_right(void)
{
    disco_set_gpioa_pin(1u, 1u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 0u);
}

void move_backward_soft_left(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 1u);
}

void move_backward_soft_right(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 1u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 0u);
}

void move_spin_right(void)
{
    disco_set_gpioa_pin(1u, 1u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 1u);
}

void move_spin_left(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 1u);
    disco_set_gpioa_pin(3u, 1u);
    disco_set_gpioa_pin(4u, 0u);
}

void stop(void)
{
    disco_set_gpioa_pin(1u, 0u);
    disco_set_gpioa_pin(2u, 0u);
    disco_set_gpioa_pin(3u, 0u);
    disco_set_gpioa_pin(4u, 0u);
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> (uint32_t)i) & 1u);
#else
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
#endif
}

void SysTick_Handler(void)
{
    msTicks++;

    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250u) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0u;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void)
{
    if (laststate != buttstate) {
        laststate = buttstate;
        switch (buttstate) {
            case ButtonIsReleased:
                break;
            case ButtonIsPressed:
                break;
            default:
                break;
        }
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    if (pitch == NULL || roll == NULL) {
        return;
    }

    double y = (double)acc_y;
    double z = (double)acc_z;
    double x = (double)acc_x;

    *roll = (float)(atan2(y, z) * 180.0 / DISCO_PI);
    *pitch = (float)(atan2(-x, sqrt(y * y + z * z)) * 180.0 / DISCO_PI);
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
#endif

    GPIOA_IDR &= ~1u;
    b_i = 0u;
    buttstate = ButtonIsReleased;
}

void init_LED_pins(void)
{
    disco_set_gpiod_pin(12u, 0u);
    disco_set_gpiod_pin(13u, 0u);
    disco_set_gpiod_pin(14u, 0u);
    disco_set_gpiod_pin(15u, 0u);

#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &gpio);

    disco_set_gpiod_pin(12u, 0u);
    disco_set_gpiod_pin(13u, 0u);
    disco_set_gpiod_pin(14u, 0u);
    disco_set_gpiod_pin(15u, 0u);
#endif
}

void LED_On(int i)
{
    if (i >= 0 && i <= 3) {
        disco_set_gpiod_pin((uint32_t)(12 + i), 1u);
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i <= 3) {
        disco_set_gpiod_pin((uint32_t)(12 + i), 0u);
    }
}

int init_accelerometers(void)
{
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }

#ifdef DISCOBOT_TARGET
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    TM_LIS302DL_LIS3DSH_ReadAxes(&x, &y, &z);
    disco_accel_raw_mg[0] = x;
    disco_accel_raw_mg[1] = y;
    disco_accel_raw_mg[2] = z;
#endif

    b[0] = (float)disco_accel_raw_mg[0] / 1000.0f;
    b[1] = (float)disco_accel_raw_mg[1] / 1000.0f;
    b[2] = (float)disco_accel_raw_mg[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    disco_accel_raw_mg[0] = x_mg;
    disco_accel_raw_mg[1] = y_mg;
    disco_accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    ADC_CommonInitTypeDef adc_common;
    ADC_InitTypeDef adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);

    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_Cmd(ADC1, ENABLE);
#else
    disco_adc_eoc = false;
#endif
}

float read_temperature_sensor(void)
{
    uint16_t raw = 0u;

#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }
    raw = (uint16_t)ADC_GetConversionValue(ADC1);
#else
    if (!disco_adc_eoc) {
        /* Not-ready host sentinel: target would block until EOC. */
        return (float)NAN;
    }
    raw = disco_adc_raw;
    disco_adc_eoc = false;
#endif

    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw)
{
    disco_adc_raw = raw;
    disco_adc_eoc = true;
}

void init_rng(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#else
    disco_rng_ready = false;
    disco_rng_value = 0u;
#endif
}

uint32_t get_random_number(void)
{
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }
    return RNG_GetRandomNumber();
#else
    /* Host-only bounded wait. Preserves the DRDY not-ready blocking condition
     * without hanging a host process indefinitely if a test calls before
     * discobot_set_rng(true, value). Same injected ready/value is deterministic.
     */
    uint32_t timeout_ticks = 0u;
    while (!disco_rng_ready && (timeout_ticks < DISCO_RNG_HOST_TIMEOUT)) {
        timeout_ticks++;
    }
    return disco_rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value)
{
    disco_rng_ready = ready;
    disco_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL || size <= 0) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "CircArray already initialized\n");
        return;
    }

    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) {
        return;
    }

    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->size == 0u) {
        return 0;
    }

    if (buf_full(arr)) {
        return 0;
    }

    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->size == 0u) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr)
{
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || arr->size == 0u) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr)
{
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize)
{
    if (arr == NULL || newSize <= 0) {
        return false;
    }

    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (new_buf == NULL) {
        return false;
    }

    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr)
{
    if (arr == NULL) {
        return false;
    }

    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud)
{
    int effective_baud = baud;
    if (effective_baud == 0) {
        effective_baud = 9600;
        fprintf(stderr, "Warning: baud was 0, using 9600\n");
    }

    usart1_baud_used = (uint32_t)effective_baud;

#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    usart.USART_BaudRate = (uint32_t)effective_baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
#endif

    initCircArray(&msg, 200);

#ifdef DISCOBOT_TARGET
    USART_Cmd(USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = (char)USART1->DR;
        (void)buf_putbyte(&msg, c);
    }
#else
    if (disco_usart1_rxne) {
        char c = (char)disco_usart1_dr;
        disco_usart1_rxne = false;
        (void)buf_putbyte(&msg, c);
    }
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        char c = *s;
#ifdef DISCOBOT_TARGET
        while (!(USART1->SR & 0x40u)) {
        }
        USART_SendData(USART1, (uint16_t)c);
#else
        (void)c;
#endif

        if (usart1_tx_length < (sizeof(usart1_tx_log) - 1u)) {
            usart1_tx_log[usart1_tx_length] = c;
            usart1_tx_length++;
            usart1_tx_log[usart1_tx_length] = '\0';
        }

        s++;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
#ifdef DISCOBOT_TARGET
    (void)value;
    (void)rxne;
#else
    disco_usart1_dr = value;
    disco_usart1_rxne = rxne;
    if (rxne) {
        USART1_IRQHandler();
    }
#endif
}

bool dispatch_uart_command(int command)
{
    if (command >= 0 && command <= 8) {
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void)
{
    if ((msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }

    if (usart1_available() > 0u) {
        char c = usart1_readc();
        (void)dispatch_uart_command((int)c);
    }

    checkbutton();
    execute_tasks();
}

#ifndef DISCOBOT_TARGET
int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}
#endif