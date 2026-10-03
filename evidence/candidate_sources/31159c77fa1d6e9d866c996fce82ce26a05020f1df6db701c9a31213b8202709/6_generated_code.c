#include "6_generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_misc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
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

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = UART_BAUD;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {{0}};
CircArray msg = {0};

int16_t discobot_accel_raw_x_mg = 0;
int16_t discobot_accel_raw_y_mg = 0;
int16_t discobot_accel_raw_z_mg = 0;
uint16_t discobot_adc_raw = 0;
volatile bool discobot_rng_ready = false;
volatile uint32_t discobot_rng_value = 0;
bool discobot_usart1_rxne = false;
uint8_t discobot_usart1_rx_value = 0;

bool accelerometer_initialized_state = false;
bool adc_initialized_state = false;
bool rng_enabled_state = false;

static ButtonState last_button_state = ButtonIsReleased;
static uint32_t last_1s_idle_tick = 0;

#ifndef DISCOBOT_TARGET

typedef enum { RESET = 0, SET = 1 } FlagStatus;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

typedef struct { volatile uint32_t SR; volatile uint32_t DR; } USART_TypeDef;
static USART_TypeDef host_usart1;
#define USART1 (&host_usart1)
#define USART_IT_RXNE 0x20u

static FlagStatus USART_GetITStatus(USART_TypeDef *uart, uint32_t flag)
{
    (void)uart;
    return (host_usart1.SR & flag) ? SET : RESET;
}

static void USART_SendData(USART_TypeDef *uart, uint16_t data)
{
    (void)uart;
    (void)data;
}

typedef struct { volatile uint32_t SR; volatile uint32_t DR; } ADC_TypeDef;
static ADC_TypeDef host_adc1;
#define ADC1 (&host_adc1)
#define ADC_FLAG_EOC 0x1u

static void ADC_SoftwareStartConv(ADC_TypeDef *adc)
{
    (void)adc;
}

static FlagStatus ADC_GetFlagStatus(ADC_TypeDef *adc, uint32_t flag)
{
    (void)adc;
    (void)flag;
    return SET;
}

static uint16_t ADC_GetConversionValue(ADC_TypeDef *adc)
{
    (void)adc;
    return discobot_adc_raw;
}

#define RNG_FLAG_DRDY 0x1u
#define RCC_AHB2Periph_RNG 0x00000001u

static void RCC_AHB2PeriphClockCmd(uint32_t periph, FunctionalState state)
{
    (void)periph;
    (void)state;
}

static void RNG_Cmd(FunctionalState state)
{
    (void)state;
}

static FlagStatus RNG_GetFlagStatus(uint32_t flag)
{
    (void)flag;
    return discobot_rng_ready ? SET : RESET;
}

static uint32_t RNG_GetRandomNumber(void)
{
    return discobot_rng_value;
}

#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 1

static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter)
{
    (void)sensitivity;
    (void)filter;
    accelerometer_initialized_state = true;
    return 0;
}

__attribute__((weak)) uint32_t SysTick_Config(uint32_t ticks)
{
    SysTick_reload = ticks;
    return 0;
}

static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000u;
}

#endif

static void write_gpioa_pin(uint32_t pin, uint8_t value)
{
    if (pin >= 16u) {
        return;
    }
    GPIOA_output[pin] = value & 1u;
#ifdef DISCOBOT_TARGET
    if (GPIOA != NULL) {
        if (value & 1u) {
            GPIOA->BSRRL = (1u << pin);
        } else {
            GPIOA->BSRRH = (1u << pin);
        }
    }
#else
    (void)value;
#endif
}

static void write_gpiod_pin(uint32_t pin, uint8_t value)
{
    if (pin >= 16u) {
        return;
    }
    GPIOD_output[pin] = value & 1u;
#ifdef DISCOBOT_TARGET
    if (GPIOD != NULL) {
        if (value & 1u) {
            GPIOD->BSRRL = (1u << pin);
        } else {
            GPIOD->BSRRH = (1u << pin);
        }
    }
#else
    (void)value;
#endif
}

void SystemInit(void)
{
#ifdef DISCOBOT_TARGET
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    if (RCC_WaitForHSEStartUp() == SUCCESS) {
        FLASH_SetLatency(FLASH_Latency_5);
        FLASH_PrefetchBufferCmd(ENABLE);
        FLASH_InstructionCacheCmd(ENABLE);
        FLASH_DataCacheCmd(ENABLE);

        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div4);
        RCC_PCLK2Config(RCC_HCLK_Div2);
        RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08) {}
    }
#endif
    SystemCoreClock = 168000000u;
}

void init_systick(void)
{
    SystemCoreClockUpdate();
    uint32_t ticks = SystemCoreClock / 1000u;
    if (SysTick_Config(ticks)) {
        while (1) {
        }
    }
    SysTick_reload = ticks;
}

void init_GPIO_A1A2A3A4_output(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
#endif

    write_gpioa_pin(1, 0);
    write_gpioa_pin(2, 0);
    write_gpioa_pin(3, 0);
    write_gpioa_pin(4, 0);
}

void init_LED_pins(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &gpio);
#endif

    write_gpiod_pin(12, 0);
    write_gpiod_pin(13, 0);
    write_gpiod_pin(14, 0);
    write_gpiod_pin(15, 0);
}

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);
#endif

    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

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
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        write_gpioa_pin(1, 1);
        write_gpioa_pin(2, 0);
        break;
    case BACKWARD:
        write_gpioa_pin(1, 0);
        write_gpioa_pin(2, 1);
        break;
    case STOP:
    default:
        write_gpioa_pin(1, 0);
        write_gpioa_pin(2, 0);
        break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        write_gpioa_pin(3, 1);
        write_gpioa_pin(4, 0);
        break;
    case BACKWARD:
        write_gpioa_pin(3, 0);
        write_gpioa_pin(4, 1);
        break;
    case STOP:
    default:
        write_gpioa_pin(3, 0);
        write_gpioa_pin(4, 0);
        break;
    }
}

void move_forward(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_backward(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_forward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_forward_soft_right(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_backward_soft_right(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_spin_left(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void stop(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
#ifdef DISCOBOT_TARGET
    if (GPIOA != NULL) {
        return (int)((GPIOA->IDR >> (uint32_t)i) & 1u);
    }
#endif
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
}

void SysTick_Handler(void)
{
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void)
{
    if (last_button_state != buttstate) {
        switch (buttstate) {
        case ButtonIsPressed:
            break;
        case ButtonIsReleased:
            break;
        default:
            break;
        }
        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    const float pi = 3.14159265358979323846f;
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
    }
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    }
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void LED_On(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }
    write_gpiod_pin((uint32_t)(12 + i), 1);
}

void LED_Off(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }
    write_gpiod_pin((uint32_t)(12 + i), 0);
}

int init_accelerometers(void)
{
    int result = TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
    accelerometer_initialized_state = (result == 0);
    return result;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }
#ifdef DISCOBOT_TARGET
    {
        TM_LIS302DL_LIS3DSH_t raw = TM_LIS302DL_LIS3DSH_ReadAxes();
        b[0] = raw.X / 1000.0f;
        b[1] = raw.Y / 1000.0f;
        b[2] = raw.Z / 1000.0f;
    }
#else
    b[0] = discobot_accel_raw_x_mg / 1000.0f;
    b[1] = discobot_accel_raw_y_mg / 1000.0f;
    b[2] = discobot_accel_raw_z_mg / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    discobot_accel_raw_x_mg = x_mg;
    discobot_accel_raw_y_mg = y_mg;
    discobot_accel_raw_z_mg = z_mg;
}

void init_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_CommonInitTypeDef adc_common;
    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInit(&adc_common);

    ADC_InitTypeDef adc;
    ADC_StructInit(&adc);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#endif
    adc_initialized_state = true;
}

float read_temperature_sensor(void)
{
    float temp;
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }
    temp = (float)ADC_GetConversionValue(ADC1);
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw)
{
    discobot_adc_raw = raw;
}

void init_rng(void)
{
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
    rng_enabled_state = true;
}

uint32_t get_random_number(void)
{
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }
    return RNG_GetRandomNumber();
}

void discobot_set_rng(bool ready, uint32_t value)
{
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    long msinterval;
    int i;
    if (myfunc == NULL) {
        return;
    }
    msinterval = (long)(interval_sec * 1000.0f);
    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)msinterval;
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            break;
        }
    }
}

void execute_tasks(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; ++i) {
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
    int i;
    for (i = 0; i < MAXNUMTASKS; ++i) {
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
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == NULL) {
        fprintf(stderr, "initCircArray: allocation failed\n");
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (!buf_empty(arr)) {
        char c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
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
    if (arr == NULL || arr->size == 0) {
        return false;
    }
    if (buf_empty(arr)) {
        return false;
    }
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
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
    char *newbuf;
    if (arr == NULL || newSize <= 0) {
        return false;
    }
    newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) {
        return false;
    }
    arr->buf = newbuf;
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
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
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
    if (baud == 0) {
        baud = 9600;
        fprintf(stderr, "warning: baud 0, using 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;

#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
    usart.USART_BaudRate = (uint32_t)baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
#else
    host_usart1.SR = 0x40u;
    host_usart1.DR = 0;
#endif

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

#ifdef DISCOBOT_TARGET
    USART_Cmd(USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART1->DR;
        buf_putbyte(&msg, (char)c);
    }
#else
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = (char)USART1->DR;
        host_usart1.SR &= ~USART_IT_RXNE;
        buf_putbyte(&msg, c);
    }
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        char c = (char)*s;
        while (!(USART1->SR & 0x40u)) {
        }
        USART_SendData(USART1, (uint16_t)(unsigned char)c);
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = c;
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
    discobot_usart1_rxne = rxne;
    discobot_usart1_rx_value = value;
#ifdef DISCOBOT_TARGET
    (void)value;
    (void)rxne;
#else
    host_usart1.DR = value;
    if (rxne) {
        host_usart1.SR |= USART_IT_RXNE;
    } else {
        host_usart1.SR &= ~USART_IT_RXNE;
    }
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
    if (usart1_available() > 0) {
        int command = (int)usart1_readc();
        dispatch_uart_command(command);
    }
    checkbutton();
    execute_tasks();
    if ((msTicks - last_1s_idle_tick) > 1000u) {
        last_1s_idle_tick = msTicks;
    }
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