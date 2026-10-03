#include "6_generated_code.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef DISCOBOT_TARGET
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rng.h>
#include <misc.h>
#include <tm_stm32f4_lis302dl_lis3dsh.h>
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
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

static CircArray usart_rx;
static bool usart_rxne = false;
static uint8_t usart_dr = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;
static int16_t accel_raw[3] = {0, 0, 0};
static uint16_t adc_raw = 0;
static ButtonState button_laststate = ButtonIsReleased;

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void);
static uint32_t SysTick_Config(uint32_t ticks);
#endif

static void write_GPIOA_pin(uint32_t pin_index, uint32_t high) {
    if (pin_index > 15U) return;
    GPIOA_output[pin_index] = high ? 1U : 0U;
#ifdef DISCOBOT_TARGET
    uint16_t pin_mask = (uint16_t)(1U << pin_index);
    if (high) {
        GPIO_SetBits(GPIOA, pin_mask);
    } else {
        GPIO_ResetBits(GPIOA, pin_mask);
    }
#endif
}

static void write_GPIOD_pin(uint32_t pin_index, uint32_t high) {
    if (pin_index > 15U) return;
    GPIOD_output[pin_index] = high ? 1U : 0U;
#ifdef DISCOBOT_TARGET
    uint16_t pin_mask = (uint16_t)(1U << pin_index);
    if (high) {
        GPIO_SetBits(GPIOD, pin_mask);
    } else {
        GPIO_ResetBits(GPIOD, pin_mask);
    }
#endif
}

#ifndef DISCOBOT_TARGET
void SystemInit(void) {
    SystemCoreClock = 168000000U;
}
static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000U;
}
static uint32_t SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0U;
}
#endif

void init_systick(void) {
#ifdef DISCOBOT_TARGET
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        while (1) {}
    }
#else
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U)) {
        for (;;) {}
    }
#endif
}

void init_GPIO_A1A2A3A4_output(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, gpio.GPIO_Pin);
#endif
    write_GPIOA_pin(1U, 0U);
    write_GPIOA_pin(2U, 0U);
    write_GPIOA_pin(3U, 0U);
    write_GPIOA_pin(4U, 0U);
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            write_GPIOA_pin(1U, 1U);
            write_GPIOA_pin(2U, 0U);
            break;
        case BACKWARD:
            write_GPIOA_pin(1U, 0U);
            write_GPIOA_pin(2U, 1U);
            break;
        case STOP:
        default:
            write_GPIOA_pin(1U, 0U);
            write_GPIOA_pin(2U, 0U);
            break;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            write_GPIOA_pin(3U, 1U);
            write_GPIOA_pin(4U, 0U);
            break;
        case BACKWARD:
            write_GPIOA_pin(3U, 0U);
            write_GPIOA_pin(4U, 1U);
            break;
        case STOP:
        default:
            write_GPIOA_pin(3U, 0U);
            write_GPIOA_pin(4U, 0U);
            break;
    }
}

void move_forward(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_backward(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_forward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_forward_soft_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_backward_soft_right(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_spin_left(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void stop(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void init_button(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
#endif
    GPIOA_IDR &= ~1U;
    b_i = 0;
    buttstate = ButtonIsReleased;
    button_laststate = ButtonIsReleased;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) return -1;
#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> (uint32_t)i) & 1U);
#else
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1U);
#endif
}

void SysTick_Handler(void) {
    ++msTicks;
    if (read_buttonc(0) == 1) {
        ++b_i;
        if (b_i >= 250U) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (button_laststate == buttstate) return;
    switch (buttstate) {
        case ButtonIsPressed:
            break;
        case ButtonIsReleased:
            break;
        default:
            break;
    }
    button_laststate = buttstate;
}

void discobot_set_button_level(bool high) {
    if (high) GPIOA_IDR |= 1U;
    else GPIOA_IDR &= ~1U;
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (pitch == NULL || roll == NULL) return;
    *roll = atan2f(acc_y, acc_z) * 180.0f / (float)M_PI;
    *pitch = atan2f(-acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z))) * 180.0f / (float)M_PI;
}

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &gpio);
    GPIO_ResetBits(GPIOD, gpio.GPIO_Pin);
#endif
    write_GPIOD_pin(12U, 0U);
    write_GPIOD_pin(13U, 0U);
    write_GPIOD_pin(14U, 0U);
    write_GPIOD_pin(15U, 0U);
}

void LED_On(int i) {
    if (i < 0 || i > 3) return;
    write_GPIOD_pin((uint32_t)(12 + i), 1U);
}

void LED_Off(int i) {
    if (i < 0 || i > 3) return;
    write_GPIOD_pin((uint32_t)(12 + i), 0U);
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#endif
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b == NULL) return;
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_Axes_t axes = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = (float)axes.X / 1000.0f;
    b[1] = (float)axes.Y / 1000.0f;
    b[2] = (float)axes.Z / 1000.0f;
#else
    b[0] = (float)accel_raw[0] / 1000.0f;
    b[1] = (float)accel_raw[1] / 1000.0f;
    b[2] = (float)accel_raw[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw[0] = x_mg;
    accel_raw[1] = y_mg;
    accel_raw[2] = z_mg;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonInitTypeDef adc_common;
    ADC_InitTypeDef adc;
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_Cmd(ADC1, ENABLE);
#endif
}

float read_temperature_sensor(void) {
    uint16_t raw;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    raw = (uint16_t)ADC_GetConversionValue(ADC1);
#else
    raw = adc_raw;
#endif
    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw = raw;
}

void init_rng(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {}
    return RNG_GetRandomNumber();
#else
    while (!rng_ready) {}
    return rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (uint32_t i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0U;
            timed_tasks[i].numcalls = 0U;
            return;
        }
    }
}

void execute_tasks(void) {
    for (uint32_t i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL) {
            if (msTicks >= (t->last_called + t->msinterval)) {
                t->task();
                t->last_called = msTicks;
                t->numcalls++;
            }
        }
    }
}

void printtimes(void) {
    for (uint32_t i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            printf("t%u=%d", (unsigned)i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) return;
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }
    char *buf = (char *)calloc((size_t)size, 1U);
    if (buf == NULL) {
        arr->buf = NULL;
        arr->size = 0U;
        arr->enabled = false;
        arr->n_r = 0U;
        arr->n_w = 0U;
        return;
    }
    arr->buf = buf;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0U;
    arr->n_w = 0U;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0U || arr->buf == NULL) return 0;
    if (buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->size == 0U || arr->buf == NULL) return 0;
    if (buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0U) return false;
    if (buf_empty(arr)) return false;
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) {
    if (arr == NULL) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) return false;
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) return false;
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0U;
    arr->enabled = false;
    arr->n_r = 0U;
    arr->n_w = 0U;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) return;
    memset(arr->buf, 0, arr->size);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    usart_dr = value;
    usart_rxne = rxne;
    USART1_IRQHandler();
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(USART1);
        (void)buf_putbyte(&usart_rx, (char)c);
    }
#else
    if (usart_rxne) {
        (void)buf_putbyte(&usart_rx, (char)usart_dr);
        usart_rxne = false;
    }
#endif
}

void init_usart1(int baud) {
    int actual_baud = baud;
    if (actual_baud == 0) {
        actual_baud = 9600;
        fprintf(stderr, "init_usart1: baud 0, using 9600\n");
    }
    usart1_baud_used = (uint32_t)actual_baud;
    initCircArray(&usart_rx, CIRC_BUFFER_MIN_SIZE);
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    USART_InitTypeDef usart;
    usart.USART_BaudRate = (uint32_t)actual_baud;
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
    USART_Cmd(USART1, ENABLE);
#endif
    usart_rxne = false;
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40U) == 0U) {}
        USART_SendData(USART1, (uint16_t)*s);
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
        }
        s++;
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&usart_rx);
}

char usart1_readc(void) {
    return (char)buf_getbyte(&usart_rx);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&usart_rx);
}

bool dispatch_uart_command(int command) {
    if (command < 0 || command >= MAXIDSIZE) {
        return false;
    }
    msgid = (_ID)command;
    callme = flookup[command];
    if (callme != NULL) {
        callme();
    }
    return true;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0U) {
        char c = usart1_readc();
        (void)dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
    static uint32_t t_prev = 0U;
    if ((msTicks - t_prev) > 1000U) {
        t_prev = msTicks;
    }
}

void init_system(void) {
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

#ifndef DISCOBOT_TARGET
int func2(int R0) {
    return (int)((uint32_t)R0 + 1U);
}

int func1(int R0) {
    return func2(R0);
}
#endif