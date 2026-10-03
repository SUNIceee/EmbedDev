#include "6_generated_code.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef DISCOBOT_TARGET
#include <errno.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
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
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = UART_BAUD;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {0};

static CircArray msg = {0};
static ButtonState last_button_state = ButtonIsReleased;

#ifndef DISCOBOT_TARGET
static int16_t accel_mg[3] = {0, 0, 0};
static volatile bool adc_eoc = false;
static uint16_t adc_raw = 0;
static volatile bool rng_ready = false;
static uint32_t rng_value = 0;
static volatile bool usart1_rxne = false;
static uint8_t usart1_dr = 0;
#endif

static void write_gpioa_pin(int pin, uint8_t value) {
    if (pin < 0 || pin > 15) return;
    GPIOA_output[pin] = value ? 1u : 0u;
#ifdef DISCOBOT_TARGET
    if (value) {
        GPIOA->BSRRL = (uint32_t)(1u << pin);
    } else {
        GPIOA->BSRRH = (uint32_t)(1u << pin);
    }
#endif
}

static void write_gpiod_pin(int pin, uint8_t value) {
    if (pin < 0 || pin > 15) return;
    GPIOD_output[pin] = value ? 1u : 0u;
#ifdef DISCOBOT_TARGET
    if (value) {
        GPIOD->BSRRL = (uint32_t)(1u << pin);
    } else {
        GPIOD->BSRRH = (uint32_t)(1u << pin);
    }
#endif
}

static void append_tx_log(char c) {
    if (usart1_tx_length < sizeof(usart1_tx_log)) {
        usart1_tx_log[usart1_tx_length++] = c;
    }
}

#ifndef DISCOBOT_TARGET
void SystemInit(void) {
    SystemCoreClock = 168000000;
}
#endif

void init_systick(void) {
#ifdef DISCOBOT_TARGET
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000;
    if (SysTick_Config(SysTick_reload) != 0) {
        while (1) {}
    }
#else
    SystemCoreClock = 168000000;
    SysTick_reload = SYSTICK_PERIOD_TICKS;
#endif
}

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif
    write_gpiod_pin(12, 0);
    write_gpiod_pin(13, 0);
    write_gpiod_pin(14, 0);
    write_gpiod_pin(15, 0);
}

void init_button(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#else
    GPIOA_IDR &= ~1u;
#endif
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void init_GPIO_A1A2A3A4_output(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
    write_gpioa_pin(1, 0);
    write_gpioa_pin(2, 0);
    write_gpioa_pin(3, 0);
    write_gpioa_pin(4, 0);
}

void set_left_motor_direc(int direc, float speed) {
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

void set_right_motor_direc(int direc, float speed) {
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

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
        return true;
    }
    return false;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) return -1;
#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> i) & 1u);
#else
    return (int)((GPIOA_IDR >> i) & 1u);
#endif
}

void discobot_set_button_level(bool high) {
#ifndef DISCOBOT_TARGET
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
#else
    (void)high;
#endif
}

void SysTick_Handler(void) {
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

void checkbutton(void) {
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

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (pitch == NULL || roll == NULL) return;
    float pi = (float)M_PI;
    *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    float r = sqrtf(acc_y * acc_y + acc_z * acc_z);
    *pitch = atan2f(-acc_x, r) * 180.0f / pi;
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        write_gpiod_pin(12 + i, 1);
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        write_gpiod_pin(12 + i, 0);
    }
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void read_accelerometers(float b[3]) {
    if (b == NULL) return;
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_Axes_t axes;
    TM_LIS302DL_LIS3DSH_ReadAxes(&axes);
    b[0] = axes.X / 1000.0f;
    b[1] = axes.Y / 1000.0f;
    b[2] = axes.Z / 1000.0f;
#else
    b[0] = accel_mg[0] / 1000.0f;
    b[1] = accel_mg[1] / 1000.0f;
    b[2] = accel_mg[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
#ifndef DISCOBOT_TARGET
    accel_mg[0] = x_mg;
    accel_mg[1] = y_mg;
    accel_mg[2] = z_mg;
#else
    (void)x_mg;
    (void)y_mg;
    (void)z_mg;
#endif
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#else
    adc_eoc = false;
    adc_raw = 0;
#endif
}

#ifndef DISCOBOT_TARGET
static void adc_start_conversion(void) {
    /* Host model: discobot_set_adc_raw makes a result ready. */
}
#endif

float read_temperature_sensor(void) {
    uint16_t raw;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    raw = (uint16_t)ADC_GetConversionValue(ADC1);
#else
    adc_start_conversion();
    errno = 0;
    if (!adc_eoc) {
        errno = EAGAIN;
        return nanf("");
    }
    raw = adc_raw;
    adc_eoc = false;
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
#ifndef DISCOBOT_TARGET
    adc_raw = raw;
    adc_eoc = true;
#else
    (void)raw;
#endif
}

void init_rng(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#else
    rng_ready = false;
    rng_value = 0;
#endif
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {}
    return RNG_GetRandomNumber();
#else
    errno = 0;
    if (!rng_ready) {
        errno = EAGAIN;
        return 0;
    }
    uint32_t value = rng_value;
    rng_ready = false;
    return value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) {
#ifndef DISCOBOT_TARGET
    rng_ready = ready;
    rng_value = value;
#else
    (void)ready;
    (void)value;
#endif
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) return;
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) return;
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\r\n");
        return;
    }
    char *newbuf = (char *)calloc((size_t)size, 1);
    if (newbuf == NULL) return;
    arr->buf = newbuf;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) return 0;
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) return 0;
    if (!buf_empty(arr)) {
        char c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0) return false;
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL || newSize <= 0) return false;
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) return false;
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    if (arr->buf != NULL && arr->size > 0) {
        memset(arr->buf, 0, arr->size);
    }
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) return;
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "init_usart1: baud is 0, using 9600\r\n");
    }
    usart1_baud_used = (uint32_t)baud;
#ifndef DISCOBOT_TARGET
    usart1_rxne = false;
    usart1_dr = 0;
#endif
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = (uint32_t)baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, (char)c);
    }
#else
    if (usart1_rxne) {
        uint8_t c = usart1_dr;
        usart1_rxne = false;
        (void)buf_putbyte(&msg, (char)c);
    }
#endif
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s != '\0') {
        char c = *s++;
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40u) == 0u) {}
        USART_SendData(USART1, (uint16_t)(unsigned char)c);
#else
        /* Host model: USART TX is always ready. */
#endif
        append_tx_log(c);
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)(unsigned char)buf_getbyte(&msg);
}

char usart1_readc(void) {
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
#ifndef DISCOBOT_TARGET
    usart1_dr = value;
    usart1_rxne = rxne;
    if (rxne) {
        USART1_IRQHandler();
    }
#else
    (void)value;
    (void)rxne;
#endif
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        int c = (int)usart1_readc();
        (void)dispatch_uart_command(c);
    }
    checkbutton();
    execute_tasks();

    static uint32_t t_prev = 0;
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
}

#ifndef DISCOBOT_TARGET
int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    uint32_t v = (uint32_t)R0;
    v += 1u;
    return (int)v;
}
#endif