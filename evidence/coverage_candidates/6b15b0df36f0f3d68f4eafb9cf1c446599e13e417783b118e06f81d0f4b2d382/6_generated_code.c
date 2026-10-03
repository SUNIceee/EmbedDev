/*
 * DiscoBot Test-Compatible v3 implementation.
 *
 * When DISCOBOT_TARGET is not defined, this file implements the deterministic
 * host model required for testability.  When DISCOBOT_TARGET is defined, the
 * hardware-visible functions use STM32F407 SPL register access while still
 * updating the observable GPIO arrays/TX log.
 *
 * No standalone main is provided.
 */
#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

/* -------------------------------------------------------------------------
 * Global observable state
 * ------------------------------------------------------------------------- */
volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = 0;

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
#ifndef DISCOBOT_TARGET
uint32_t SystemCoreClock = 168000000;
#endif
uint32_t SysTick_reload = SYSTICK_PERIOD_TICKS;
uint32_t usart1_baud_used = UART_BAUD;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* -------------------------------------------------------------------------
 * Host-only helpers/state
 * ------------------------------------------------------------------------- */
#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000ul;
}

static int SysTick_Config(uint32_t ticks)
{
    if ((ticks == 0u) || (ticks > 0x00FFFFFFu)) {
        return 1;
    }
    SysTick_reload = ticks;
    return 0;
}
#endif

static CircArray msg;

static int16_t host_accel_mg[3] = {0, 0, 0};
static uint16_t host_adc_raw = 0;
static volatile bool host_rng_ready = true;
static volatile uint32_t host_rng_value = 0;
static uint8_t host_usart_dr = 0;
static bool host_usart_rxne = false;
static ButtonState last_button_state = ButtonIsReleased;

static float adc_raw_to_temperature(uint16_t raw)
{
    float temp;

    temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    return temp;
}

/* -------------------------------------------------------------------------
 * Small pin-write helpers preserve observable arrays in both modes.
 * ------------------------------------------------------------------------- */
static void gpioa_write_pin(uint8_t pin, uint8_t value)
{
    if (pin > 15u) {
        return;
    }

#ifdef DISCOBOT_TARGET
    if (value != 0u) {
        GPIOA->BSRRL = ((uint32_t)1u << pin);
    } else {
        GPIOA->BSRRH = ((uint32_t)1u << pin);
    }
#endif

    GPIOA_output[pin] = (value != 0u) ? 1u : 0u;
}

static void gpiod_write_pin(uint8_t pin, uint8_t value)
{
    if (pin > 15u) {
        return;
    }

#ifdef DISCOBOT_TARGET
    if (value != 0u) {
        GPIOD->BSRRL = ((uint32_t)1u << pin);
    } else {
        GPIOD->BSRRH = ((uint32_t)1u << pin);
    }
#endif

    GPIOD_output[pin] = (value != 0u) ? 1u : 0u;
}

/* -------------------------------------------------------------------------
 * System initialization
 * ------------------------------------------------------------------------- */
#ifndef DISCOBOT_TARGET
void SystemInit(void)
{
    SystemCoreClock = 168000000ul;
}
#endif

void init_systick(void)
{
#ifdef DISCOBOT_TARGET
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) {
        while (1) {
        }
    }

    SysTick_reload = SystemCoreClock / 1000u;
#else
    uint32_t ticks;

    SystemCoreClockUpdate();
    ticks = SystemCoreClock / 1000u;

    if (SysTick_Config(ticks) != 0) {
        SysTick_reload = 0;
        return;
    }

    SysTick_reload = ticks;
#endif
}

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

bool dispatch_uart_command(int command)
{
    if ((command < 0) || (command >= MAXIDSIZE)) {
        return false;
    }

    msgid = (_ID)command;
    callme = flookup[command];
    callme();
    return true;
}

void main_loop_iteration(void)
{
    static uint32_t t_prev = 0;

    if ((msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
        /* 1-second idle timer has no required action. */
    }

    if (usart1_available() > 0u) {
        int command = (int)usart1_readc();
        (void)dispatch_uart_command(command);
    }

    checkbutton();
    execute_tasks();
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

/* -------------------------------------------------------------------------
 * Motor control / car
 * ------------------------------------------------------------------------- */
void init_GPIO_A1A2A3A4_output(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIOA->BSRRH = (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
#endif

    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed; /* No PWM: speed is intentionally unused. */

    switch (direc) {
    case FORWARD:
        gpioa_write_pin(1, 1);
        gpioa_write_pin(2, 0);
        break;
    case BACKWARD:
        gpioa_write_pin(1, 0);
        gpioa_write_pin(2, 1);
        break;
    case STOP:
    default:
        gpioa_write_pin(1, 0);
        gpioa_write_pin(2, 0);
        break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed; /* No PWM: speed is intentionally unused. */

    switch (direc) {
    case FORWARD:
        gpioa_write_pin(3, 1);
        gpioa_write_pin(4, 0);
        break;
    case BACKWARD:
        gpioa_write_pin(3, 0);
        gpioa_write_pin(4, 1);
        break;
    case STOP:
    default:
        gpioa_write_pin(3, 0);
        gpioa_write_pin(4, 0);
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

/* -------------------------------------------------------------------------
 * Button input and debounce
 * ------------------------------------------------------------------------- */
int read_buttonc(int i)
{
    if ((i < 0) || (i > 3)) {
        return -1;
    }

#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> (uint32_t)i) & 1u);
#else
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
#endif
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
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
    if (last_button_state == buttstate) {
        return;
    }

    switch (buttstate) {
    case ButtonIsPressed:
        /* Released -> Pressed transition observed. */
        break;
    case ButtonIsReleased:
        /* Pressed -> Released transition observed. */
        break;
    default:
        break;
    }

    last_button_state = buttstate;
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    const double pi = 3.14159265358979323846;

    if (roll != NULL) {
        *roll = (float)(atan2((double)acc_y, (double)acc_z) * (180.0 / pi));
    }

    if (pitch != NULL) {
        double denom = sqrt((double)acc_y * (double)acc_y +
                           (double)acc_z * (double)acc_z);
        *pitch = (float)(atan2(-(double)acc_x, denom) * (180.0 / pi));
    }
}

/* -------------------------------------------------------------------------
 * LED control
 * ------------------------------------------------------------------------- */
void init_LED_pins(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIOD->BSRRH = (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
#endif

    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i)
{
    if ((i < 0) || (i > 3)) {
        return;
    }

    gpiod_write_pin((uint8_t)(12 + i), 1);
}

void LED_Off(int i)
{
    if ((i < 0) || (i > 3)) {
        return;
    }

    gpiod_write_pin((uint8_t)(12 + i), 0);
}

/* -------------------------------------------------------------------------
 * Accelerometers
 * ------------------------------------------------------------------------- */
int init_accelerometers(void)
{
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G,
                                    TM_LIS3DSH_Filter_50Hz);
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
    TM_LIS302DL_LIS3DSH_Axes_t axes;

    axes = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = (float)axes.X / 1000.0f;
    b[1] = (float)axes.Y / 1000.0f;
    b[2] = (float)axes.Z / 1000.0f;
#else
    b[0] = (float)host_accel_mg[0] / 1000.0f;
    b[1] = (float)host_accel_mg[1] / 1000.0f;
    b[2] = (float)host_accel_mg[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    host_accel_mg[0] = x_mg;
    host_accel_mg[1] = y_mg;
    host_accel_mg[2] = z_mg;
}

/* -------------------------------------------------------------------------
 * Temperature sensor (ADC1)
 * ------------------------------------------------------------------------- */
void init_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#else
    /* Host model has no separate unready state in the fixed API. */
#endif
}

float read_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    uint16_t raw;

    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }

    raw = (uint16_t)ADC_GetConversionValue(ADC1);
    return adc_raw_to_temperature(raw);
#else
    return adc_raw_to_temperature(host_adc_raw);
#endif
}

void discobot_set_adc_raw(uint16_t raw)
{
    host_adc_raw = raw;
}

/* -------------------------------------------------------------------------
 * RNG
 * ------------------------------------------------------------------------- */
void init_rng(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#else
    /* Host model: enabling RNG does not overwrite injected data. */
#endif
}

uint32_t get_random_number(void)
{
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }

    return RNG_GetRandomNumber();
#else
    /*
     * Host mode must not deadlock.  Callers can observe readiness through
     * discobot_set_rng(); this path returns the last injected value.
     */
    return host_rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value)
{
    host_rng_value = value;
    host_rng_ready = ready;
}

/* -------------------------------------------------------------------------
 * Timed task scheduler
 * ------------------------------------------------------------------------- */
void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    uint32_t i;
    long msinterval = (long)(interval_sec * 1000.0f);

    if (myfunc == NULL) {
        return;
    }

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)msinterval;
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }

    /* All 25 slots are full: silent failure. */
}

void execute_tasks(void)
{
    uint32_t i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];

        if ((t->task != NULL) &&
            (msTicks >= (t->last_called + t->msinterval))) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void)
{
    uint32_t i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", (int)i, (int)timed_tasks[i].numcalls);
        }
    }
}

/* -------------------------------------------------------------------------
 * CircArray
 * ------------------------------------------------------------------------- */
void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\r\n");
        return;
    }

    if (size <= 0) {
        arr->buf = NULL;
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }

    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == NULL) {
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }

    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c)
{
    if ((arr == NULL) || (arr->buf == NULL) || !arr->enabled || (arr->size == 0u)) {
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
    char c;

    if ((arr == NULL) || (arr->buf == NULL) || !arr->enabled || (arr->size == 0u)) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr)
{
    if ((arr == NULL) || !arr->enabled) {
        return true;
    }

    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if ((arr == NULL) || (arr->buf == NULL) || !arr->enabled || (arr->size == 0u)) {
        return false;
    }

    if (buf_empty(arr)) {
        return false;
    }

    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr)
{
    if ((arr == NULL) || !arr->enabled || (arr->size == 0u)) {
        return 0;
    }

    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize)
{
    char *new_buf;

    if ((arr == NULL) || (newSize <= 0)) {
        return false;
    }

    new_buf = (char *)realloc(arr->buf, (size_t)newSize);
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
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL) {
        return;
    }

    if ((arr->buf != NULL) && (arr->size > 0u)) {
        memset(arr->buf, 0, arr->size);
    }
}

/* -------------------------------------------------------------------------
 * USART1
 * ------------------------------------------------------------------------- */
void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
#ifndef DISCOBOT_TARGET
        fprintf(stderr, "init_usart1: baud==0, defaulting to 9600\r\n");
#endif
    }

#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* PB6 = TX, PB7 = RX, AF7 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = (uint32_t)baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART_Cmd(USART1, ENABLE);
#else
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
#endif

    usart1_baud_used = (uint32_t)baud;
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = (char)USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, c);
    }
#else
    if (host_usart_rxne) {
        char c = (char)host_usart_dr;
        host_usart_rxne = false;

        /* If the buffer is full, buf_putbyte returns 0 and data is lost. */
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
        char c = (char)*s;
        s++;

#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40u) == 0u) {
        }
        USART1->DR = (uint16_t)c;
#endif

        if (usart1_tx_length < (sizeof(usart1_tx_log) - 1u)) {
            usart1_tx_log[usart1_tx_length] = c;
            usart1_tx_length++;
        }
    }

    if (usart1_tx_length < sizeof(usart1_tx_log)) {
        usart1_tx_log[usart1_tx_length] = '\0';
    } else {
        usart1_tx_log[sizeof(usart1_tx_log) - 1u] = '\0';
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
    if (!rxne) {
        host_usart_rxne = false;
        return;
    }

    host_usart_dr = value;
    host_usart_rxne = true;

#ifdef DISCOBOT_TARGET
    /*
     * Target builds normally receive through the real USART1 interrupt.
     * This function is kept as a host-test injection API and has no target
     * hardware effect.
     */
    (void)value;
#else
    USART1_IRQHandler();
#endif
}

/* -------------------------------------------------------------------------
 * ARM assembly funcs1.s host-equivalent semantics
 * ------------------------------------------------------------------------- */
int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}
