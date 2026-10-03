#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#endif

volatile uint32_t msTicks;
volatile uint32_t b_i;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme;
uint8_t GPIOA_output[16];
uint8_t GPIOD_output[16];
uint32_t GPIOA_IDR;
uint32_t SystemCoreClock = 168000000UL;
uint32_t SysTick_reload;
uint32_t usart1_baud_used;
char usart1_tx_log[1024];
size_t usart1_tx_length;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS];

static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_previous_tick;
static CircArray msg;
static int16_t accel_raw[3];
static uint16_t adc_raw_value;
static volatile bool rng_ready;
static uint32_t rng_value;
static bool systick_failed;
#ifndef DISCOBOT_TARGET
static uint8_t usart_rx_dr;
static bool usart_rxne;
#endif

#ifdef DISCOBOT_TARGET
static void gpioa_pin_write(unsigned pin, uint8_t value)
{
    if (pin >= 16U) return;
    if (value) GPIO_SetBits(GPIOA, (uint16_t)(1U << pin));
    else GPIO_ResetBits(GPIOA, (uint16_t)(1U << pin));
    GPIOA_output[pin] = value ? 1U : 0U;
}
static void gpiod_pin_write(unsigned pin, uint8_t value)
{
    if (pin >= 16U) return;
    if (value) GPIO_SetBits(GPIOD, (uint16_t)(1U << pin));
    else GPIO_ResetBits(GPIOD, (uint16_t)(1U << pin));
    GPIOD_output[pin] = value ? 1U : 0U;
}
#else
static void gpioa_pin_write(unsigned pin, uint8_t value)
{
    if (pin < 16U) GPIOA_output[pin] = value ? 1U : 0U;
}
static void gpiod_pin_write(unsigned pin, uint8_t value)
{
    if (pin < 16U) GPIOD_output[pin] = value ? 1U : 0U;
}
#endif

void SystemInit(void)
{
    SystemCoreClock = 168000000UL;
}

void init_systick(void)
{
#ifdef DISCOBOT_TARGET
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000UL) != 0U) {
        systick_failed = true;
        while (1) { }
    }
#else
    if (SystemCoreClock == 0U) {
        systick_failed = true;
        SysTick_reload = 0U;
        return;
    }
#endif
    SysTick_reload = SystemCoreClock / 1000UL;
}

void init_LED_pins(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_StructInit(&init);
    init.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    init.GPIO_Mode = GPIO_Mode_OUT;
    init.GPIO_OType = GPIO_OType_PP;
    init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    init.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &init);
#endif
    for (unsigned i = 12U; i <= 15U; ++i) gpiod_pin_write(i, 0U);
}

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&init);
    init.GPIO_Pin = GPIO_Pin_0;
    init.GPIO_Mode = GPIO_Mode_IN;
    init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &init);
#endif
    GPIOA_IDR &= ~1UL;
    b_i = 0U;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void init_GPIO_A1A2A3A4_output(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&init);
    init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    init.GPIO_Mode = GPIO_Mode_OUT;
    init.GPIO_OType = GPIO_OType_PP;
    init.GPIO_PuPd = GPIO_PuPd_UP;
    init.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &init);
#endif
    for (unsigned i = 1U; i <= 4U; ++i) gpioa_pin_write(i, 0U);
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    gpioa_pin_write(1U, 0U);
    gpioa_pin_write(2U, 0U);
    if (direc == FORWARD) gpioa_pin_write(1U, 1U);
    else if (direc == BACKWARD) gpioa_pin_write(2U, 1U);
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    gpioa_pin_write(3U, 0U);
    gpioa_pin_write(4U, 0U);
    if (direc == FORWARD) gpioa_pin_write(3U, 1U);
    else if (direc == BACKWARD) gpioa_pin_write(4U, 1U);
}

void move_forward(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(STOP, 1.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(STOP, 1.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void stop(void) { set_left_motor_direc(STOP, 1.0f); set_right_motor_direc(STOP, 1.0f); }

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left,
    move_forward_soft_right, move_backward_soft_left,
    move_backward_soft_right, move_spin_right, move_spin_left, stop
};

bool dispatch_uart_command(int command)
{
    if (command < 0 || command >= MAXIDSIZE) return false;
    msgid = (_ID)command;
    callme = flookup[command];
    if (callme != NULL) callme();
    return true;
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) return -1;
#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> i) & 1U);
#else
    return (int)((GPIOA_IDR >> i) & 1U);
#endif
}

void SysTick_Handler(void)
{
    ++msTicks;
    if (read_buttonc(0) == 1) {
        ++b_i;
        if (b_i >= BUTTON_DEBOUNCE_MS) buttstate = ButtonIsPressed;
    } else {
        b_i = 0U;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void)
{
    if (last_button_state != buttstate) {
        switch (buttstate) {
        case ButtonIsPressed: break;
        case ButtonIsReleased: break;
        default: break;
        }
        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    const float degrees = 180.0f / 3.14159265358979323846f;
    if (roll != NULL) *roll = (float)(atan2((double)acc_y, (double)acc_z) * degrees);
    if (pitch != NULL) {
        double denominator = sqrt((double)acc_y * (double)acc_y + (double)acc_z * (double)acc_z);
        *pitch = (float)(atan2((double)-acc_x, denominator) * degrees);
    }
}

void discobot_set_button_level(bool high)
{
    if (high) GPIOA_IDR |= 1UL;
    else GPIOA_IDR &= ~1UL;
}

void LED_On(int i)
{
    if (i >= 0 && i < 4) gpiod_pin_write((unsigned)(12 + i), 1U);
}

void LED_Off(int i)
{
    if (i >= 0 && i < 4) gpiod_pin_write((unsigned)(12 + i), 0U);
}

#ifdef DISCOBOT_TARGET
#ifndef TM_LIS3DSH_Sensitivity_2G
#define TM_LIS3DSH_Sensitivity_2G 0
#endif
#ifndef TM_LIS3DSH_Filter_50Hz
#define TM_LIS3DSH_Filter_50Hz 0
#endif
extern int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
typedef struct { int X; int Y; int Z; } discobot_axes_t;
extern void TM_LIS302DL_LIS3DSH_ReadAxes(discobot_axes_t *axes);
#endif

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
    if (b == NULL) return;
#ifdef DISCOBOT_TARGET
    discobot_axes_t axes;
    TM_LIS302DL_LIS3DSH_ReadAxes(&axes);
    b[0] = (float)axes.X / 1000.0f;
    b[1] = (float)axes.Y / 1000.0f;
    b[2] = (float)axes.Z / 1000.0f;
#else
    b[0] = (float)accel_raw[0] / 1000.0f;
    b[1] = (float)accel_raw[1] / 1000.0f;
    b[2] = (float)accel_raw[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    accel_raw[0] = x_mg;
    accel_raw[1] = y_mg;
    accel_raw[2] = z_mg;
}

void init_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    ADC_InitTypeDef adc;
    ADC_CommonInitTypeDef common;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonStructInit(&common);
    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInit(&common);
    ADC_StructInit(&adc);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#endif
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = (uint16_t)(raw & 0x0FFFU);
}

float read_temperature_sensor(void)
{
    uint32_t raw;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    raw = ADC_GetConversionValue(ADC1);
#else
    raw = adc_raw_value;
#endif
    {
        float temperature = (float)raw / 4095.0f;
        temperature *= 3.3f;
        temperature -= 0.760f;
        temperature /= 0.0025f;
        temperature += 25.0f;
        return temperature;
    }
}

void init_rng(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
}

void discobot_set_rng(bool ready, uint32_t value)
{
    rng_ready = ready;
    rng_value = value;
}

uint32_t get_random_number(void)
{
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) { }
    return RNG_GetRandomNumber();
#else
    /* The frozen API has no error result. Bound the host wait so a not-ready
       injection is observable as an unavailable result without deadlocking. */
    for (volatile uint32_t spins = 0U; !rng_ready && spins < 100000U; ++spins) { }
    if (!rng_ready) return 0U;
    return rng_value;
#endif
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    if (myfunc == NULL) return;
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            long interval = (long)(interval_sec * 1000.0f);
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)interval;
            timed_tasks[i].last_called = 0U;
            timed_tasks[i].numcalls = 0U;
            return;
        }
    }
}

void execute_tasks(void)
{
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *task = &timed_tasks[i];
        if (task->task != NULL && msTicks >= task->last_called + task->msinterval) {
            task->task();
            task->last_called = msTicks;
            ++task->numcalls;
        }
    }
}

void printtimes(void)
{
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
    }
}

void initCircArray(CircArray *arr, int size)
{
    char *memory;
    if (arr == NULL || size <= 0) return;
    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }
    memory = (char *)calloc((size_t)size, sizeof(char));
    if (memory == NULL) return;
    arr->buf = memory;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0U;
    arr->n_w = 0U;
}

bool buf_empty(CircArray *arr)
{
    return arr == NULL || arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->size == 0U || buf_empty(arr)) return false;
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0U || buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    ++arr->n_w;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    char value;
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0U || buf_empty(arr)) return 0;
    value = arr->buf[arr->n_r % arr->size];
    ++arr->n_r;
    return value;
}

int buf_available(CircArray *arr)
{
    if (arr == NULL) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize)
{
    char *memory;
    if (arr == NULL || !arr->enabled || arr->buf == NULL || newSize <= 0) return false;
    memory = (char *)realloc(arr->buf, (size_t)newSize);
    if (memory == NULL) return false;
    arr->buf = memory;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr != NULL && arr->buf != NULL && arr->size != 0U) memset(arr->buf, 0, arr->size);
}

bool buf_delete(CircArray *arr)
{
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

void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "USART1 baud was zero; using 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) buf_delete(&msg);
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
#ifdef DISCOBOT_TARGET
    {
        GPIO_InitTypeDef gpio;
        USART_InitTypeDef uart;
        NVIC_InitTypeDef nvic;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
        GPIO_StructInit(&gpio);
        gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        gpio.GPIO_Mode = GPIO_Mode_AF;
        gpio.GPIO_OType = GPIO_OType_PP;
        gpio.GPIO_PuPd = GPIO_PuPd_UP;
        gpio.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_Init(GPIOB, &gpio);
        USART_StructInit(&uart);
        uart.USART_BaudRate = (uint32_t)baud;
        uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART_Init(USART1, &uart);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        nvic.NVIC_IRQChannel = USART1_IRQn;
        nvic.NVIC_IRQChannelPreemptionPriority = 0;
        nvic.NVIC_IRQChannelSubPriority = 0;
        nvic.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic);
        USART_Cmd(USART1, ENABLE);
    }
#endif
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        (void)buf_putbyte(&msg, (char)USART_ReceiveData(USART1));
    }
#else
    if (usart_rxne) {
        (void)buf_putbyte(&msg, (char)usart_rx_dr);
        usart_rxne = false;
    }
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) return;
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40U) == 0U) { }
        USART_SendData(USART1, (uint16_t)(unsigned char)*s);
#else
        if (usart1_tx_length < sizeof(usart1_tx_log)) usart1_tx_log[usart1_tx_length++] = *s;
#endif
        ++s;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)(unsigned char)buf_getbyte(&msg);
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
#ifndef DISCOBOT_TARGET
    if (rxne) {
        usart_rx_dr = value;
        usart_rxne = true;
        USART1_IRQHandler();
    }
#else
    (void)value;
    (void)rxne;
#endif
}

void main_loop_iteration(void)
{
    if (usart1_available() != 0U) (void)dispatch_uart_command((int)(signed char)usart1_readc());
    checkbutton();
    execute_tasks();
    if ((uint32_t)(msTicks - main_previous_tick) > 1000U) main_previous_tick = msTicks;
}

void init_system(void)
{
    memset(timed_tasks, 0, sizeof(timed_tasks));
    msTicks = 0U;
    b_i = 0U;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
    callme = flookup[FWD];
    msgid = FWD;
    usart1_tx_length = 0U;
    memset(usart1_tx_log, 0, sizeof(usart1_tx_log));
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
    main_previous_tick = msTicks;
    (void)systick_failed;
}

int func2(int R0)
{
    return R0 + 1;
}

int func1(int R0)
{
    return func2(R0);
}