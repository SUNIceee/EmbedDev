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
#ifndef TM_LIS3DSH_Sensitivity_2G
#define TM_LIS3DSH_Sensitivity_2G 0
#endif
#ifndef TM_LIS3DSH_Filter_50Hz
#define TM_LIS3DSH_Filter_50Hz 0
#endif
typedef struct { int16_t X; int16_t Y; int16_t Z; } TM_LIS302DL_LIS3DSH_t;
extern int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
extern void TM_LIS302DL_LIS3DSH_ReadAxes(TM_LIS302DL_LIS3DSH_t *axes);
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
#ifndef DISCOBOT_TARGET
uint32_t SystemCoreClock = 168000000u;
#endif
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS];

static CircArray msg;
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_loop_prev_tick = 0;
static int16_t accel_raw[3] = {0, 0, 0};
static uint16_t adc_raw_value = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;
static bool temperature_initialized = false;
static bool rng_initialized = false;
static bool accelerometer_initialized = false;
static bool usart1_enabled = false;
static uint8_t usart1_dr = 0;
static bool usart1_rxne = false;
static bool usart1_txe = true;
static int systick_config_status = 0;

motor_command_fn callme = stop;
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

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000u;
}

static int SysTick_Config_host(uint32_t ticks)
{
    SysTick_reload = ticks;
    return systick_config_status;
}
#else
#define SysTick_Config_host(ticks) SysTick_Config((ticks))
#endif

static void set_gpioa_pin(unsigned int pin, uint8_t value)
{
    if (pin < 16u) {
        GPIOA_output[pin] = value ? 1u : 0u;
    }
#ifdef DISCOBOT_TARGET
    if (pin < 16u) {
        if (value) {
            GPIOA->BSRRL = (uint16_t)(1u << pin);
        } else {
            GPIOA->BSRRH = (uint16_t)(1u << pin);
        }
    }
#endif
}

static void set_gpiod_pin(unsigned int pin, uint8_t value)
{
    if (pin < 16u) {
        GPIOD_output[pin] = value ? 1u : 0u;
    }
#ifdef DISCOBOT_TARGET
    if (pin < 16u) {
        if (value) {
            GPIOD->BSRRL = (uint16_t)(1u << pin);
        } else {
            GPIOD->BSRRH = (uint16_t)(1u << pin);
        }
    }
#endif
}

#ifndef DISCOBOT_TARGET
void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}
#endif

void init_systick(void)
{
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000u;
    if (SysTick_Config_host(SysTick_reload) != 0) {
        for (;;) {
        }
    }
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

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
#endif
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

bool dispatch_uart_command(int command)
{
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

void main_loop_iteration(void)
{
    if ((uint32_t)(msTicks - main_loop_prev_tick) > 1000u) {
        main_loop_prev_tick = msTicks;
    }
    if (usart1_available() > 0u) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
#endif
    set_gpioa_pin(1u, 0u);
    set_gpioa_pin(2u, 0u);
    set_gpioa_pin(3u, 0u);
    set_gpioa_pin(4u, 0u);
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        set_gpioa_pin(1u, 1u);
        set_gpioa_pin(2u, 0u);
        break;
    case BACKWARD:
        set_gpioa_pin(1u, 0u);
        set_gpioa_pin(2u, 1u);
        break;
    case STOP:
    default:
        set_gpioa_pin(1u, 0u);
        set_gpioa_pin(2u, 0u);
        break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        set_gpioa_pin(3u, 1u);
        set_gpioa_pin(4u, 0u);
        break;
    case BACKWARD:
        set_gpioa_pin(3u, 0u);
        set_gpioa_pin(4u, 1u);
        break;
    case STOP:
    default:
        set_gpioa_pin(3u, 0u);
        set_gpioa_pin(4u, 0u);
        break;
    }
}

void move_forward(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void move_backward(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_forward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void move_forward_soft_right(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_backward_soft_right(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_spin_left(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void stop(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

int read_buttonc(int i)
{
    if (i > 3 || i < 0) {
        return -1;
    }
#ifdef DISCOBOT_TARGET
    GPIOA_IDR = GPIOA->IDR;
#endif
    return (int)((GPIOA_IDR >> (unsigned int)i) & 1u);
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
        default:
            break;
        }
        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z))) * 180.0f / PI;
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

void init_LED_pins(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &gpio);
#endif
    set_gpiod_pin(12u, 0u);
    set_gpiod_pin(13u, 0u);
    set_gpiod_pin(14u, 0u);
    set_gpiod_pin(15u, 0u);
}

void LED_On(int i)
{
    if (i >= 0 && i < 4) {
        set_gpiod_pin((unsigned int)(12 + i), 1u);
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i < 4) {
        set_gpiod_pin((unsigned int)(12 + i), 0u);
    }
}

int init_accelerometers(void)
{
    accelerometer_initialized = true;
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
    TM_LIS302DL_LIS3DSH_t axes;
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
    ADC_CommonInitTypeDef adc_common;
    ADC_InitTypeDef adc;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonStructInit(&adc_common);
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    ADC_CommonInit(&adc_common);
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
    temperature_initialized = true;
}

float read_temperature_sensor(void)
{
    float temp;
    (void)temperature_initialized;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }
    temp = (float)ADC_GetConversionValue(ADC1);
#else
    temp = (float)adc_raw_value;
#endif
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = (uint16_t)(raw & 0x0fffu);
}

void init_rng(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
    rng_initialized = true;
}

uint32_t get_random_number(void)
{
    (void)rng_initialized;
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }
    return RNG_GetRandomNumber();
#else
    if (!rng_ready) {
        fprintf(stderr, "DiscoBot RNG read attempted while DRDY is reset\n");
        abort();
    }
    return rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value)
{
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
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
        printf("Error: CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) {
        arr->size = 0u;
        arr->enabled = false;
        arr->n_r = 0u;
        arr->n_w = 0u;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
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

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0u || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    char c;
    if (arr == NULL || !arr->enabled || arr->buf == NULL || arr->size == 0u || buf_empty(arr)) {
        return 0;
    }
    c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
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

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }
    memset(arr->buf, 0, arr->size);
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

void init_usart1(int baud)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
#endif
    if (baud == 0) {
        baud = UART_BAUD;
        printf("Warning: baud 0, using 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;
    if (!msg.enabled) {
        initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    }
#ifdef DISCOBOT_TARGET
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
    USART_StructInit(&usart);
    usart.USART_BaudRate = (uint32_t)baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART1, ENABLE);
#endif
    usart1_enabled = true;
    usart1_txe = true;
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        (void)buf_putbyte(&msg, (char)(USART1->DR & 0xffu));
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
#else
    if (usart1_rxne) {
        (void)buf_putbyte(&msg, (char)usart1_dr);
        usart1_rxne = false;
    }
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }
#ifdef DISCOBOT_TARGET
    while (!usart1_enabled) {
    }
#endif
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40u) == 0u) {
        }
        USART_SendData(USART1, (uint16_t)(uint8_t)*s);
#else
        if (!usart1_enabled) {
            return;
        }
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1u) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
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
    return (char)((int8_t)buf_getbyte(&msg));
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    usart1_dr = value;
    usart1_rxne = rxne;
    USART1_IRQHandler();
}

int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}