#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
#include "stm32f4xx.h"
#include "misc.h"

extern int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} DiscobotAxes;
extern void TM_LIS302DL_LIS3DSH_ReadAxes(DiscobotAxes *axes);

#ifndef TM_LIS302DL_LIS3DSH_Sensitivity_2G
#define TM_LIS302DL_LIS3DSH_Sensitivity_2G 0
#endif
#ifndef TM_LIS302DL_LIS3DSH_Filter_50Hz
#define TM_LIS302DL_LIS3DSH_Filter_50Hz 0
#endif
#else
static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000U;
}

static uint32_t SysTick_Config(uint32_t ticks)
{
    if (ticks == 0U) return 1U;
    SysTick_reload = ticks;
    return 0U;
}
#endif

volatile uint32_t msTicks = 0U;
volatile uint32_t b_i = 0U;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = 0;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
#ifndef DISCOBOT_TARGET
uint32_t SystemCoreClock = 168000000U;
#endif
uint32_t GPIOA_IDR = 0U;
uint32_t SysTick_reload = 0U;
uint32_t usart1_baud_used = 0U;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0U;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static int16_t accel_x_mg;
static int16_t accel_y_mg;
static int16_t accel_z_mg;
static uint16_t adc_raw;
static bool adc_eoc;
static bool rng_ready;
static uint32_t rng_value;
static CircArray msg;
static uint8_t usart_dr;
static bool usart_rxne;
static bool usart_txe_ready = true;
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t previous_second_tick;

static void set_pin(uint8_t *state, unsigned pin, unsigned value)
{
    if (state != NULL && pin < 16U) state[pin] = value ? 1U : 0U;
}

static void set_motor_pins(uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
    set_pin(GPIOA_output, 1U, a1);
    set_pin(GPIOA_output, 2U, a2);
    set_pin(GPIOA_output, 3U, a3);
    set_pin(GPIOA_output, 4U, a4);
#ifdef DISCOBOT_TARGET
    if (a1) GPIO_SetBits(GPIOA, GPIO_Pin_1); else GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    if (a2) GPIO_SetBits(GPIOA, GPIO_Pin_2); else GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    if (a3) GPIO_SetBits(GPIOA, GPIO_Pin_3); else GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    if (a4) GPIO_SetBits(GPIOA, GPIO_Pin_4); else GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#endif
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    if (direc == FORWARD) set_motor_pins(1U, 0U, GPIOA_output[3], GPIOA_output[4]);
    else if (direc == BACKWARD) set_motor_pins(0U, 1U, GPIOA_output[3], GPIOA_output[4]);
    else set_motor_pins(0U, 0U, GPIOA_output[3], GPIOA_output[4]);
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    if (direc == FORWARD) set_motor_pins(GPIOA_output[1], GPIOA_output[2], 1U, 0U);
    else if (direc == BACKWARD) set_motor_pins(GPIOA_output[1], GPIOA_output[2], 0U, 1U);
    else set_motor_pins(GPIOA_output[1], GPIOA_output[2], 0U, 0U);
}

void move_forward(void) { set_motor_pins(1U, 0U, 1U, 0U); }
void move_backward(void) { set_motor_pins(0U, 1U, 0U, 1U); }
void move_forward_soft_left(void) { set_motor_pins(0U, 0U, 1U, 0U); }
void move_forward_soft_right(void) { set_motor_pins(1U, 0U, 0U, 0U); }
void move_backward_soft_left(void) { set_motor_pins(0U, 0U, 0U, 1U); }
void move_backward_soft_right(void) { set_motor_pins(0U, 1U, 0U, 0U); }
void move_spin_right(void) { set_motor_pins(1U, 0U, 0U, 1U); }
void move_spin_left(void) { set_motor_pins(0U, 1U, 1U, 0U); }
void stop(void) { set_motor_pins(0U, 0U, 0U, 0U); }

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left,
    move_forward_soft_right, move_backward_soft_left,
    move_backward_soft_right, move_spin_right, move_spin_left, stop
};

void init_GPIO_A1A2A3A4_output(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef cfg;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    cfg.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    cfg.GPIO_Mode = GPIO_Mode_OUT;
    cfg.GPIO_OType = GPIO_OType_PP;
    cfg.GPIO_PuPd = GPIO_PuPd_UP;
    cfg.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &cfg);
#endif
    set_motor_pins(0U, 0U, 0U, 0U);
}

void init_LED_pins(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef cfg;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    cfg.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    cfg.GPIO_Mode = GPIO_Mode_OUT;
    cfg.GPIO_OType = GPIO_OType_PP;
    cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
    cfg.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &cfg);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
#endif
    for (int i = 0; i < 4; ++i) GPIOD_output[12 + i] = 0U;
}

void LED_On(int i)
{
    if (i < 0 || i >= 4) return;
    GPIOD_output[12 + i] = 1U;
#ifdef DISCOBOT_TARGET
    GPIOD->BSRRL = (uint16_t)(1U << (12 + i));
#endif
}

void LED_Off(int i)
{
    if (i < 0 || i >= 4) return;
    GPIOD_output[12 + i] = 0U;
#ifdef DISCOBOT_TARGET
    GPIOD->BSRRH = (uint16_t)(1U << (12 + i));
#endif
}

void SystemInit(void)
{
    SystemCoreClock = 168000000U;
}

void init_systick(void)
{
    uint32_t reload;
    SystemCoreClockUpdate();
    reload = SystemCoreClock / 1000U;
    if (SysTick_Config(reload) != 0U) {
        SysTick_reload = 0U;
#ifdef DISCOBOT_TARGET
        while (1) { }
#else
        return;
#endif
    }
    SysTick_reload = reload;
}

void init_button(void)
{
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef cfg;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    cfg.GPIO_Pin = GPIO_Pin_0;
    cfg.GPIO_Mode = GPIO_Mode_IN;
    cfg.GPIO_OType = GPIO_OType_PP;
    cfg.GPIO_PuPd = GPIO_PuPd_NOPULL;
    cfg.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &cfg);
#endif
    GPIOA_IDR &= ~1U;
    b_i = 0U;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void discobot_set_button_level(bool high)
{
    if (high) GPIOA_IDR |= 1U; else GPIOA_IDR &= ~1U;
#ifdef DISCOBOT_TARGET
    if (high) GPIO_SetBits(GPIOA, GPIO_Pin_0); else GPIO_ResetBits(GPIOA, GPIO_Pin_0);
#endif
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) return -1;
#ifdef DISCOBOT_TARGET
    return GPIO_ReadInputDataBit(GPIOA, (uint16_t)(1U << i)) != Bit_RESET;
#else
    return (int)((GPIOA_IDR >> i) & 1U);
#endif
}

void SysTick_Handler(void)
{
    ++msTicks;
    if (read_buttonc(0) != 0) {
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
    const float degrees = 180.0f / 3.14159265358979323846f;
    if (roll != NULL) *roll = atan2f(acc_y, acc_z) * degrees;
    if (pitch != NULL) *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * degrees;
}

int init_accelerometers(void)
{
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_LIS3DSH_Sensitivity_2G,
                                    TM_LIS302DL_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    accel_x_mg = x_mg;
    accel_y_mg = y_mg;
    accel_z_mg = z_mg;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) return;
#ifdef DISCOBOT_TARGET
    DiscobotAxes axes;
    TM_LIS302DL_LIS3DSH_ReadAxes(&axes);
    b[0] = (float)axes.X / 1000.0f;
    b[1] = (float)axes.Y / 1000.0f;
    b[2] = (float)axes.Z / 1000.0f;
#else
    b[0] = (float)accel_x_mg / 1000.0f;
    b[1] = (float)accel_y_mg / 1000.0f;
    b[2] = (float)accel_z_mg / 1000.0f;
#endif
}

void init_temperature_sensor(void)
{
#ifdef DISCOBOT_TARGET
    ADC_InitTypeDef adc;
    ADC_CommonInitTypeDef common;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_TempSensorVrefintCmd(ENABLE);
    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_Prescaler = ADC_Prescaler_Div8;
    common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&common);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1U;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1U, ADC_SampleTime_144Cycles);
    ADC_Cmd(ADC1, ENABLE);
#else
    adc_eoc = true;
#endif
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw = raw & 0x0FFFU;
    adc_eoc = true;
}

float read_temperature_sensor(void)
{
    uint16_t raw;
    float temperature;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    raw = ADC_GetConversionValue(ADC1);
#else
    while (!adc_eoc) { }
    raw = adc_raw;
    adc_eoc = false;
#endif
    temperature = (float)raw / 4095.0f;
    temperature *= 3.3f;
    temperature -= 0.760f;
    temperature /= 0.0025f;
    temperature += 25.0f;
    return temperature;
}

void init_rng(void)
{
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#else
    rng_ready = false;
    rng_value = 0U;
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
    unsigned long guard = 1000000UL;
    while (!rng_ready && guard != 0UL) --guard;
    return rng_ready ? rng_value : 0U;
#endif
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    if (myfunc == NULL) return;
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
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
    for (int i = 0; i < MAXNUMTASKS; ++i)
        if (timed_tasks[i].task != NULL)
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
}

void initCircArray(CircArray *arr, int size)
{
    char *p;
    if (arr == NULL || arr->enabled || size <= 0) return;
    p = (char *)calloc((size_t)size, 1U);
    if (p == NULL) {
        arr->buf = NULL;
        arr->size = 0U;
        arr->enabled = false;
        arr->n_r = 0U;
        arr->n_w = 0U;
        return;
    }
    arr->buf = p;
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
    return arr != NULL && arr->size != 0U && !buf_empty(arr) &&
           (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr)
{
    return arr == NULL ? 0 : (int)(arr->n_w - arr->n_r);
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
    char c;
    if (arr == NULL || !arr->enabled || arr->buf == NULL || buf_empty(arr)) return 0;
    c = arr->buf[arr->n_r % arr->size];
    ++arr->n_r;
    return c;
}

bool buf_resize(CircArray *arr, int newSize)
{
    char *p;
    if (arr == NULL || !arr->enabled || arr->buf == NULL || newSize <= 0) return false;
    p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p == NULL) return false;
    arr->buf = p;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr != NULL && arr->buf != NULL && arr->size != 0U)
        memset(arr->buf, 0, arr->size);
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
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef cfg;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);
    cfg.USART_BaudRate = baud == 0 ? UART_BAUD : (uint32_t)baud;
    cfg.USART_WordLength = USART_WordLength_8b;
    cfg.USART_StopBits = USART_StopBits_1;
    cfg.USART_Parity = USART_Parity_No;
    cfg.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    cfg.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &cfg);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART1, ENABLE);
#endif
    if (msg.enabled) buf_delete(&msg);
    memset(&msg, 0, sizeof(msg));
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_baud_used = baud == 0 ? UART_BAUD : (uint32_t)baud;
    if (baud == 0) fprintf(stderr, "init_usart1: baud 0, using 9600\n");
    usart_dr = 0U;
    usart_rxne = false;
    usart_txe_ready = true;
}

void USART1_IRQHandler(void)
{
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        (void)buf_putbyte(&msg, (char)USART_ReceiveData(USART1));
#else
    if (usart_rxne) {
        (void)buf_putbyte(&msg, (char)usart_dr);
        usart_rxne = false;
    }
#endif
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
#ifndef DISCOBOT_TARGET
    usart_dr = value;
    usart_rxne = rxne;
    if (rxne) USART1_IRQHandler();
#else
    (void)value;
    (void)rxne;
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) return;
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40U) == 0U) { }
        USART_SendData(USART1, (uint16_t)(unsigned char)*s++);
#else
        while (!usart_txe_ready) { }
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1U) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }
        ++s;
#endif
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    unsigned char c = (unsigned char)buf_getbyte(&msg);
    return c == 0xFFU ? (char)-1 : (char)c;
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

bool dispatch_uart_command(int command)
{
    if (command < 0 || command >= MAXIDSIZE) return false;
    msgid = (_ID)command;
    callme = flookup[command];
    if (callme != NULL) callme();
    return true;
}

void main_loop_iteration(void)
{
    if (usart1_available() != 0U) {
        char c = usart1_readc();
        if ((signed char)c >= 0)
            (void)dispatch_uart_command((int)(unsigned char)c);
    }
    checkbutton();
    execute_tasks();
    if (msTicks - previous_second_tick > 1000U)
        previous_second_tick = msTicks;
}

void init_system(void)
{
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
}

int func2(int R0)
{
    uint32_t value = (uint32_t)R0;
    return (int)(value + 1U);
}

int func1(int R0)
{
    return func2(R0);
}