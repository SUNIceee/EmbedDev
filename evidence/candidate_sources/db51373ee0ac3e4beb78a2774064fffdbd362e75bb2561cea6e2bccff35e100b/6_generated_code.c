#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(DISCOBOT_TARGET)
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rng.h"
#include "misc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

#define DISCOBOT_PI 3.14159265358979323846f
#define MOTOR_PIN1 (1u << 1u)
#define MOTOR_PIN2 (1u << 2u)
#define MOTOR_PIN3 (1u << 3u)
#define MOTOR_PIN4 (1u << 4u)

volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = NULL;
motor_command_fn flookup[MAXIDSIZE] = {
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
uint32_t SystemCoreClock = 168000000UL;
uint32_t SysTick_reload = 0u;
uint32_t usart1_baud_used = 0u;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

#if !defined(DISCOBOT_TARGET)
static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000UL;
}
#endif

static CircArray msg;
static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0u;
static int16_t discobot_accel_raw[3] = {0, 0, 0};
static uint16_t discobot_adc_raw = 0u;
static volatile bool discobot_rng_ready = false;
static volatile uint32_t discobot_rng_value = 0u;
#if !defined(DISCOBOT_TARGET)
static volatile int discobot_systick_failed = 0;
#endif

#if !defined(DISCOBOT_TARGET)
static int SysTick_Config(uint32_t ticks)
{
    if (ticks > 0x00FFFFFFu) {
        discobot_systick_failed = 1;
        SysTick_reload = 0u;
        return 1;
    }
    SysTick_reload = ticks;
    return 0;
}
#endif

static void motor_write_pin(uint32_t pin, uint8_t value, uint8_t *observable)
{
    *observable = value;
#if defined(DISCOBOT_TARGET)
    if (value != 0u) {
        GPIOA->BSRRL = pin;
    } else {
        GPIOA->BSRRH = pin;
    }
#else
    (void)pin;
#endif
}

void SystemInit(void)
{
#if defined(DISCOBOT_TARGET)
    RCC->CR |= 0x00010000u;
    while ((RCC->CR & 0x00020000u) == 0u) {}

    RCC->APB1ENR |= 0x10000000u;
    PWR->CR = (PWR->CR & ~0xC000u) | 0xC000u;
    while ((PWR->CSR & 0x4000u) == 0u) {}

    FLASH->ACR = (5u << 0u) | (1u << 8u) | (1u << 9u) | (1u << 10u);

    RCC->PLLCFGR = (8u << 0u) | (336u << 6u) | (0u << 16u) | (1u << 22u) | (7u << 24u);
    RCC->CR |= 0x01000000u;
    while ((RCC->CR & 0x02000000u) == 0u) {}

    RCC->CFGR = (0x5u << 10u) | (0x4u << 13u);
    RCC->CFGR = (RCC->CFGR & ~0x3u) | 0x2u;
    while ((RCC->CFGR & 0xCu) != 0x8u) {}

    SystemCoreClock = 168000000UL;
#else
    SystemCoreClock = 168000000UL;
#endif
}

void init_systick(void)
{
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000u;
#if defined(DISCOBOT_TARGET)
    if (SysTick_Config(SysTick_reload) != 0) {
        while (1) {}
    }
#else
    if (SysTick_Config(SysTick_reload) != 0) {
        discobot_systick_failed = 1;
        return;
    }
#endif
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
    init_usart1((int)UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void init_button(void)
{
#if defined(DISCOBOT_TARGET)
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);
#else
    GPIOA_IDR &= ~(1u << 0u);
#endif
    b_i = 0u;
    buttstate = ButtonIsReleased;
}

void init_LED_pins(void)
{
#if defined(DISCOBOT_TARGET)
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &gpio);
    GPIOD->BSRRH = gpio.GPIO_Pin;
#else
    GPIOD_output[12] = 0u;
    GPIOD_output[13] = 0u;
    GPIOD_output[14] = 0u;
    GPIOD_output[15] = 0u;
#endif
}

void init_GPIO_A1A2A3A4_output(void)
{
#if defined(DISCOBOT_TARGET)
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIOA->BSRRH = gpio.GPIO_Pin;
#else
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
#endif
}

void set_left_motor_direc(int direc, float speed)
{
    uint8_t pa1 = 0u;
    uint8_t pa2 = 0u;
    (void)speed;

    switch (direc) {
        case FORWARD:
            pa1 = 1u;
            pa2 = 0u;
            break;
        case BACKWARD:
            pa1 = 0u;
            pa2 = 1u;
            break;
        case STOP:
        default:
            pa1 = 0u;
            pa2 = 0u;
            break;
    }

    motor_write_pin(MOTOR_PIN1, pa1, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, pa2, &GPIOA_output[2]);
}

void set_right_motor_direc(int direc, float speed)
{
    uint8_t pa3 = 0u;
    uint8_t pa4 = 0u;
    (void)speed;

    switch (direc) {
        case FORWARD:
            pa3 = 1u;
            pa4 = 0u;
            break;
        case BACKWARD:
            pa3 = 0u;
            pa4 = 1u;
            break;
        case STOP:
        default:
            pa3 = 0u;
            pa4 = 0u;
            break;
    }

    motor_write_pin(MOTOR_PIN3, pa3, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, pa4, &GPIOA_output[4]);
}

void move_forward(void)
{
    motor_write_pin(MOTOR_PIN1, 1u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 1u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

void move_backward(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 1u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 1u, &GPIOA_output[4]);
}

void move_forward_soft_left(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 1u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

void move_forward_soft_right(void)
{
    motor_write_pin(MOTOR_PIN1, 1u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

void move_backward_soft_left(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 1u, &GPIOA_output[4]);
}

void move_backward_soft_right(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 1u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

void move_spin_right(void)
{
    motor_write_pin(MOTOR_PIN1, 1u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 1u, &GPIOA_output[4]);
}

void move_spin_left(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 1u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 1u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

void stop(void)
{
    motor_write_pin(MOTOR_PIN1, 0u, &GPIOA_output[1]);
    motor_write_pin(MOTOR_PIN2, 0u, &GPIOA_output[2]);
    motor_write_pin(MOTOR_PIN3, 0u, &GPIOA_output[3]);
    motor_write_pin(MOTOR_PIN4, 0u, &GPIOA_output[4]);
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
#if defined(DISCOBOT_TARGET)
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
        switch (buttstate) {
            case ButtonIsReleased:
                break;
            case ButtonIsPressed:
                break;
            default:
                break;
        }
        laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    if (pitch != NULL && roll != NULL) {
        float yz = sqrtf((acc_y * acc_y) + (acc_z * acc_z));
        *roll = atan2f(acc_y, acc_z) * 180.0f / DISCOBOT_PI;
        *pitch = atan2f(-acc_x, yz) * 180.0f / DISCOBOT_PI;
    }
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= (1u << 0u);
    } else {
        GPIOA_IDR &= ~(1u << 0u);
    }
}

void LED_On(int i)
{
    if (i >= 0 && i <= 3) {
#if defined(DISCOBOT_TARGET)
        GPIOD->BSRRL = (1u << (12u + (uint32_t)i));
#endif
        GPIOD_output[12 + i] = 1u;
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i <= 3) {
#if defined(DISCOBOT_TARGET)
        GPIOD->BSRRH = (1u << (12u + (uint32_t)i));
#endif
        GPIOD_output[12 + i] = 0u;
    }
}

int init_accelerometers(void)
{
#if defined(DISCOBOT_TARGET)
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
#if defined(DISCOBOT_TARGET)
    TM_LIS302DL_LIS3DSH_Axes_t raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
#else
    b[0] = discobot_accel_raw[0] / 1000.0f;
    b[1] = discobot_accel_raw[1] / 1000.0f;
    b[2] = discobot_accel_raw[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    discobot_accel_raw[0] = x_mg;
    discobot_accel_raw[1] = y_mg;
    discobot_accel_raw[2] = z_mg;
}

void init_temperature_sensor(void)
{
#if defined(DISCOBOT_TARGET)
    ADC_CommonInitTypeDef adc_common;
    ADC_InitTypeDef adc_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    adc_init.ADC_Resolution = ADC_Resolution_12b;
    adc_init.ADC_ScanConvMode = DISABLE;
    adc_init.ADC_ContinuousConvMode = ENABLE;
    adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    adc_init.ADC_DataAlign = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc_init);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#else
    (void)0;
#endif
}

float read_temperature_sensor(void)
{
#if defined(DISCOBOT_TARGET)
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    uint16_t raw = ADC_GetConversionValue(ADC1);
#else
    uint16_t raw = discobot_adc_raw;
#endif
    float temp = (float)raw / 4095.0f;
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
#if defined(DISCOBOT_TARGET)
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#else
    (void)0;
#endif
}

uint32_t get_random_number(void)
{
#if defined(DISCOBOT_TARGET)
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {}
    return RNG_GetRandomNumber();
#else
    while (!discobot_rng_ready) {
    }
    return discobot_rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value)
{
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    uint32_t interval_ms = (uint32_t)(long)(interval_sec * 1000.0f);
    int i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = interval_ms;
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            break;
        }
    }
}

void execute_tasks(void)
{
    int i;

    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL &&
            msTicks >= (timed_tasks[i].last_called + timed_tasks[i].msinterval)) {
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
            printf("t%d=%lu", i, (unsigned long)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size)
{
    char *new_buf;

    if (arr == NULL || size <= 0) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }
    new_buf = (char *)calloc((size_t)size, 1u);
    if (new_buf == NULL) {
        return;
    }
    arr->buf = new_buf;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled) {
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
    char c;

    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled) {
        return 0;
    }
    if (!buf_empty(arr)) {
        c = arr->buf[arr->n_r % arr->size];
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
    char *new_buf;

    if (arr == NULL || newSize <= 0) {
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
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr != NULL && arr->buf != NULL && arr->size > 0u) {
        memset(arr->buf, 0, arr->size);
    }
}

void init_usart1(int baud)
{
    int effective_baud = baud;

    if (effective_baud == 0) {
        effective_baud = (int)UART_BAUD;
        fprintf(stderr, "warning: baud 0 defaults to 9600\n");
    }
    usart1_baud_used = (uint32_t)effective_baud;

#if defined(DISCOBOT_TARGET)
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;

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

    usart_init.USART_BaudRate = (uint32_t)effective_baud;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart_init);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic_init.NVIC_IRQChannel = USART1_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
#endif

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

#if defined(DISCOBOT_TARGET)
    USART_Cmd(USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void)
{
#if defined(DISCOBOT_TARGET)
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = (char)USART1->DR;
        (void)buf_putbyte(&msg, c);
    }
#else
    (void)0;
#endif
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
#if defined(DISCOBOT_TARGET)
        while ((USART1->SR & 0x40u) == 0u) {}
        USART_SendData(USART1, (uint16_t)(unsigned char)*s);
#else
        if (usart1_tx_length < (sizeof(usart1_tx_log) - 1u)) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }
#endif
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
    if (rxne) {
        (void)buf_putbyte(&msg, (char)value);
    }
}

bool dispatch_uart_command(int command)
{
    if (command >= (int)FWD && command <= (int)STOPCAR) {
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
    if (usart1_available() > 0u) {
        int c = (signed char)usart1_readc();
        if (c >= (int)FWD && c <= (int)STOPCAR) {
            (void)dispatch_uart_command(c);
        }
    }

    checkbutton();
    execute_tasks();

    if ((msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }
}

int func1(int R0)
{
    return func2(R0);
}

int func2(int R0)
{
    uint32_t u = (uint32_t)R0 + 1u;
    return (int)u;
}