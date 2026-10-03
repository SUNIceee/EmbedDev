#include "6_generated_code.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

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

#define DISCOBOT_PI 3.14159265358979323846f

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
uint32_t SystemCoreClock = 0;
#endif
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
static char discobot_startup_banner_array[] = "UART1 Initialized. @9600bps\r\n";
const char *discobot_startup_banner = discobot_startup_banner_array;
TimedTask timed_tasks[MAXNUMTASKS] = {0};

static CircArray msg;

static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0;
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t adc_raw = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;
#ifndef DISCOBOT_TARGET
static bool host_usart_rxne = false;
static uint8_t host_usart_dr = 0;
#endif

#ifndef DISCOBOT_TARGET
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;
typedef enum { RESET = 0, SET = 1 } FlagStatus;
typedef struct {
    uint32_t GPIO_Pin;
    uint32_t GPIO_Mode;
    uint32_t GPIO_OType;
    uint32_t GPIO_PuPd;
    uint32_t GPIO_Speed;
    uint32_t GPIO_AF;
} GPIO_InitTypeDef;
typedef struct {
    uint32_t USART_BaudRate;
    uint32_t USART_WordLength;
    uint32_t USART_StopBits;
    uint32_t USART_Parity;
    uint32_t USART_Mode;
    uint32_t USART_HardwareFlowControl;
} USART_InitTypeDef;
typedef struct {
    uint32_t ADC_Resolution;
    uint32_t ADC_ScanConvMode;
    uint32_t ADC_ContinuousConvMode;
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_DataAlign;
    uint32_t ADC_NbrOfConversion;
} ADC_InitTypeDef;
typedef struct {
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;
typedef int GPIO_TypeDef;
typedef int USART_TypeDef;
typedef int ADC_TypeDef;
typedef int RNG_TypeDef;
#define RCC_AHB1Periph_GPIOA 0x0001u
#define RCC_AHB1Periph_GPIOB 0x0002u
#define RCC_AHB1Periph_GPIOD 0x0008u
#define RCC_APB2Periph_ADC1 0x0100u
#define RCC_APB2Periph_USART1 0x0010u
#define RCC_AHB2Periph_RNG 0x0040u
#define GPIOA ((GPIO_TypeDef*)1)
#define GPIOB ((GPIO_TypeDef*)2)
#define GPIOD ((GPIO_TypeDef*)3)
#define ADC1 ((ADC_TypeDef*)1)
#define USART1 ((USART_TypeDef*)1)
#define RNG ((RNG_TypeDef*)1)
#define ADC_FLAG_EOC 1u
#define USART_IT_RXNE 1u
#define RNG_FLAG_DRDY 1u
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 0
static void RCC_AHB1PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
static void RCC_APB2PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
static void RCC_AHB2PeriphClockCmd(uint32_t p, FunctionalState s) { (void)p; (void)s; }
static void GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
static void USART_Init(USART_TypeDef *p, USART_InitTypeDef *i) { (void)p; (void)i; }
static void USART_Cmd(USART_TypeDef *p, FunctionalState s) { (void)p; (void)s; }
static void USART_ITConfig(USART_TypeDef *p, uint32_t it, FunctionalState s) { (void)p; (void)it; (void)s; }
static void NVIC_Init(NVIC_InitTypeDef *n) { (void)n; }
static void ADC_Init(ADC_TypeDef *p, ADC_InitTypeDef *i) { (void)p; (void)i; }
static void ADC_RegularChannelConfig(ADC_TypeDef *p, uint32_t ch, uint32_t rank, uint32_t sample) { (void)p; (void)ch; (void)rank; (void)sample; }
static void ADC_TempSensorVrefintCmd(FunctionalState s) { (void)s; }
static void ADC_Cmd(ADC_TypeDef *p, FunctionalState s) { (void)p; (void)s; }
static void ADC_SoftwareStartConv(ADC_TypeDef *p) { (void)p; }
static FlagStatus ADC_GetFlagStatus(ADC_TypeDef *p, uint32_t flag) { (void)p; (void)flag; return SET; }
static uint16_t ADC_GetConversionValue(ADC_TypeDef *p) { (void)p; return adc_raw; }
static void RNG_Cmd(FunctionalState s) { (void)s; }
static FlagStatus RNG_GetFlagStatus(uint32_t flag) { (void)flag; return rng_ready ? SET : RESET; }
static uint32_t RNG_GetRandomNumber(void) { return rng_value; }
static void SystemCoreClockUpdate(void) { }
static int SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return 0; }
#endif

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
    usart1_send((volatile char *)discobot_startup_banner_array);
}

#ifndef DISCOBOT_TARGET
void SystemInit(void) {
    SystemCoreClock = 168000000;
}
#endif

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000) != 0) {
        while (1) {}
    }
    SysTick_reload = SystemCoreClock / 1000;
}

#ifdef DISCOBOT_TARGET
void init_button(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}
#else
void init_button(void) {
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    memset(&gpio, 0, sizeof(gpio));
    GPIO_Init(GPIOA, &gpio);
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}
#endif

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        if (callme != 0) {
            callme();
        }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        int c = (int)usart1_readc();
        (void)dispatch_uart_command(c);
    }
    checkbutton();
    execute_tasks();
    if ((msTicks - t_prev) > 1000) {
        t_prev = msTicks;
    }
}

void init_GPIO_A1A2A3A4_output(void) {
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
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
#else
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    memset(&gpio, 0, sizeof(gpio));
    GPIO_Init(GPIOA, &gpio);
#endif
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[1] = 1;
        GPIOA_output[2] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 1;
    } else {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[3] = 1;
        GPIOA_output[4] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 1;
    } else {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
    }
}

void move_forward(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void move_backward(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
}

void stop(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
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
    if (laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
        laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    float denom;
    if (pitch == 0 || roll == 0) {
        return;
    }
    denom = sqrtf(acc_y * acc_y + acc_z * acc_z);
    *pitch = atan2f(-acc_x, denom) * 180.0f / DISCOBOT_PI;
    *roll = atan2f(acc_y, acc_z) * 180.0f / DISCOBOT_PI;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_LED_pins(void) {
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
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
#else
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    memset(&gpio, 0, sizeof(gpio));
    GPIO_Init(GPIOD, &gpio);
#endif
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i) {
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 1;
}

void LED_Off(int i) {
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 0;
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void read_accelerometers(float b[3]) {
    if (b == 0) {
        return;
    }
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_AxesRaw raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
#else
    b[0] = accel_raw_mg[0] / 1000.0f;
    b[1] = accel_raw_mg[1] / 1000.0f;
    b[2] = accel_raw_mg[2] / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg;
    accel_raw_mg[1] = y_mg;
    accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    ADC_InitTypeDef adc;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_StructInit(&adc);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConv = DISABLE;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#else
    ADC_InitTypeDef adc;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    memset(&adc, 0, sizeof(adc));
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, 1u, 1u, 0u);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#endif
}

float read_temperature_sensor(void) {
    float temp;
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    temp = (float)ADC_GetConversionValue(ADC1);
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
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}

uint32_t get_random_number(void) {
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {}
    return RNG_GetRandomNumber();
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == 0) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != 0 && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void) {
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != 0) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    char *p;
    if (arr == 0) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "CircArray already initialized\n");
        return;
    }
    if (size <= 0) {
        return;
    }
    p = (char *)calloc((size_t)size, 1);
    if (p == 0) {
        return;
    }
    arr->buf = p;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == 0 || arr->buf == 0 || arr->size == 0 || !arr->enabled) {
        return 0;
    }
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    char c;
    if (arr == 0 || arr->buf == 0 || arr->size == 0 || !arr->enabled) {
        return 0;
    }
    if (!buf_empty(arr)) {
        c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
}

bool buf_empty(CircArray *arr) {
    if (arr == 0) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == 0 || arr->size == 0) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (arr == 0) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    char *tmp;
    if (arr == 0 || arr->buf == 0 || newSize <= 0) {
        return false;
    }
    tmp = (char *)realloc(arr->buf, (size_t)newSize);
    if (tmp == 0) {
        return false;
    }
    arr->buf = tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == 0) {
        return false;
    }
    if (arr->buf != 0) {
        buf_clear(arr);
        free(arr->buf);
    }
    arr->buf = 0;
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == 0 || arr->buf == 0) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
    }
    usart1_baud_used = (uint32_t)baud;
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
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
#else
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    memset(&gpio, 0, sizeof(gpio));
    memset(&usart, 0, sizeof(usart));
    memset(&nvic, 0, sizeof(nvic));
    GPIO_Init(GPIOB, &gpio);
    USART_Init(USART1, &usart);
    NVIC_Init(&nvic);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, (char)c);
    }
#else
    if (host_usart_rxne) {
        (void)buf_putbyte(&msg, (char)host_usart_dr);
        host_usart_rxne = false;
    }
#endif
}

void usart1_send(volatile char *s) {
    if (s == 0) {
        return;
    }
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40) == 0) {}
        USART_SendData(USART1, (uint16_t)(unsigned char)*s);
#else
        /* Host model: append to the observable TX log. */
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
        }
        s++;
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
#ifdef DISCOBOT_TARGET
    (void)value;
    (void)rxne;
#else
    host_usart_dr = value;
    host_usart_rxne = rxne;
    if (rxne) {
        USART1_IRQHandler();
    }
#endif
}

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}