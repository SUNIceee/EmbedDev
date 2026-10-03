#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
#define ENABLE 1
#define RESET 0
#define RCC_AHB1Periph_GPIOA 0x00000001U
#define RCC_AHB1Periph_GPIOB 0x00000002U
#define RCC_AHB1Periph_GPIOD 0x00000008U
#define RCC_AHB2Periph_RNG 0x00000040U
#define RCC_APB2Periph_USART1 0x00000010U
#define RCC_APB2Periph_ADC1 0x00000100U
#define GPIO_Pin_0 0x0001U
#define GPIO_Pin_1 0x0002U
#define GPIO_Pin_2 0x0004U
#define GPIO_Pin_3 0x0008U
#define GPIO_Pin_4 0x0010U
#define GPIO_Pin_6 0x0040U
#define GPIO_Pin_7 0x0080U
#define GPIO_Pin_12 0x1000U
#define GPIO_Pin_13 0x2000U
#define GPIO_Pin_14 0x4000U
#define GPIO_Pin_15 0x8000U
#define GPIO_Mode_IN 0U
#define GPIO_Mode_OUT 1U
#define GPIO_Mode_AF 2U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_NOPULL 0U
#define GPIO_PuPd_UP 1U
#define GPIO_Speed_25MHz 25U
#define GPIO_Speed_100MHz 100U
#define GPIO_AF_USART1 7U
#define USART_IT_RXNE 0x0525U
#define USART_Mode_Tx 0x0008U
#define USART_Mode_Rx 0x0004U
#define USART_WordLength_8b 0U
#define USART_StopBits_1 0U
#define USART_Parity_No 0U
#define USART_HardwareFlowControl_None 0U
#define USART1_IRQn 37U
#define ADC_Mode_Independent 0U
#define ADC_Prescaler_Div8 8U
#define ADC_DMAAccessMode_Disabled 0U
#define ADC_Resolution_12b 12U
#define ADC_ExternalTrigConvEdge_None 0U
#define ADC_DataAlign_Right 0U
#define ADC_Channel_TempSensor 16U
#define ADC_SampleTime_144Cycles 144U
#define ADC_FLAG_EOC 0x02U
#define RNG_FLAG_DRDY 1U
#define TM_LIS3DSH_Sensitivity_2G 0U
#define TM_LIS3DSH_Filter_50Hz 0U
typedef struct { uint32_t BSRRL; uint32_t BSRRH; uint32_t IDR; } DiscobotGpioRegs;
typedef struct { uint32_t SR; uint32_t DR; } DiscobotUsartRegs;
typedef struct { uint32_t GPIO_Pin; uint32_t GPIO_Mode; uint32_t GPIO_Speed; uint32_t GPIO_OType; uint32_t GPIO_PuPd; } GPIO_InitTypeDef;
typedef struct { uint32_t USART_BaudRate; uint32_t USART_WordLength; uint32_t USART_StopBits; uint32_t USART_Parity; uint32_t USART_HardwareFlowControl; uint32_t USART_Mode; } USART_InitTypeDef;
typedef struct { uint8_t NVIC_IRQChannel; uint8_t NVIC_IRQChannelPreemptionPriority; uint8_t NVIC_IRQChannelSubPriority; uint8_t NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
typedef struct { uint32_t ADC_Mode; uint32_t ADC_Prescaler; uint32_t ADC_DMAAccessMode; uint32_t ADC_TwoSamplingDelay; } ADC_CommonInitTypeDef;
typedef struct { uint32_t ADC_Resolution; uint32_t ADC_ScanConvMode; uint32_t ADC_ContinuousConvMode; uint32_t ADC_ExternalTrigConvEdge; uint32_t ADC_ExternalTrigConv; uint32_t ADC_DataAlign; uint32_t ADC_NbrOfConversion; } ADC_InitTypeDef;
typedef struct { int16_t X; int16_t Y; int16_t Z; } TM_LIS302DL_LIS3DSH_t;
extern DiscobotGpioRegs *GPIOA;
extern DiscobotGpioRegs *GPIOB;
extern DiscobotGpioRegs *GPIOD;
extern DiscobotUsartRegs *USART1;
extern void *ADC1;
extern void SystemCoreClockUpdate(void);
extern int SysTick_Config(uint32_t ticks);
extern void RCC_AHB1PeriphClockCmd(uint32_t periph, int enable);
extern void RCC_AHB2PeriphClockCmd(uint32_t periph, int enable);
extern void RCC_APB2PeriphClockCmd(uint32_t periph, int enable);
extern void GPIO_Init(void *port, GPIO_InitTypeDef *init);
extern void GPIO_PinAFConfig(void *port, uint16_t pin_source, uint8_t af);
extern void NVIC_Init(NVIC_InitTypeDef *init);
extern void USART_Init(void *usart, USART_InitTypeDef *init);
extern void USART_ITConfig(void *usart, uint16_t it, int enable);
extern void USART_Cmd(void *usart, int enable);
extern void USART_SendData(void *usart, uint16_t data);
extern void ADC_CommonInit(ADC_CommonInitTypeDef *init);
extern void ADC_Init(void *adc, ADC_InitTypeDef *init);
extern void ADC_RegularChannelConfig(void *adc, uint8_t channel, uint8_t rank, uint8_t sample_time);
extern void ADC_TempSensorVrefintCmd(int enable);
extern void ADC_Cmd(void *adc, int enable);
extern void ADC_SoftwareStartConv(void *adc);
extern int ADC_GetFlagStatus(void *adc, uint8_t flag);
extern uint16_t ADC_GetConversionValue(void *adc);
extern void RNG_Cmd(int enable);
extern int RNG_GetFlagStatus(uint8_t flag);
extern uint32_t RNG_GetRandomNumber(void);
extern int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
extern void TM_LIS302DL_LIS3DSH_ReadAxes(TM_LIS302DL_LIS3DSH_t *axes);
#endif

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000U; }
static int SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return 0; }
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = SYSTICK_PERIOD_TICKS;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

static CircArray msg = {0};
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_t_prev = 0;
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t adc_raw_value = 0;
static bool rng_ready_flag = true;
static uint32_t rng_value_word = 0;
static uint8_t usart1_rxne_flag = 0;
static uint8_t usart1_dr_value = 0;
static uint32_t usart1_sr_value = 0x40U;
static uint8_t usart1_nvic_preempt_priority = 0U;
static uint8_t usart1_nvic_sub_priority = 0U;
static uint8_t usart1_nvic_irq_channel_cmd = 0U;

TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static void set_motor_pin(unsigned pin, uint8_t value) {
    if (pin < 16U) {
        GPIOA_output[pin] = value ? 1U : 0U;
    }
#ifdef DISCOBOT_TARGET
    if (GPIOA != NULL) {
        if (value) { GPIOA->BSRRL = (uint32_t)(1UL << pin); }
        else { GPIOA->BSRRH = (uint32_t)(1UL << pin); }
    }
#endif
}

static void set_motor_pattern(uint8_t pa1, uint8_t pa2, uint8_t pa3, uint8_t pa4) {
    set_motor_pin(1U, pa1);
    set_motor_pin(2U, pa2);
    set_motor_pin(3U, pa3);
    set_motor_pin(4U, pa4);
}

static void set_led_pin(int i, uint8_t value) {
    int pin = 12 + i;
    GPIOD_output[pin] = value ? 1U : 0U;
#ifdef DISCOBOT_TARGET
    if (GPIOD != NULL) {
        if (value) { GPIOD->BSRRL = (uint32_t)(1UL << (unsigned)pin); }
        else { GPIOD->BSRRH = (uint32_t)(1UL << (unsigned)pin); }
    }
#endif
}

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, move_forward_soft_right,
    move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop
};
motor_command_fn callme = 0;

void SystemInit(void) { SystemCoreClock = 168000000U; }

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U) != 0) {
        while (1) { }
    }
}

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);
#endif
    GPIOD_output[12] = 0U; GPIOD_output[13] = 0U; GPIOD_output[14] = 0U; GPIOD_output[15] = 0U;
#ifdef DISCOBOT_TARGET
    if (GPIOD != NULL) { GPIOD->BSRRH = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; }
#endif
}

void init_button(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
#endif
    GPIOA_IDR &= ~1U;
    b_i = 0U;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void init_system(void) {
    SystemInit(); init_systick(); init_LED_pins(); init_button(); init_accelerometers(); init_rng();
    init_temperature_sensor(); init_GPIO_A1A2A3A4_output(); init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme != 0) { callme(); }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    uint32_t now = msTicks;
    if ((uint32_t)(now - main_t_prev) > 1000U) { main_t_prev = now; }
    if (usart1_available() > 0U) { char c = usart1_readc(); dispatch_uart_command((int)c); }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
#endif
    set_motor_pattern(0U, 0U, 0U, 0U);
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { set_motor_pin(1U, 1U); set_motor_pin(2U, 0U); }
    else if (direc == BACKWARD) { set_motor_pin(1U, 0U); set_motor_pin(2U, 1U); }
    else { set_motor_pin(1U, 0U); set_motor_pin(2U, 0U); }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { set_motor_pin(3U, 1U); set_motor_pin(4U, 0U); }
    else if (direc == BACKWARD) { set_motor_pin(3U, 0U); set_motor_pin(4U, 1U); }
    else { set_motor_pin(3U, 0U); set_motor_pin(4U, 0U); }
}

void move_forward(void) { set_motor_pattern(1U, 0U, 1U, 0U); }
void move_backward(void) { set_motor_pattern(0U, 1U, 0U, 1U); }
void move_forward_soft_left(void) { set_motor_pattern(0U, 0U, 1U, 0U); }
void move_forward_soft_right(void) { set_motor_pattern(1U, 0U, 0U, 0U); }
void move_backward_soft_left(void) { set_motor_pattern(0U, 0U, 0U, 1U); }
void move_backward_soft_right(void) { set_motor_pattern(0U, 1U, 0U, 0U); }
void move_spin_right(void) { set_motor_pattern(1U, 0U, 0U, 1U); }
void move_spin_left(void) { set_motor_pattern(0U, 1U, 1U, 0U); }
void stop(void) { set_motor_pattern(0U, 0U, 0U, 0U); }

int read_buttonc(int i) {
    if (i > 3 || i < 0) { return -1; }
#ifdef DISCOBOT_TARGET
    if (GPIOA != NULL) { GPIOA_IDR = GPIOA->IDR; }
#endif
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1U);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) { buttstate = ButtonIsPressed; }
    } else {
        b_i = 0U;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (last_button_state != buttstate) {
        switch (buttstate) { case ButtonIsPressed: break; case ButtonIsReleased: break; default: break; }
        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;
    if (roll != NULL) { *roll = atan2f(acc_y, acc_z) * 180.0f / PI; }
    if (pitch != NULL) { *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI; }
}

void discobot_set_button_level(bool high) { if (high) { GPIOA_IDR |= 1U; } else { GPIOA_IDR &= ~1U; } }

void LED_On(int i) { if (i >= 0 && i < 4) { set_led_pin(i, 1U); } }
void LED_Off(int i) { if (i >= 0 && i < 4) { set_led_pin(i, 0U); } }

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void read_accelerometers(float b[3]) {
    if (b == NULL) { return; }
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_t axes;
    TM_LIS302DL_LIS3DSH_ReadAxes(&axes);
    b[0] = ((float)axes.X) / 1000.0f;
    b[1] = ((float)axes.Y) / 1000.0f;
    b[2] = ((float)axes.Z) / 1000.0f;
#else
    b[0] = ((float)accel_raw_mg[0]) / 1000.0f;
    b[1] = ((float)accel_raw_mg[1]) / 1000.0f;
    b[2] = ((float)accel_raw_mg[2]) / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg; accel_raw_mg[1] = y_mg; accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    ADC_CommonInitTypeDef common;
    ADC_InitTypeDef adc;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_Prescaler = ADC_Prescaler_Div8;
    common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    common.ADC_TwoSamplingDelay = 0U;
    ADC_CommonInit(&common);
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_ScanConvMode = 0U;
    adc.ADC_ContinuousConvMode = ENABLE;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = 0U;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfConversion = 1U;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1U, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#endif
}

float read_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    uint16_t raw;
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    raw = ADC_GetConversionValue(ADC1);
#else
    uint16_t raw = adc_raw_value;
#endif
    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) { adc_raw_value = raw; }

void init_rng(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) { }
    return RNG_GetRandomNumber();
#else
    if (!rng_ready_flag) {
        abort();
    }
    return rng_value_word;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) { rng_ready_flag = ready; rng_value_word = value; }

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (uint32_t i = 0U; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0U;
            timed_tasks[i].numcalls = 0U;
            break;
        }
    }
}

void execute_tasks(void) {
    for (uint32_t i = 0U; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void) {
    for (uint32_t i = 0U; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) { printf("t%d=%d", (int)i, (int)timed_tasks[i].numcalls); }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) { return; }
    if (arr->enabled) { printf("CircArray already enabled"); return; }
    arr->buf = (char *)calloc((size_t)size, 1U);
    if (arr->buf == NULL) { arr->size = 0U; arr->enabled = false; arr->n_r = 0U; arr->n_w = 0U; return; }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0U;
    arr->n_w = 0U;
}

bool buf_empty(CircArray *arr) { if (arr == NULL) { return true; } return arr->n_r == arr->n_w; }

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0U) { return false; }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) { return 0; }
    if (!buf_full(arr)) { arr->buf[arr->n_w % arr->size] = c; arr->n_w++; return 1; }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    char c;
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) { return 0; }
    if (!buf_empty(arr)) { c = arr->buf[arr->n_r % arr->size]; arr->n_r++; return c; }
    return 0;
}

int buf_available(CircArray *arr) { if (arr == NULL) { return 0; } return (int)(arr->n_w - arr->n_r); }

bool buf_resize(CircArray *arr, int newSize) {
    void *newbuf;
    if (arr == NULL || newSize <= 0) { return false; }
    newbuf = realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) { return false; }
    arr->buf = (char *)newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) { if (arr != NULL && arr->buf != NULL && arr->size != 0U) { memset(arr->buf, 0, (size_t)arr->size); } }

bool buf_delete(CircArray *arr) {
    if (arr == NULL) { return false; }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0U;
    arr->n_r = 0U;
    arr->n_w = 0U;
    arr->enabled = false;
    return true;
}

void init_usart1(int baud) {
    if (baud == 0) { baud = 9600; }
    usart1_baud_used = (uint32_t)baud;
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, 6U, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, 7U, GPIO_AF_USART1);
    usart.USART_BaudRate = (uint32_t)baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART1, ENABLE);
#endif
    usart1_nvic_preempt_priority = 0U;
    usart1_nvic_sub_priority = 0U;
    usart1_nvic_irq_channel_cmd = 1U;
    if (msg.enabled) { (void)buf_delete(&msg); }
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_sr_value = 0x40U;
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART1 != NULL && (USART1->SR & 0x20U) != 0U) {
        char c = (char)(uint8_t)USART1->DR;
        (void)buf_putbyte(&msg, c);
        return;
    }
#endif
    if (usart1_rxne_flag) {
        char c = (char)usart1_dr_value;
        (void)buf_putbyte(&msg, c);
        usart1_rxne_flag = 0U;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) { return; }
    while (*s) {
#ifdef DISCOBOT_TARGET
        while (USART1 != NULL && !(USART1->SR & 0x40U)) { }
        USART_SendData(USART1, (uint16_t)(uint8_t)(*s));
#else
        while (!(usart1_sr_value & 0x40U)) { }
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log)) { usart1_tx_log[usart1_tx_length++] = (char)*s; }
        s++;
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) { usart1_dr_value = value; usart1_rxne_flag = 1U; USART1_IRQHandler(); }
}

int func2(int R0) { uint32_t v = (uint32_t)R0; v += 1U; return (int)v; }
int func1(int R0) { return func2(R0); }