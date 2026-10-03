#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
volatile _ID msgid = FWD;
motor_command_fn callme = NULL;
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
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {0};
static ButtonState laststate = ButtonIsReleased;
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t adc_raw_temperature = 0;
static volatile bool rng_ready = false;
static volatile uint32_t rng_value = 0;
static uint8_t usart1_host_dr = 0;
static bool usart1_host_rxne = false;
static uint32_t t_prev = 0;

#ifndef DISCOBOT_TARGET
static uint32_t SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return 0u; }
static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000u; }
#endif

static void write_motor_pair(int pin_a, int pin_b, int direc) {
    if (direc == FORWARD) {
        GPIOA_output[pin_a] = 1u;
        GPIOA_output[pin_b] = 0u;
    } else if (direc == BACKWARD) {
        GPIOA_output[pin_a] = 0u;
        GPIOA_output[pin_b] = 1u;
    } else {
        GPIOA_output[pin_a] = 0u;
        GPIOA_output[pin_b] = 0u;
    }
}

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

void init_system(void) {
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

#ifndef DISCOBOT_TARGET
void SystemInit(void) { SystemCoreClock = 168000000u; }
#endif

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000u) != 0u) {
        for (;;) { }
    }
    SysTick_reload = SystemCoreClock / 1000u;
}

void init_button(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    g.GPIO_Pin = GPIO_Pin_0;
    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &g);
#endif
    GPIOA_IDR &= ~1u;
    b_i = 0u;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if ((uint32_t)(msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }
    if (usart1_available() > 0u) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    g.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &g);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);
#endif
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    write_motor_pair(1, 2, direc);
#ifdef DISCOBOT_TARGET
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, GPIOA_output[1] ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_2, GPIOA_output[2] ? Bit_SET : Bit_RESET);
#endif
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    write_motor_pair(3, 4, direc);
#ifdef DISCOBOT_TARGET
    GPIO_WriteBit(GPIOA, GPIO_Pin_3, GPIOA_output[3] ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, GPIOA_output[4] ? Bit_SET : Bit_RESET);
#endif
}

void move_forward(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(STOP, 0.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(STOP, 0.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void stop(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(STOP, 0.0f); }

int read_buttonc(int i) {
    if (i < 0 || i > 3) return -1;
#ifdef DISCOBOT_TARGET
    if (i == 0) GPIOA_IDR = (GPIOA_IDR & ~1u) | (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) ? 1u : 0u);
#endif
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250u) buttstate = ButtonIsPressed;
    } else {
        b_i = 0u;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed: break;
            case ButtonIsReleased: break;
            default: break;
        }
        laststate = (ButtonState)buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;
    if (roll != NULL) *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    if (pitch != NULL) *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI;
}

void discobot_set_button_level(bool high) { if (high) GPIOA_IDR |= 1u; else GPIOA_IDR &= ~1u; }

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef g;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    g.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    g.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &g);
    GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
#endif
    GPIOD_output[12] = 0u; GPIOD_output[13] = 0u; GPIOD_output[14] = 0u; GPIOD_output[15] = 0u;
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1u;
#ifdef DISCOBOT_TARGET
        GPIO_SetBits(GPIOD, (uint16_t)(1u << (12 + i)));
#endif
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0u;
#ifdef DISCOBOT_TARGET
        GPIO_ResetBits(GPIOD, (uint16_t)(1u << (12 + i)));
#endif
    }
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#endif
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b != NULL) {
#ifdef DISCOBOT_TARGET
        TM_LIS302DL_LIS3DSH_t axes;
        TM_LIS302DL_LIS3DSH_ReadAxes(&axes);
        b[0] = (float)axes.X / 1000.0f;
        b[1] = (float)axes.Y / 1000.0f;
        b[2] = (float)axes.Z / 1000.0f;
#else
        b[0] = (float)accel_raw_mg[0] / 1000.0f;
        b[1] = (float)accel_raw_mg[1] / 1000.0f;
        b[2] = (float)accel_raw_mg[2] / 1000.0f;
#endif
    }
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg; accel_raw_mg[1] = y_mg; accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    ADC_CommonInitTypeDef common;
    ADC_InitTypeDef adc;
    ADC_DeInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    common.ADC_Mode = ADC_Mode_Independent;
    common.ADC_Prescaler = ADC_Prescaler_Div8;
    common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&common);
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
}

float read_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    uint16_t raw = ADC_GetConversionValue(ADC1);
#else
    uint16_t raw = adc_raw_temperature;
#endif
    float temp = raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) { adc_raw_temperature = raw; }

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
    if (!rng_ready) {
        abort();
    }
    return (uint32_t)rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) { rng_ready = ready; rng_value = value; }

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) return;
    if (arr->enabled) { printf("CircArray already enabled"); return; }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) { arr->size = 0u; arr->enabled = false; arr->n_r = 0u; arr->n_w = 0u; return; }
    arr->size = (uint32_t)size; arr->enabled = true; arr->n_r = 0u; arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) return 0;
    if (!buf_full(arr)) { arr->buf[arr->n_w % arr->size] = c; arr->n_w++; return 1; }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) return 0;
    if (!buf_empty(arr)) { char c = arr->buf[arr->n_r % arr->size]; arr->n_r++; return c; }
    return 0;
}

bool buf_empty(CircArray *arr) { if (arr == NULL) return true; return arr->n_r == arr->n_w; }
bool buf_full(CircArray *arr) { if (arr == NULL || arr->size == 0u) return false; return !buf_empty(arr) && (arr->n_r % arr->size) == (arr->n_w % arr->size); }
int buf_available(CircArray *arr) { if (arr == NULL) return 0; return (int)(arr->n_w - arr->n_r); }

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) return false;
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) return false;
    arr->buf = (char *)tmp; arr->size = (uint32_t)newSize; return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    buf_clear(arr); free(arr->buf); arr->buf = NULL; arr->size = 0u; arr->n_r = 0u; arr->n_w = 0u; arr->enabled = false; return true;
}

void buf_clear(CircArray *arr) { if (arr != NULL && arr->buf != NULL && arr->size > 0u) memset(arr->buf, 0, arr->size); }

void init_usart1(int baud) {
    if (baud == 0) baud = 9600;
    usart1_baud_used = (uint32_t)baud;
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef us;
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
    us.USART_BaudRate = (uint32_t)baud;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &us);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
#endif
    if (msg.enabled) (void)buf_delete(&msg);
    initCircArray(&msg, 200);
#ifdef DISCOBOT_TARGET
    USART_Cmd(USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = (char)USART1->DR;
        (void)buf_putbyte(&msg, c);
    }
#else
    if (usart1_host_rxne) { char c = (char)usart1_host_dr; (void)buf_putbyte(&msg, c); usart1_host_rxne = false; }
#endif
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s) {
#ifdef DISCOBOT_TARGET
        while (!(USART1->SR & 0x40u)) { }
        USART_SendData(USART1, (uint16_t)(*s));
#endif
        char ch = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) usart1_tx_log[usart1_tx_length++] = ch;
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)(signed char)(uint8_t)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    usart1_host_dr = value; usart1_host_rxne = rxne;
    if (rxne) USART1_IRQHandler();
}

int func2(int R0) { uint32_t v = (uint32_t)R0; v += 1u; return (int)v; }
int func1(int R0) { return func2(R0); }