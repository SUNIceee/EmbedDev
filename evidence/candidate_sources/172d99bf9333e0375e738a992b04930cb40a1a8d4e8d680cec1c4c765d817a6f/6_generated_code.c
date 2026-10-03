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
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
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

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = SYSTICK_PERIOD_TICKS;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};
CircArray msg = {0};
ButtonState laststate = ButtonIsReleased;
static int systick_config_result = 0;

static int16_t host_accel_raw_x = 0;
static int16_t host_accel_raw_y = 0;
static int16_t host_accel_raw_z = 0;
static uint16_t host_adc_raw = 0;
static bool host_adc_eoc = false;
static bool host_rng_ready = false;
static uint32_t host_rng_value = 0;
static volatile bool host_usart1_rxne = false;
static volatile uint8_t host_usart1_dr = 0;

void SystemInit(void) {
#ifdef DISCOBOT_TARGET
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0u) {}

    RCC->PLLCFGR = 0u;
    RCC->PLLCFGR = (8u << RCC_PLLCFGR_PLLM_Pos) |
                   (336u << RCC_PLLCFGR_PLLN_Pos) |
                   (0u << RCC_PLLCFGR_PLLP_Pos) |
                   (7u << RCC_PLLCFGR_PLLQ_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0u) {}

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~((uint32_t)RCC_CFGR_SW);
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
    SystemCoreClock = 168000000u;
#else
    SystemCoreClock = 168000000u;
#endif
}

void init_systick(void) {
#ifdef DISCOBOT_TARGET
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000u;
    systick_config_result = SysTick_Config(SysTick_reload);
    if (systick_config_result != 0) {
        while (1) {}
    }
#else
    if (SysTick_reload == 0u) {
        systick_config_result = 1;
        SysTick_reload = 0u;
        return;
    }
    SysTick_reload = SystemCoreClock / 1000u;
    systick_config_result = 0;
#endif
}

void init_LED_pins(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &gpio);
#endif
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void init_button(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &gpio);
#endif
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
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
#endif
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void init_system(void) {
    discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
    SystemInit();
    init_systick();
#ifndef DISCOBOT_TARGET
    if (systick_config_result != 0) {
        return;
    }
#endif
    init_LED_pins();
    init_button();
    init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[1] = 1;
            GPIOA_output[2] = 0;
#ifdef DISCOBOT_TARGET
            GPIO_SetBits(GPIOA, GPIO_Pin_1);
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#endif
            break;
        case BACKWARD:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 1;
#ifdef DISCOBOT_TARGET
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            GPIO_SetBits(GPIOA, GPIO_Pin_2);
#endif
            break;
        case STOP:
        default:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 0;
#ifdef DISCOBOT_TARGET
            GPIO_ResetBits(GPIOA, GPIO_Pin_1);
            GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#endif
            break;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[3] = 1;
            GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
            GPIO_SetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#endif
            break;
        case BACKWARD:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 1;
#ifdef DISCOBOT_TARGET
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
#endif
            break;
        case STOP:
        default:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
            GPIO_ResetBits(GPIOA, GPIO_Pin_3);
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
#endif
            break;
    }
}

void move_forward(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
#endif
}

void move_backward(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
#endif
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
#endif
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
#endif
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
#endif
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
#endif
}

void move_spin_right(void) {
    GPIOA_output[1] = 1;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 1;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
#endif
}

void move_spin_left(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 1;
    GPIOA_output[3] = 1;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
#endif
}

void stop(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
#ifdef DISCOBOT_TARGET
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
#endif
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
#ifdef DISCOBOT_TARGET
    return (int)((GPIOA->IDR >> (uint32_t)i) & 1u);
#else
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
#endif
}

void SysTick_Handler(void) {
    ++msTicks;
    int btn = read_buttonc(0);
    if (btn == 1) {
        ++b_i;
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
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / (float)M_PI;
    }
    if (pitch != NULL) {
        float denom = sqrtf((acc_y * acc_y) + (acc_z * acc_z));
        *pitch = atan2f(-acc_x, denom) * 180.0f / (float)M_PI;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void LED_On(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 1;
#ifdef DISCOBOT_TARGET
    GPIOD->BSRRL = (1u << (12u + (uint32_t)i));
#endif
}

void LED_Off(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 0;
#ifdef DISCOBOT_TARGET
    GPIOD->BSRRH = (1u << (12u + (uint32_t)i));
#endif
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
#else
    return 0;
#endif
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_Axes_t raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
#else
    b[0] = host_accel_raw_x / 1000.0f;
    b[1] = host_accel_raw_y / 1000.0f;
    b[2] = host_accel_raw_z / 1000.0f;
#endif
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    host_accel_raw_x = x_mg;
    host_accel_raw_y = y_mg;
    host_accel_raw_z = z_mg;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_CommonInitTypeDef adc_common;
    adc_common.ADC_Mode = ADC_Mode_Independent;
    adc_common.ADC_Prescaler = ADC_Prescaler_Div8;
    adc_common.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&adc_common);

    ADC_InitTypeDef adc;
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
#else
    host_adc_eoc = true;
#endif
}

float read_temperature_sensor(void) {
    uint16_t raw = 0;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    raw = (uint16_t)ADC_GetConversionValue(ADC1);
#else
    raw = host_adc_raw;
#endif

    float temp = (float)raw;
    temp = temp / 4095.0f;
    temp = temp * 3.3f;
    temp = temp - 0.760f;
    temp = temp / 0.0025f;
    temp = temp + 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    host_adc_raw = raw;
    host_adc_eoc = true;
}

void init_rng(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
    host_rng_ready = false;
    host_rng_value = 0;
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {}
    return RNG_GetRandomNumber();
#else
    while (!host_rng_ready) {
        /* Controlled host blocking condition; injected ready flag must be set externally. */
    }
    return host_rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) {
    host_rng_ready = ready;
    host_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) {
        return;
    }
    for (int i = 0; i < MAXNUMTASKS; ++i) {
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
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\r\n");
        return;
    }
    char *p = (char *)calloc((size_t)size, 1);
    if (p == NULL) {
        arr->buf = NULL;
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }
    arr->buf = p;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return 0;
    }
    if (buf_empty(arr)) {
        return 0;
    }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) {
        return false;
    }
    char *tmp = (char *)realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) {
        return false;
    }
    arr->buf = tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
        fprintf(stderr, "init_usart1: baud==0 defaulting to 9600\r\n");
    }
    usart1_baud_used = (uint32_t)baud;

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

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
    usart.USART_BaudRate = (uint32_t)baud;
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
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t c = (uint8_t)USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, (char)c);
    }
#else
    if (host_usart1_rxne) {
        (void)buf_putbyte(&msg, (char)host_usart1_dr);
        host_usart1_rxne = false;
    }
#endif
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((USART1->SR & 0x40u) == 0u) {}
        USART_SendData(USART1, (uint16_t)*s);
#else
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
        }
#endif
        ++s;
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
#ifndef DISCOBOT_TARGET
    host_usart1_dr = value;
    host_usart1_rxne = rxne;
    if (rxne) {
        USART1_IRQHandler();
    }
#else
    (void)value;
    (void)rxne;
#endif
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    static uint32_t t_prev = 0;
    if ((msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }

    if (usart1_available() > 0u) {
        int command = (int)usart1_readc();
        (void)dispatch_uart_command(command);
    }

    checkbutton();
    execute_tasks();
}

#ifndef DISCOBOT_TARGET
int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}
#endif