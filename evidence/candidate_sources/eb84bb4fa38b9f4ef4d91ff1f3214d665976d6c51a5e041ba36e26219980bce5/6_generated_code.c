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
#include "tm_stm32f4_lis302dl_lis3dsh.h"
#endif

#ifndef DISCOBOT_TARGET
typedef enum { RESET = 0, SET = 1 } FlagStatus;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;
typedef FlagStatus ITStatus;

#define RCC_AHB1Periph_GPIOA  ((uint32_t)0x00000001u)
#define RCC_AHB1Periph_GPIOB  ((uint32_t)0x00000002u)
#define RCC_AHB1Periph_GPIOD  ((uint32_t)0x00000008u)
#define RCC_AHB2Periph_RNG    ((uint32_t)0x00000040u)
#define RCC_APB2Periph_ADC1   ((uint32_t)0x00000100u)
#define RCC_APB2Periph_USART1 ((uint32_t)0x00000010u)

#define GPIO_Pin_0  ((uint32_t)0x00000001u)
#define GPIO_Pin_1  ((uint32_t)0x00000002u)
#define GPIO_Pin_2  ((uint32_t)0x00000004u)
#define GPIO_Pin_3  ((uint32_t)0x00000008u)
#define GPIO_Pin_4  ((uint32_t)0x00000010u)
#define GPIO_Pin_5  ((uint32_t)0x00000020u)
#define GPIO_Pin_6  ((uint32_t)0x00000040u)
#define GPIO_Pin_7  ((uint32_t)0x00000080u)
#define GPIO_Pin_8  ((uint32_t)0x00000100u)
#define GPIO_Pin_9  ((uint32_t)0x00000200u)
#define GPIO_Pin_10 ((uint32_t)0x00000400u)
#define GPIO_Pin_11 ((uint32_t)0x00000800u)
#define GPIO_Pin_12 ((uint32_t)0x00001000u)
#define GPIO_Pin_13 ((uint32_t)0x00002000u)
#define GPIO_Pin_14 ((uint32_t)0x00004000u)
#define GPIO_Pin_15 ((uint32_t)0x00008000u)
#define GPIO_PinSource6 ((uint8_t)6u)
#define GPIO_PinSource7 ((uint8_t)7u)
#define GPIO_AF_USART1 ((uint8_t)7u)

#define GPIO_Mode_IN  0u
#define GPIO_Mode_OUT 1u
#define GPIO_Mode_AF  2u
#define GPIO_OType_PP 0u
#define GPIO_PuPd_NOPULL 0u
#define GPIO_PuPd_UP 1u
#define GPIO_Speed_25MHz  2u
#define GPIO_Speed_100MHz 3u

#define USART_WordLength_8b 0u
#define USART_StopBits_1 0u
#define USART_Parity_No 0u
#define USART_Mode_Rx 1u
#define USART_Mode_Tx 2u
#define USART_HardwareFlowControl_None 0u
#define USART_IT_RXNE 0x01u
#define USART1_IRQn 37u

#define ADC_Mode_Independent 0u
#define ADC_Prescaler_Div8 6u
#define ADC_Resolution_12b 0u
#define ADC_DataAlign_Right 0u
#define ADC_ScanConvMode_Disable 0u
#define ADC_ContinuousConvMode_Enable 1u
#define ADC_ExternalTrigConvEdge_None 0u
#define ADC_ExternalTrigConv_None 0u
#define ADC_Channel_TempSensor 16u
#define ADC_SampleTime_144Cycles 6u
#define ADC_FLAG_EOC 0x02u

#define RNG_FLAG_DRDY 0x01u
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 1

typedef struct {
    uint32_t GPIO_Pin;
    uint32_t GPIO_Mode;
    uint32_t GPIO_OType;
    uint32_t GPIO_PuPd;
    uint32_t GPIO_Speed;
} GPIO_InitTypeDef;

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRRL;
    volatile uint32_t BSRRH;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
} USART_TypeDef;

typedef struct {
    uint32_t USART_BaudRate;
    uint32_t USART_WordLength;
    uint32_t USART_StopBits;
    uint32_t USART_Parity;
    uint32_t USART_Mode;
    uint32_t USART_HardwareFlowControl;
} USART_InitTypeDef;

typedef struct {
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t DR;
} ADC_TypeDef;

typedef struct {
    uint32_t ADC_Mode;
    uint32_t ADC_Prescaler;
    uint32_t ADC_Resolution;
    uint32_t ADC_ScanConvMode;
    uint32_t ADC_ContinuousConvMode;
    uint32_t ADC_ExternalTrigConvEdge;
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_DataAlign;
    uint32_t ADC_NbrOfConversion;
} ADC_InitTypeDef;

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} TM_LIS302DL_LIS3DSH_t;

static GPIO_TypeDef host_gpioa = {0};
static GPIO_TypeDef host_gpiob = {0};
static GPIO_TypeDef host_gpiod = {0};
static USART_TypeDef host_usart1 = {0x40u, 0u};
static ADC_TypeDef host_adc1 = {0};
static volatile bool host_usart1_rxne = false;

#define GPIOA (&host_gpioa)
#define GPIOB (&host_gpiob)
#define GPIOD (&host_gpiod)
#define USART1 (&host_usart1)
#define ADC1 (&host_adc1)
#endif

volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
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
uint32_t GPIOA_IDR = 0u;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 168000u;
uint32_t usart1_baud_used = 9600u;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg;
static int16_t discobot_accel_x_mg = 0;
static int16_t discobot_accel_y_mg = 0;
static int16_t discobot_accel_z_mg = 0;
static uint16_t discobot_adc_raw = 0u;
static volatile bool discobot_rng_ready = false;
static uint32_t discobot_rng_value = 0u;
static ButtonState laststate = ButtonIsReleased;
static uint32_t main_loop_t_prev = 0u;
static volatile int discobot_usart1_rxne_inject = 0;

#ifndef DISCOBOT_TARGET
static uint32_t SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0u;
}

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}

static void RCC_AHB1PeriphClockCmd(uint32_t periph, FunctionalState state) {
    (void)periph;
    (void)state;
}

static void RCC_AHB2PeriphClockCmd(uint32_t periph, FunctionalState state) {
    (void)periph;
    (void)state;
}

static void RCC_APB2PeriphClockCmd(uint32_t periph, FunctionalState state) {
    (void)periph;
    (void)state;
}

static void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct) {
    (void)GPIOx;
    (void)GPIO_InitStruct;
}

static void GPIO_PinAFConfig(GPIO_TypeDef *GPIOx, uint8_t GPIO_PinSource, uint8_t GPIO_AF) {
    (void)GPIOx;
    (void)GPIO_PinSource;
    (void)GPIO_AF;
}

static void USART_Init(USART_TypeDef *USARTx, USART_InitTypeDef *USART_InitStruct) {
    (void)USARTx;
    (void)USART_InitStruct;
}

static void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState) {
    (void)USARTx;
    (void)NewState;
}

static void USART_ITConfig(USART_TypeDef *USARTx, uint32_t USART_IT, FunctionalState NewState) {
    (void)USARTx;
    (void)USART_IT;
    (void)NewState;
}

static void USART_SendData(USART_TypeDef *USARTx, uint16_t Data) {
    (void)USARTx;
    (void)Data;
}

static ITStatus USART_GetITStatus(USART_TypeDef *USARTx, uint32_t USART_IT) {
    (void)USARTx;
    (void)USART_IT;
    return host_usart1_rxne ? SET : RESET;
}

static void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct) {
    (void)NVIC_InitStruct;
}

static void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct) {
    (void)ADCx;
    (void)ADC_InitStruct;
}

static void ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime) {
    (void)ADCx;
    (void)ADC_Channel;
    (void)Rank;
    (void)ADC_SampleTime;
}

static void ADC_TempSensorVrefintCmd(FunctionalState NewState) {
    (void)NewState;
}

static void ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState NewState) {
    (void)ADCx;
    (void)NewState;
}

static void ADC_SoftwareStartConv(ADC_TypeDef *ADCx) {
    (void)ADCx;
}

static FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint32_t ADC_FLAG) {
    (void)ADCx;
    (void)ADC_FLAG;
    return SET;
}

static uint16_t ADC_GetConversionValue(ADC_TypeDef *ADCx) {
    (void)ADCx;
    return discobot_adc_raw;
}

static void RNG_Cmd(FunctionalState NewState) {
    (void)NewState;
}

static FlagStatus RNG_GetFlagStatus(uint32_t RNG_FLAG) {
    (void)RNG_FLAG;
    return discobot_rng_ready ? SET : RESET;
}

static uint32_t RNG_GetRandomNumber(void) {
    return discobot_rng_value;
}

static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter) {
    (void)sensitivity;
    (void)filter;
    return 0;
}

static TM_LIS302DL_LIS3DSH_t TM_LIS302DL_LIS3DSH_ReadAxes(void) {
    TM_LIS302DL_LIS3DSH_t raw;
    raw.X = discobot_accel_x_mg;
    raw.Y = discobot_accel_y_mg;
    raw.Z = discobot_accel_z_mg;
    return raw;
}
#endif

static void gpioa_motor_pin_pair(uint32_t pin_a, uint8_t val_a, uint32_t pin_b, uint8_t val_b) {
    if (val_a == 0u) {
        GPIOA->BSRRH = pin_a;
    }
    if (val_b == 0u) {
        GPIOA->BSRRH = pin_b;
    }
    if (val_a == 1u) {
        GPIOA->BSRRL = pin_a;
    }
    if (val_b == 1u) {
        GPIOA->BSRRL = pin_b;
    }
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
    init_usart1(9600);
    usart1_send((volatile char *)discobot_startup_banner);
}

void SystemInit(void) {
    SystemCoreClockUpdate();
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    SystemCoreClock = 168000000u;
    SysTick_reload = SystemCoreClock / 1000u;
    msTicks = 0u;
    if (SysTick_Config(SysTick_reload) != 0u) {
        while (1) {
        }
    }
}

void init_button(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
#ifndef DISCOBOT_TARGET
    GPIOA_IDR &= ~1u;
#endif
    b_i = 0u;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
        msgid = (_ID)command;
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if ((msTicks - main_loop_t_prev) > 1000u) {
        main_loop_t_prev = msTicks;
    }
    if (usart1_available() > 0u) {
        char c = usart1_readc();
        (void)dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOA, &gpio);
    gpioa_motor_pin_pair(GPIO_Pin_1, 0u, GPIO_Pin_2, 0u);
    gpioa_motor_pin_pair(GPIO_Pin_3, 0u, GPIO_Pin_4, 0u);
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    uint8_t p1 = 0u;
    uint8_t p2 = 0u;
    if (direc == FORWARD) {
        p1 = 1u;
        p2 = 0u;
    } else if (direc == BACKWARD) {
        p1 = 0u;
        p2 = 1u;
    } else {
        p1 = 0u;
        p2 = 0u;
    }
    GPIOA_output[1] = p1;
    GPIOA_output[2] = p2;
    gpioa_motor_pin_pair(GPIO_Pin_1, p1, GPIO_Pin_2, p2);
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    uint8_t p3 = 0u;
    uint8_t p4 = 0u;
    if (direc == FORWARD) {
        p3 = 1u;
        p4 = 0u;
    } else if (direc == BACKWARD) {
        p3 = 0u;
        p4 = 1u;
    } else {
        p3 = 0u;
        p4 = 0u;
    }
    GPIOA_output[3] = p3;
    GPIOA_output[4] = p4;
    gpioa_motor_pin_pair(GPIO_Pin_3, p3, GPIO_Pin_4, p4);
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
    const float pi = 3.14159265358979323846f;
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_LED_pins(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOD, &gpio);
    GPIOD->BSRRH = (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIOD_output[12] = 0u;
    GPIOD_output[13] = 0u;
    GPIOD_output[14] = 0u;
    GPIOD_output[15] = 0u;
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        GPIOD->BSRRL = (1u << (12 + i));
        GPIOD_output[12 + i] = 1u;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD->BSRRH = (1u << (12 + i));
        GPIOD_output[12 + i] = 0u;
    }
}

int init_accelerometers(void) {
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }
    TM_LIS302DL_LIS3DSH_t raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = (float)raw.X / 1000.0f;
    b[1] = (float)raw.Y / 1000.0f;
    b[2] = (float)raw.Z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    discobot_accel_x_mg = x_mg;
    discobot_accel_y_mg = y_mg;
    discobot_accel_z_mg = z_mg;
}

void init_temperature_sensor(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_InitTypeDef adc;
    memset(&adc, 0, sizeof(adc));
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_Prescaler = ADC_Prescaler_Div8;
    adc.ADC_Resolution = ADC_Resolution_12b;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_ScanConvMode = ADC_ScanConvMode_Disable;
    adc.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_NbrOfConversion = 1u;
    ADC_Init(ADC1, &adc);
    ADC_RegularChannelConfig(ADC1, (uint8_t)ADC_Channel_TempSensor, 1u, (uint8_t)ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
}

float read_temperature_sensor(void) {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
    }
    uint16_t raw = ADC_GetConversionValue(ADC1);
    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    discobot_adc_raw = raw;
}

void init_rng(void) {
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    }
    return RNG_GetRandomNumber();
#else
    uint32_t timeout_guard = 0u;
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
        if (++timeout_guard >= 1000000u) {
            break;
        }
    }
    return RNG_GetRandomNumber();
#endif
}

void discobot_set_rng(bool ready, uint32_t value) {
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL &&
            msTicks >= (timed_tasks[i].last_called + timed_tasks[i].msinterval)) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
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
    if (arr == NULL) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }
    int safe_size = (size > 0) ? size : 0;
    arr->buf = (char *)calloc((size_t)safe_size, 1);
    if (arr->buf == NULL && safe_size > 0) {
        arr->size = 0u;
        arr->enabled = false;
        arr->n_r = 0u;
        arr->n_w = 0u;
        return;
    }
    arr->size = (uint32_t)safe_size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0u || arr->buf == NULL) {
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
    if (arr == NULL || arr->size == 0u || arr->buf == NULL || buf_empty(arr)) {
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
    if (arr == NULL || arr->size == 0u || arr->buf == NULL) {
        return true;
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
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) {
        return false;
    }
    arr->buf = newbuf;
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
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        fprintf(stderr, "init_usart1: baud=0 defaulting to 9600\n");
        baud = 9600;
    }
    usart1_baud_used = (uint32_t)baud;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    USART_InitTypeDef usart;
    memset(&usart, 0, sizeof(usart));
    usart.USART_BaudRate = (uint32_t)baud;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef nvic;
    memset(&nvic, 0, sizeof(nvic));
    nvic.NVIC_IRQChannel = (uint8_t)USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0u;
    nvic.NVIC_IRQChannelSubPriority = 0u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    initCircArray(&msg, 200);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (discobot_usart1_rxne_inject != 0 || USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        discobot_usart1_rxne_inject = 0;
#else
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
#endif
        uint8_t c = (uint8_t)(USART1->DR & 0xFFu);
        (void)buf_putbyte(&msg, (char)c);
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while ((USART1->SR & 0x40u) == 0u) {
        }
        USART_SendData(USART1, (uint16_t)(uint8_t)*s);
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
    uint8_t b = usart1_read();
    signed char sc = (signed char)b;
    return (char)sc;
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
#ifdef DISCOBOT_TARGET
    USART1->DR = (uint32_t)value;
    if (rxne) {
        discobot_usart1_rxne_inject = 1;
        USART1_IRQHandler();
    } else {
        discobot_usart1_rxne_inject = 0;
    }
#else
    host_usart1_rxne = rxne;
    USART1->DR = (uint32_t)value;
    if (rxne) {
        USART1_IRQHandler();
    }
    host_usart1_rxne = false;
#endif
}

#ifndef DISCOBOT_TARGET
int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}
#endif