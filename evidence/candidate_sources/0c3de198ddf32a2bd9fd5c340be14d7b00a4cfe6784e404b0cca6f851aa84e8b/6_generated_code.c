#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
typedef struct { int16_t X; int16_t Y; int16_t Z; } TM_LIS302DL_LIS3DSH_t;
typedef struct { uint32_t SR; uint32_t DR; } DISCOBOT_USART_Regs;
typedef struct { uint32_t ADC_Mode; uint32_t ADC_Prescaler; uint32_t ADC_DMAAccessMode; uint32_t ADC_TwoSamplingDelay; } ADC_CommonInitTypeDef;
typedef struct { uint32_t ADC_Resolution; uint32_t ADC_ScanConvMode; uint32_t ADC_ContinuousConvMode; uint32_t ADC_ExternalTrigConvEdge; uint32_t ADC_ExternalTrigConv; uint32_t ADC_DataAlign; uint32_t ADC_NbrOfConversion; } ADC_InitTypeDef;
typedef struct { uint32_t USART_BaudRate; uint32_t USART_WordLength; uint32_t USART_StopBits; uint32_t USART_Parity; uint32_t USART_HardwareFlowControl; uint32_t USART_Mode; } USART_InitTypeDef;
typedef struct { uint8_t NVIC_IRQChannel; uint8_t NVIC_IRQChannelPreemptionPriority; uint8_t NVIC_IRQChannelSubPriority; int NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
typedef struct { uint32_t GPIO_Pin; uint32_t GPIO_Mode; uint32_t GPIO_Speed; uint32_t GPIO_OType; uint32_t GPIO_PuPd; } GPIO_InitTypeDef;
extern int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter);
extern void TM_LIS302DL_LIS3DSH_ReadAxes(TM_LIS302DL_LIS3DSH_t *axes);
extern void RCC_AHB2PeriphClockCmd(uint32_t periph, int enable);
extern void RNG_Cmd(int enable);
extern int RNG_GetFlagStatus(uint32_t flag);
extern uint32_t RNG_GetRandomNumber(void);
extern void RCC_APB2PeriphClockCmd(uint32_t periph, int enable);
extern void RCC_AHB1PeriphClockCmd(uint32_t periph, int enable);
extern void RCC_APB1PeriphClockCmd(uint32_t periph, int enable);
extern void ADC_CommonInit(ADC_CommonInitTypeDef *init);
extern void ADC_Init(void *adc, ADC_InitTypeDef *init);
extern void ADC_RegularChannelConfig(void *adc, uint8_t channel, uint8_t rank, uint8_t sample_time);
extern void ADC_TempSensorVrefintCmd(int enable);
extern void ADC_Cmd(void *adc, int enable);
extern void ADC_SoftwareStartConv(void *adc);
extern int ADC_GetFlagStatus(void *adc, uint32_t flag);
extern uint16_t ADC_GetConversionValue(void *adc);
extern void GPIO_Init(void *gpio, GPIO_InitTypeDef *init);
extern void GPIO_PinAFConfig(void *gpio, uint16_t pin_source, uint8_t af);
extern void USART_Init(void *usart, USART_InitTypeDef *init);
extern void USART_ITConfig(void *usart, uint16_t it, int enable);
extern int USART_GetITStatus(void *usart, uint16_t it);
extern void USART_Cmd(void *usart, int enable);
extern void USART_SendData(void *usart, uint16_t data);
extern void NVIC_Init(NVIC_InitTypeDef *init);
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define RCC_AHB2Periph_RNG 0x40U
#define RCC_AHB1Periph_GPIOB 0x02U
#define RCC_APB2Periph_USART1 0x10U
#define RCC_APB2Periph_ADC1 0x100U
#define RNG_FLAG_DRDY 0x01U
#define ADC_FLAG_EOC 0x02U
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 0
#define ADC_Mode_Independent 0U
#define ADC_Prescaler_Div8 3U
#define ADC_DMAAccessMode_Disabled 0U
#define ADC_TwoSamplingDelay_5Cycles 0U
#define ADC_Resolution_12b 0U
#define ADC_ExternalTrigConvEdge_None 0U
#define ADC_ExternalTrigConv_T1_CC1 0U
#define ADC_DataAlign_Right 0U
#define ADC_Channel_TempSensor 16U
#define ADC_SampleTime_144Cycles 144U
#define GPIO_Pin_6 (1U << 6)
#define GPIO_Pin_7 (1U << 7)
#define GPIO_Mode_AF 2U
#define GPIO_Speed_100MHz 100U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_UP 1U
#define GPIO_PinSource6 6U
#define GPIO_PinSource7 7U
#define GPIO_AF_USART1 7U
#define USART_WordLength_8b 0U
#define USART_StopBits_1 0U
#define USART_Parity_No 0U
#define USART_HardwareFlowControl_None 0U
#define USART_Mode_Tx 0x08U
#define USART_Mode_Rx 0x04U
#define USART_IT_RXNE 0x0525U
#define USART1_IRQn 37U
#define ADC1 ((void *)0x40012000U)
#define GPIOB ((void *)0x40020400U)
#define USART1 ((DISCOBOT_USART_Regs *)0x40011000U)
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {0};
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t adc_raw = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;
static bool rng_initialized = false;
static bool rng_blocked_condition = false;
static bool adc_initialized = false;
static bool accelerometer_initialized = false;
static bool usart1_host_rxne = false;
static uint8_t usart1_host_dr = 0;
static ButtonState previous_button_state = ButtonIsReleased;
static uint32_t idle_t_prev = 0;

static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000U; }
static int SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return 0; }

void stop(void);
void move_forward(void);
void move_backward(void);
void move_forward_soft_left(void);
void move_forward_soft_right(void);
void move_backward_soft_left(void);
void move_backward_soft_right(void);
void move_spin_right(void);
void move_spin_left(void);

motor_command_fn callme = stop;
motor_command_fn flookup[9] = { move_forward, move_backward, move_forward_soft_left, move_forward_soft_right, move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop };

void SystemInit(void) { SystemCoreClock = 168000000U; }

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U) != 0) {
        for (;;) { }
    }
}

void init_LED_pins(void) {
    for (int i = 12; i <= 15; ++i) GPIOD_output[i] = 0;
}

void init_button(void) {
    GPIOA_IDR &= ~1U;
    b_i = 0;
    buttstate = ButtonIsReleased;
    previous_button_state = ButtonIsReleased;
}

int init_accelerometers(void) {
#ifdef DISCOBOT_TARGET
    int rc = TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
    accelerometer_initialized = (rc == 0);
    return rc;
#else
    accelerometer_initialized = true;
    return 0;
#endif
}

void init_rng(void) {
#ifdef DISCOBOT_TARGET
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
#endif
    rng_initialized = true;
    rng_blocked_condition = false;
}

void init_temperature_sensor(void) {
#ifdef DISCOBOT_TARGET
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
    adc_init.ADC_NbrOfConversion = 1U;
    ADC_Init(ADC1, &adc_init);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1U, ADC_SampleTime_144Cycles);
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
#endif
    adc_initialized = true;
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0; GPIOA_output[3] = 0; GPIOA_output[4] = 0;
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
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { GPIOA_output[1] = 1; GPIOA_output[2] = 0; }
    else if (direc == BACKWARD) { GPIOA_output[1] = 0; GPIOA_output[2] = 1; }
    else { GPIOA_output[1] = 0; GPIOA_output[2] = 0; }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { GPIOA_output[3] = 1; GPIOA_output[4] = 0; }
    else if (direc == BACKWARD) { GPIOA_output[3] = 0; GPIOA_output[4] = 1; }
    else { GPIOA_output[3] = 0; GPIOA_output[4] = 0; }
}

void move_forward(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(STOP, 0.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(STOP, 0.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void stop(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(STOP, 0.0f); }

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if ((uint32_t)(msTicks - idle_t_prev) > 1000U) idle_t_prev = msTicks;
    if (usart1_available() > 0U) dispatch_uart_command((int)usart1_readc());
    checkbutton();
    execute_tasks();
}

int read_buttonc(int i) {
    if (i > 3 || i < 0) return -1;
    return (int)((GPIOA_IDR >> (unsigned)i) & 1U);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) buttstate = ButtonIsPressed;
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (previous_button_state != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed: break;
            case ButtonIsReleased: break;
            default: break;
        }
        previous_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;
    if (roll != NULL) *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    if (pitch != NULL) *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI;
}

void discobot_set_button_level(bool high) {
    if (high) GPIOA_IDR |= 1U; else GPIOA_IDR &= ~1U;
}

void LED_On(int i) { if (i >= 0 && i < 4) GPIOD_output[12 + i] = 1; }
void LED_Off(int i) { if (i >= 0 && i < 4) GPIOD_output[12 + i] = 0; }

void read_accelerometers(float b[3]) {
    (void)accelerometer_initialized;
    if (b == NULL) return;
#ifdef DISCOBOT_TARGET
    TM_LIS302DL_LIS3DSH_t raw;
    TM_LIS302DL_LIS3DSH_ReadAxes(&raw);
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
    accel_raw_mg[0] = x_mg; accel_raw_mg[1] = y_mg; accel_raw_mg[2] = z_mg;
}

float read_temperature_sensor(void) {
    uint16_t raw;
    (void)adc_initialized;
#ifdef DISCOBOT_TARGET
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    raw = ADC_GetConversionValue(ADC1);
#else
    raw = adc_raw;
#endif
    float temp = raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) { adc_raw = raw; }

uint32_t get_random_number(void) {
    (void)rng_initialized;
#ifdef DISCOBOT_TARGET
    while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) { }
    return RNG_GetRandomNumber();
#else
    if (!rng_ready) {
        rng_blocked_condition = true;
        fprintf(stderr, "DiscoBot RNG not ready: controlled blocking condition\n");
        abort();
    }
    rng_blocked_condition = false;
    return rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) { rng_ready = ready; rng_value = value; rng_blocked_condition = !ready; }

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            break;
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
    arr->buf = (char *)calloc((size_t)size, 1U);
    if (arr->buf == NULL) return;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) { return arr == NULL || arr->n_r == arr->n_w; }

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0U) return false;
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) return 0;
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    char c;
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) return 0;
    if (!buf_empty(arr)) {
        c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
}

int buf_available(CircArray *arr) {
    if (arr == NULL) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) return false;
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) return false;
    arr->buf = (char *)tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL) return;
    memset(arr->buf, 0, arr->size);
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void init_usart1(int baud) {
    if (baud == 0) baud = 9600;
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) (void)buf_delete(&msg);
    initCircArray(&msg, 200);
#ifdef DISCOBOT_TARGET
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;
    NVIC_InitTypeDef nvic_init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    gpio_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio_init.GPIO_Mode = GPIO_Mode_AF;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio_init);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    usart_init.USART_BaudRate = (uint32_t)baud;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init((void *)USART1, &usart_init);
    USART_ITConfig((void *)USART1, USART_IT_RXNE, ENABLE);
    nvic_init.NVIC_IRQChannel = USART1_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic_init.NVIC_IRQChannelSubPriority = 0U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
    USART_Cmd((void *)USART1, ENABLE);
#endif
}

void USART1_IRQHandler(void) {
#ifdef DISCOBOT_TARGET
    if (USART_GetITStatus((void *)USART1, USART_IT_RXNE) != RESET) {
        char c = (char)(USART1->DR & 0xFFU);
        (void)buf_putbyte(&msg, c);
    }
#else
    if (usart1_host_rxne) {
        char c = (char)usart1_host_dr;
        (void)buf_putbyte(&msg, c);
        usart1_host_rxne = false;
    }
#endif
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s) {
        char c = *s++;
#ifdef DISCOBOT_TARGET
        while (!(USART1->SR & 0x40U)) { }
        USART_SendData((void *)USART1, (uint16_t)c);
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log)) usart1_tx_log[usart1_tx_length++] = c;
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }

char usart1_readc(void) {
    uint8_t byte = (uint8_t)buf_getbyte(&msg);
    return (char)((signed char)byte);
}

uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        usart1_host_dr = value;
        usart1_host_rxne = true;
        USART1_IRQHandler();
    }
}

int func2(int R0) {
    uint32_t v = (uint32_t)R0;
    v += 1U;
    return (int)v;
}

int func1(int R0) { return func2(R0); }