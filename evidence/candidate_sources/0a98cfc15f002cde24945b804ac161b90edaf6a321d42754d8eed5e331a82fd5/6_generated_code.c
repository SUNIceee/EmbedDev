#include "6_generated_code.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
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
uint32_t GPIOA_IDR = 0x00000000u;
uint32_t SystemCoreClock = 0;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg;
static int16_t discobot_accel_raw_x = 0;
static int16_t discobot_accel_raw_y = 0;
static int16_t discobot_accel_raw_z = 0;
static uint16_t discobot_adc_raw_value = 0;
static bool discobot_adc_eoc = true;
static bool discobot_rng_ready = false;
static uint32_t discobot_rng_value = 0;
static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0;
static volatile uint8_t usart1_rx_dr = 0;
static volatile bool usart1_rxne_flag = false;
static volatile uint32_t usart1_sr = 0x40u;

#define RESET ((uint8_t)0)
#define SET ((uint8_t)1)
#define DISABLE ((uint8_t)0)
#define ENABLE ((uint8_t)1)
#define RCC_AHB2Periph_RNG 0x00000001u
#define RNG_FLAG_DRDY 0x00000001u
#define ADC_FLAG_EOC 0x00000001u
#define USART_IT_RXNE 0x00000001u
#define TM_LIS3DSH_Sensitivity_2G 0
#define TM_LIS3DSH_Filter_50Hz 0

typedef void *ADC_TypeDef;
typedef void *RNG_TypeDef;
typedef void *USART_TypeDef;
static ADC_TypeDef ADC1 = (ADC_TypeDef)0;
static RNG_TypeDef RNG = (RNG_TypeDef)0;
static USART_TypeDef USART1 = (USART_TypeDef)0;

typedef struct LIS3DSH_Axes {
    int16_t X;
    int16_t Y;
    int16_t Z;
} LIS3DSH_Axes;

static void SystemCoreClockUpdate(void) { }
static int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0;
}
static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter) {
    (void)sensitivity;
    (void)filter;
    return 0;
}
static LIS3DSH_Axes TM_LIS302DL_LIS3DSH_ReadAxes(void) {
    LIS3DSH_Axes axes;
    axes.X = discobot_accel_raw_x;
    axes.Y = discobot_accel_raw_y;
    axes.Z = discobot_accel_raw_z;
    return axes;
}
static void ADC_SoftwareStartConv(ADC_TypeDef adc) {
    (void)adc;
    discobot_adc_eoc = true;
}
static uint8_t ADC_GetFlagStatus(ADC_TypeDef adc, uint16_t flag) {
    (void)adc;
    (void)flag;
    return discobot_adc_eoc ? SET : RESET;
}
static uint16_t ADC_GetConversionValue(ADC_TypeDef adc) {
    (void)adc;
    return discobot_adc_raw_value;
}
static void ADC_TempSensorVrefintCmd(uint8_t cmd) {
    (void)cmd;
}
static void ADC_Cmd(ADC_TypeDef adc, uint8_t cmd) {
    (void)adc;
    (void)cmd;
}
static void RCC_AHB2PeriphClockCmd(uint32_t periph, uint8_t cmd) {
    (void)periph;
    (void)cmd;
}
static void RNG_Cmd(uint8_t cmd) {
    (void)cmd;
}
static uint8_t RNG_GetFlagStatus(RNG_TypeDef rng, uint16_t flag) {
    (void)rng;
    (void)flag;
    return discobot_rng_ready ? SET : RESET;
}
static uint32_t RNG_GetRandomNumber(RNG_TypeDef rng) {
    (void)rng;
    return discobot_rng_value;
}
static uint8_t USART_GetITStatus(USART_TypeDef usart, uint16_t it) {
    (void)usart;
    (void)it;
    return usart1_rxne_flag ? SET : RESET;
}
static char USART_ReceiveData(USART_TypeDef usart) {
    (void)usart;
    return (char)usart1_rx_dr;
}
static void USART_SendData(USART_TypeDef usart, char data) {
    (void)usart;
    (void)data;
}
static void USART_Cmd(USART_TypeDef usart, uint8_t cmd) {
    (void)usart;
    (void)cmd;
}

void SystemInit(void) {
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000u;
    if (SysTick_Config(SystemCoreClock / 1000u) != 0) {
        while (1) { }
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
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
    discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
    usart1_send("UART1 Initialized. @9600bps\r\n");
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[1] = 1;
            GPIOA_output[2] = 0;
            break;
        case BACKWARD:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 1;
            break;
        case STOP:
        default:
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 0;
            break;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    switch (direc) {
        case FORWARD:
            GPIOA_output[3] = 1;
            GPIOA_output[4] = 0;
            break;
        case BACKWARD:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 1;
            break;
        case STOP:
        default:
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 0;
            break;
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
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250u) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (laststate == buttstate) {
        return;
    }
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

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (pitch == NULL || roll == NULL) {
        return;
    }
    float denom = sqrtf((acc_y * acc_y) + (acc_z * acc_z));
    *roll = atan2f(acc_y, acc_z) * (180.0f / 3.14159265358979323846f);
    *pitch = atan2f(-acc_x, denom) * (180.0f / 3.14159265358979323846f);
}

void discobot_set_button_level(bool high) {
    GPIOA_IDR = (GPIOA_IDR & ~1u) | (high ? 1u : 0u);
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 0;
    }
}

int init_accelerometers(void) {
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }
    LIS3DSH_Axes raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    b[0] = raw.X / 1000.0f;
    b[1] = raw.Y / 1000.0f;
    b[2] = raw.Z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    discobot_accel_raw_x = x_mg;
    discobot_accel_raw_y = y_mg;
    discobot_accel_raw_z = z_mg;
}

void init_temperature_sensor(void) {
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    discobot_adc_eoc = true;
}

float read_temperature_sensor(void) {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) { }
    uint16_t raw = ADC_GetConversionValue(ADC1);
    float temp = (float)raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    discobot_adc_raw_value = raw;
}

void init_rng(void) {
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);
}

uint32_t get_random_number(void) {
    volatile uint32_t guard = 0;
    while (RNG_GetFlagStatus(RNG, RNG_FLAG_DRDY) == RESET) {
        if (++guard > 1000000u) {
            break;
        }
    }
    return RNG_GetRandomNumber(RNG);
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
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
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
    if (arr == NULL || size <= 0) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf != NULL) {
        arr->size = (uint32_t)size;
        arr->enabled = true;
        arr->n_r = 0;
        arr->n_w = 0;
    } else {
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
    }
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0 || !arr->enabled) {
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
    if (arr == NULL || arr->size == 0 || !arr->enabled) {
        return 0;
    }
    if (!buf_empty(arr)) {
        char c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
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
    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (new_buf == NULL) {
        return false;
    }
    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }
    if (arr->buf != NULL) {
        buf_clear(arr);
        free(arr->buf);
    }
    arr->buf = NULL;
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    int effective_baud = baud;
    if (baud == 0) {
        effective_baud = (int)UART_BAUD;
        fprintf(stderr, "init_usart1: baud==0, using 9600");
    }
    usart1_baud_used = (uint32_t)effective_baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART_Cmd(USART1, ENABLE);
    usart1_sr = 0x40u;
}

void USART1_IRQHandler(void) {
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        char c = USART_ReceiveData(USART1);
        (void)buf_putbyte(&msg, c);
        usart1_rxne_flag = false;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while (!(usart1_sr & 0x40u)) { }
        char ch = (char)*s;
        USART_SendData(USART1, ch);
        if (usart1_tx_length < (sizeof(usart1_tx_log) - 1u)) {
            usart1_tx_log[usart1_tx_length] = ch;
            usart1_tx_length++;
            usart1_tx_log[usart1_tx_length] = 0;
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
    if (rxne) {
        usart1_rx_dr = value;
        usart1_rxne_flag = true;
        USART1_IRQHandler();
    }
}

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}