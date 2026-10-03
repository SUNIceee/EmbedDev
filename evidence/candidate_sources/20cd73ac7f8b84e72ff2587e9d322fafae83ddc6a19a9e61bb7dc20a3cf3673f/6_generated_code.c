#include "6_generated_code.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define RESET 0
#define SET 1
#define ENABLE 1
#define DISABLE 0
#define RCC_AHB1Periph_GPIOA 1u
#define RCC_AHB1Periph_GPIOD 2u
#define RCC_AHB1Periph_GPIOB 3u
#define RCC_AHB2Periph_RNG 4u
#define RCC_APB2Periph_USART1 5u
#define ADC_FLAG_EOC 0x01u
#define RNG_FLAG_DRDY 0x02u
#define USART_IT_RXNE 1
#define USART1 ((void*)0)
#define ADC1 ((void*)0)
#define TM_LIS3DSH_Sensitivity_2G 2
#define TM_LIS3DSH_Filter_50Hz 50

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = (_ID)0;
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
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 0;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};
CircArray msg = {0};
int systick_config_failed = 0;

static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0;
static int16_t accel_raw_x = 0;
static int16_t accel_raw_y = 0;
static int16_t accel_raw_z = 0;
static uint16_t adc_raw_value = 0;
static bool adc_eoc = false;
static bool rng_ready = false;
static uint32_t rng_value = 0;
static volatile uint32_t usart1_sr = 0x40;
static volatile bool usart1_rxne = false;
static volatile uint8_t usart1_dr = 0;

static void SystemCoreClockUpdate(void) {}
static int SysTick_Config(uint32_t ticks) {
    (void)ticks;
    return 0;
}
static void RCC_AHB1PeriphClockCmd(uint32_t periph, int state) {
    (void)periph;
    (void)state;
}
static void RCC_AHB2PeriphClockCmd(uint32_t periph, int state) {
    (void)periph;
    (void)state;
}
static void RCC_APB2PeriphClockCmd(uint32_t periph, int state) {
    (void)periph;
    (void)state;
}
static void RNG_Cmd(int state) {
    (void)state;
}
static int RNG_GetFlagStatus(uint32_t flag) {
    (void)flag;
    return rng_ready ? SET : RESET;
}
static uint32_t RNG_GetRandomNumber(void) {
    return rng_value;
}
static void ADC_SoftwareStartConv(void *adc) {
    (void)adc;
    adc_eoc = true;
}
static int ADC_GetFlagStatus(void *adc, uint32_t flag) {
    (void)adc;
    (void)flag;
    return adc_eoc ? SET : RESET;
}
static uint16_t ADC_GetConversionValue(void *adc) {
    (void)adc;
    return adc_raw_value;
}
static int TM_LIS302DL_LIS3DSH_Init(int sensitivity, int filter) {
    (void)sensitivity;
    (void)filter;
    return 0;
}
typedef struct AccelAxes {
    int16_t X;
    int16_t Y;
    int16_t Z;
} AccelAxes;
static AccelAxes TM_LIS302DL_LIS3DSH_ReadAxes(void) {
    AccelAxes a;
    a.X = accel_raw_x;
    a.Y = accel_raw_y;
    a.Z = accel_raw_z;
    return a;
}
static void USART_SendData(void *usart, uint16_t data) {
    (void)usart;
    (void)data;
}
static void usart1_process_rxne(void) {
    if (!usart1_rxne) {
        return;
    }
    (void)buf_putbyte(&msg, (char)usart1_dr);
    usart1_rxne = false;
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000;
    msTicks = 0;
    if (SysTick_Config(SysTick_reload) != 0) {
        systick_config_failed = 1;
        while (1) {}
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
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
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
    if (usart1_available() > 0) {
        int c = (int)usart1_readc();
        if (c >= 0 && c < MAXIDSIZE) {
            dispatch_uart_command(c);
        }
    }
    checkbutton();
    execute_tasks();
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
    set_left_motor_direc(FORWARD, 0);
    set_right_motor_direc(FORWARD, 0);
}

void move_backward(void) {
    set_left_motor_direc(BACKWARD, 0);
    set_right_motor_direc(BACKWARD, 0);
}

void move_forward_soft_left(void) {
    set_left_motor_direc(STOP, 0);
    set_right_motor_direc(FORWARD, 0);
}

void move_forward_soft_right(void) {
    set_left_motor_direc(FORWARD, 0);
    set_right_motor_direc(STOP, 0);
}

void move_backward_soft_left(void) {
    set_left_motor_direc(STOP, 0);
    set_right_motor_direc(BACKWARD, 0);
}

void move_backward_soft_right(void) {
    set_left_motor_direc(BACKWARD, 0);
    set_right_motor_direc(STOP, 0);
}

void move_spin_right(void) {
    set_left_motor_direc(FORWARD, 0);
    set_right_motor_direc(BACKWARD, 0);
}

void move_spin_left(void) {
    set_left_motor_direc(BACKWARD, 0);
    set_right_motor_direc(FORWARD, 0);
}

void stop(void) {
    set_left_motor_direc(STOP, 0);
    set_right_motor_direc(STOP, 0);
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (unsigned int)i) & 1u);
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
        laststate = buttstate;
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float pi = 3.14159265358979323846f;
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
    }
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    }
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
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0;
    }
}

int init_accelerometers(void) {
    return TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_50Hz);
}

void read_accelerometers(float b[3]) {
    AccelAxes raw = TM_LIS302DL_LIS3DSH_ReadAxes();
    if (b != NULL) {
        b[0] = raw.X / 1000.0f;
        b[1] = raw.Y / 1000.0f;
        b[2] = raw.Z / 1000.0f;
    }
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_x = x_mg;
    accel_raw_y = y_mg;
    accel_raw_z = z_mg;
}

void init_temperature_sensor(void) {
    adc_raw_value = 0;
    adc_eoc = false;
}

float read_temperature_sensor(void) {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {}
    uint16_t raw = ADC_GetConversionValue(ADC1);
    adc_raw_value = raw;
    float temp = raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw_value = raw;
    adc_eoc = true;
}

void init_rng(void) {
    rng_ready = false;
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
    if (myfunc == NULL) {
        return;
    }
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
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf != NULL) {
        arr->size = (uint32_t)size;
        arr->enabled = true;
        arr->n_r = 0;
        arr->n_w = 0;
    }
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0 || arr->buf == NULL) {
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
    if (arr == NULL || arr->size == 0 || arr->buf == NULL) {
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
    return !buf_empty(arr) && (arr->n_r % arr->size) == (arr->n_w % arr->size);
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
    char *p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p != NULL) {
        arr->buf = p;
        arr->size = (uint32_t)newSize;
        return true;
    }
    return false;
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
    if (arr != NULL && arr->buf != NULL && arr->size > 0) {
        memset(arr->buf, 0, arr->size);
    }
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
    }
    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_sr = 0x40;
    usart1_rxne = false;
    usart1_dr = 0;
}

void USART1_IRQHandler(void) {
    if (usart1_rxne) {
        usart1_process_rxne();
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while (!(usart1_sr & 0x40)) {}
        uint8_t ch = (uint8_t)*s;
        USART_SendData(USART1, (uint16_t)ch);
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = (char)ch;
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
    usart1_dr = value;
    if (rxne) {
        usart1_rxne = true;
        usart1_process_rxne();
    } else {
        usart1_rxne = false;
    }
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}