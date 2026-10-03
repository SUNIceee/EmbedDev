#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

/* Simulated USART registers */
#define USART_SR_RXNE  0x20u
#define USART_SR_TXE   0x40u
static volatile uint32_t USART1_SR = 0;
static volatile uint8_t  USART1_DR = 0;

/* Simulated ADC EOC flag */
#define ADC_SR_EOC     0x02u
static volatile uint32_t ADC1_SR = 0;
static uint16_t adc_raw = 0;

/* Accelerometer raw values */
static int16_t accel_raw_x = 0;
static int16_t accel_raw_y = 0;
static int16_t accel_raw_z = 0;

/* RNG state */
static bool rng_ready = false;
static uint32_t rng_value = 0;

/* USART receive ring buffer */
static CircArray msg;

/* Check-button last state */
static ButtonState checkbutton_laststate = ButtonIsReleased;

/* Main loop previous ms tick */
static uint32_t main_t_prev = 0;

/* Global definitions */
volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = NULL;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000UL;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS];

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, move_forward_soft_right,
    move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop
};

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000UL;
}

static uint32_t SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0; /* success */
}

void SystemInit(void) {
    SystemCoreClock = 168000000UL;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000UL) != 0) {
        while (1) { }
    }
}

void init_GPIO_A1A2A3A4_output(void) {
    for (int i = 1; i <= 4; i++) {
        GPIOA_output[i] = 0;
    }
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

void init_LED_pins(void) {
    for (int i = 12; i <= 15; i++) {
        GPIOD_output[i] = 0;
    }
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

void init_button(void) {
    GPIOA_IDR = 0;
    b_i = 0;
    buttstate = ButtonIsReleased;
    checkbutton_laststate = ButtonIsReleased;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1UL;
    } else {
        GPIOA_IDR &= ~1UL;
    }
}

int read_buttonc(int i) {
    if (i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1UL);
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
    if (checkbutton_laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsReleased:
                break;
            case ButtonIsPressed:
                break;
        }
        checkbutton_laststate = buttstate;
    }
}

int init_accelerometers(void) {
    accel_raw_x = 0;
    accel_raw_y = 0;
    accel_raw_z = 0;
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }
    b[0] = (float)accel_raw_x / 1000.0f;
    b[1] = (float)accel_raw_y / 1000.0f;
    b[2] = (float)accel_raw_z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_x = x_mg;
    accel_raw_y = y_mg;
    accel_raw_z = z_mg;
}

void init_temperature_sensor(void) {
    ADC1_SR = 0;
    adc_raw = 0;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw = raw;
    ADC1_SR |= ADC_SR_EOC;
}

float read_temperature_sensor(void) {
    while (!(ADC1_SR & ADC_SR_EOC)) {
        /* wait for conversion complete */
    }
    float temp = (float)adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    ADC1_SR &= ~ADC_SR_EOC;
    return temp;
}

void init_rng(void) {
    rng_ready = false;
    rng_value = 0;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

uint32_t get_random_number(void) {
    while (!rng_ready) {
        /* wait for DRDY */
    }
    return rng_value;
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) {
        return;
    }
    if (arr->enabled) {
        printf("CircArray already initialized\r\n");
        return;
    }
    if (size <= 0) {
        arr->buf = NULL;
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = arr->n_w = 0;
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == NULL) {
        arr->size = 0;
        arr->enabled = false;
        arr->n_r = arr->n_w = 0;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0) {
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

void buf_clear(CircArray *arr) {
    if (arr == NULL) {
        return;
    }
    if (arr->buf != NULL) {
        memset(arr->buf, 0, arr->size);
    }
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || arr->buf == NULL || newSize <= 0) {
        return false;
    }
    char *p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p == NULL) {
        return false;
    }
    arr->buf = p;
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
    arr->n_r = arr->n_w = 0;
    return true;
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
    }
    usart1_baud_used = (uint32_t)baud;
    USART1_SR = USART_SR_TXE;
    USART1_DR = 0;
    usart1_tx_length = 0;
    usart1_tx_log[0] = '\0';
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }
    while (*s) {
        while (!(USART1_SR & USART_SR_TXE)) {
            /* wait for TXE/TC */
        }
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = (char)*s;
        }
        s++;
    }
    usart1_tx_log[usart1_tx_length] = '\0';
}

void USART1_IRQHandler(void) {
    if (USART1_SR & USART_SR_RXNE) {
        uint8_t c = USART1_DR;
        (void)buf_putbyte(&msg, (char)c);
        USART1_SR &= ~USART_SR_RXNE;
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        USART1_DR = value;
        USART1_SR |= USART_SR_RXNE;
        USART1_IRQHandler();
    } else {
        USART1_SR &= ~USART_SR_RXNE;
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    uint8_t raw = (uint8_t)buf_getbyte(&msg);
    if (raw == 0xFFU) {
        return -1;
    }
    return (char)raw;
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        int c = usart1_readc();
        if (c >= 0 && c <= 8) {
            dispatch_uart_command(c);
        }
    }
    checkbutton();
    execute_tasks();
    if (msTicks - main_t_prev > 1000) {
        main_t_prev = msTicks;
    }
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(int32_t)(interval_sec * 1000.0f);
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
            printf("t%d=%d", i, timed_tasks[i].numcalls);
        }
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (pitch == NULL || roll == NULL) {
        return;
    }
    double ax = (double)acc_x;
    double ay = (double)acc_y;
    double az = (double)acc_z;
    double roll_d = atan2(ay, az) * 180.0 / PI;
    double pitch_d = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    *roll = (float)roll_d;
    *pitch = (float)pitch_d;
}

int func2(int R0) {
    uint32_t r = (uint32_t)R0;
    r += 1U;
    return (int)r;
}

int func1(int R0) {
    return func2(R0);
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
