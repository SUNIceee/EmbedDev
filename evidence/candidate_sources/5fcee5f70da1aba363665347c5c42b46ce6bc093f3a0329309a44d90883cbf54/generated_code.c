#include "generated_code.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265358979323846

volatile uint32_t msTicks = 0;
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 0;
int sys_tick_config_failed = 0;

uint8_t GPIOA_output[5] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;

volatile uint32_t b_i = 0;
volatile int buttstate = ButtonIsReleased;
int laststate = ButtonIsReleased;

CircArray msg = {0};

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

timed_task timed_tasks[MAXNUMTASKS] = {0};

volatile char usart1_tx_log[USART1_TX_LOG_SIZE] = {0};
volatile uint32_t usart1_tx_length = 0;
volatile uint8_t usart1_rxne = 0;
volatile uint8_t usart1_dr = 0;
volatile int usart1_enabled = 0;
int usart1_baud = UART_BAUD;

int16_t accel_raw_x = 0;
int16_t accel_raw_y = 0;
int16_t accel_raw_z = 0;
uint16_t temp_raw = 0;
volatile int adc_eoc = 1;
volatile int rng_ready = 0;
uint32_t rng_value = 0;

static uint32_t t_prev = 0;

/* System and SysTick */
void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000;
}

int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000) != 0) {
        sys_tick_config_failed = 1;
        for (;;) { }
    }
    sys_tick_config_failed = 0;
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

/* LED */
void init_LED_pins(void) {
    for (int i = 12; i <= 15; i++) {
        GPIOD_output[i] = 0;
    }
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

/* Button */
void init_button(void) {
    GPIOA_IDR &= ~1u;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void checkbutton(void) {
    if (laststate == buttstate) {
        return;
    }

    switch (buttstate) {
        case ButtonIsPressed:
            laststate = ButtonIsPressed;
            break;
        case ButtonIsReleased:
            laststate = ButtonIsReleased;
            break;
        default:
            break;
    }
}

void discobot_set_button_level(int level) {
    if (level) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

/* Motor control */
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
            GPIOA_output[1] = 0;
            GPIOA_output[2] = 0;
            break;
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
            GPIOA_output[3] = 0;
            GPIOA_output[4] = 0;
            break;
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

/* Accelerometer */
int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float *acc[3]) {
    if (acc == NULL) {
        return;
    }

    if (acc[0] != NULL) {
        *acc[0] = accel_raw_x / 1000.0f;
    }
    if (acc[1] != NULL) {
        *acc[1] = accel_raw_y / 1000.0f;
    }
    if (acc[2] != NULL) {
        *acc[2] = accel_raw_z / 1000.0f;
    }
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
    accel_raw_x = x;
    accel_raw_y = y;
    accel_raw_z = z;
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    float roll_val = atan2f(acc_y, acc_z) * 180.0f / (float)PI;
    float pitch_val = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / (float)PI;

    if (pitch != NULL) {
        *pitch = pitch_val;
    }
    if (roll != NULL) {
        *roll = roll_val;
    }
}

/* Internal temperature sensor */
void init_temperature_sensor(void) {
    temp_raw = 0;
    adc_eoc = 1;
}

void discobot_set_adc_raw(uint16_t raw) {
    temp_raw = raw;
    adc_eoc = 1;
}

float read_temperature_sensor(void) {
    /* ADC SoftwareStartConv: model conversion completed */
    adc_eoc = 1;

    while (adc_eoc == 0) {
        /* wait for EOC */
    }

    float temp = (float)temp_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    return temp;
}

/* RNG */
void init_rng(void) {
    rng_ready = 0;
    rng_value = 0;
}

void discobot_set_rng(int ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

uint32_t get_random_number(void) {
    while (rng_ready == 0) {
        /* wait for DRDY */
    }

    return rng_value;
}

/* Circular array */
void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }

    if (size <= 0) {
        arr->buf = NULL;
        arr->size = 0;
        arr->enabled = true;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }

    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == NULL) {
        arr->size = 0;
        arr->enabled = true;
        arr->n_r = 0;
        arr->n_w = 0;
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
    if (arr == NULL) {
        return true;
    }
    if (buf_empty(arr)) {
        return false;
    }
    if (arr->size == 0) {
        return true;
    }
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
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

int buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL) {
        return false;
    }

    if (newSize <= 0) {
        free(arr->buf);
        arr->buf = NULL;
        arr->size = 0;
        arr->n_r = 0;
        arr->n_w = 0;
        arr->enabled = false;
        return true;
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

/* USART1 */
void init_usart1(int baud) {
    if (baud == 0) {
        fprintf(stderr, "init_usart1: baud=0, defaulting to 9600\n");
        baud = UART_BAUD;
    }

    usart1_baud = baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_enabled = 1;
}

void USART1_IRQHandler(void) {
    if (usart1_rxne) {
        uint8_t c = usart1_dr;
        usart1_rxne = 0;

        (void)buf_putbyte(&msg, (char)c);
    }
}

void discobot_usart1_inject_rx(uint8_t c) {
    usart1_dr = c;
    usart1_rxne = 1;
    USART1_IRQHandler();
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }

    while (*s) {
        char ch = *s++;

        uint32_t len = usart1_tx_length;
        if (len < (USART1_TX_LOG_SIZE - 1)) {
            usart1_tx_log[len++] = ch;
            usart1_tx_log[len] = '\0';
            usart1_tx_length = len;
        }
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    if (buf_empty(&msg)) {
        return -1;
    }
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

/* Timed task scheduler */
void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) {
        return;
    }

    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        timed_task *t = &timed_tasks[i];

        if (t->task == NULL) {
            continue;
        }

        if (msTicks >= (t->last_called + t->msinterval)) {
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

/* Main loop host equivalents */
void dispatch_uart_command(void) {
    int c = (int)usart1_readc();

    if (c >= 0 && c < MAXIDSIZE) {
        if (flookup[c] != NULL) {
            flookup[c]();
        }
    }
}

void main_loop_iteration(void) {
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }

    dispatch_uart_command();
}

/* Initialization sequence */
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

    usart1_send("UART1 Initialized. @9600bps\r\n");
}

/* ARM assembly equivalent host implementations */
int func2(int R0) {
    uint32_t result = (uint32_t)R0 + 1u;
    return (int)result;
}

int func1(int R0) {
    return func2(R0);
}
