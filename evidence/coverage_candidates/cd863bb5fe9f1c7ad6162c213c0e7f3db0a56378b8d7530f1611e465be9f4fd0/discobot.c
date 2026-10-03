/* discobot.c - Host-verifiable implementation of DiscoBot Test-Compatible v3. */
/*
 * DiscoBot Test-Compatible v3 host-verifiable C11 implementation.
 * This file implements the API declared in 6_generated_code.h.
 * Host mode models STM32F407 observable state and device injection.
 */

#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Private helpers and simulated hardware state
 * ---------------------------------------------------------------------- */

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000U;
}

static int SysTick_Config(uint32_t ticks) {
    if (ticks == 0U || ticks > 0x00FFFFFFU) {
        return 1;
    }
    SysTick_reload = ticks;
    return 0;
}

static int16_t discobot_accel_x_mg = 0;
static int16_t discobot_accel_y_mg = 0;
static int16_t discobot_accel_z_mg = 0;
static uint16_t discobot_adc_raw = 0;
static bool discobot_rng_ready = false;
static uint32_t discobot_rng_value = 0;
static uint8_t discobot_usart_dr = 0;
static bool discobot_usart_rxne = false;
static CircArray msg;
static ButtonState checkbutton_laststate = ButtonIsReleased;

/* -------------------------------------------------------------------------
 * Exported global state required by the public API
 * ---------------------------------------------------------------------- */

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = NULL;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = UART_BAUD;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* -------------------------------------------------------------------------
 * Motor control
 * ---------------------------------------------------------------------- */

void init_GPIO_A1A2A3A4_output(void) {
    int i;
    for (i = 1; i <= 4; ++i) {
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

/* -------------------------------------------------------------------------
 * Button handling
 * ---------------------------------------------------------------------- */

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (unsigned int)i) & 1U);
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
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
        checkbutton_laststate = buttstate;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1U;
    } else {
        GPIOA_IDR &= ~1U;
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1U;
    b_i = 0;
    buttstate = ButtonIsReleased;
    checkbutton_laststate = ButtonIsReleased;
}

/* -------------------------------------------------------------------------
 * LED control
 * ---------------------------------------------------------------------- */

void init_LED_pins(void) {
    int i;
    for (i = 12; i <= 15; ++i) {
        GPIOD_output[i] = 0;
    }
}

void LED_On(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 1;
}

void LED_Off(int i) {
    if (i < 0 || i > 3) {
        return;
    }
    GPIOD_output[12 + i] = 0;
}

/* -------------------------------------------------------------------------
 * Attitude calculation
 * ---------------------------------------------------------------------- */

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (pitch == NULL || roll == NULL) {
        return;
    }

    {
        const float pi = 3.14159265358979323846f;
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
    }
}

/* -------------------------------------------------------------------------
 * Accelerometer model
 * ---------------------------------------------------------------------- */

int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }

    b[0] = (float)discobot_accel_x_mg / 1000.0f;
    b[1] = (float)discobot_accel_y_mg / 1000.0f;
    b[2] = (float)discobot_accel_z_mg / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    discobot_accel_x_mg = x_mg;
    discobot_accel_y_mg = y_mg;
    discobot_accel_z_mg = z_mg;
}

/* -------------------------------------------------------------------------
 * Internal temperature model
 * ---------------------------------------------------------------------- */

void init_temperature_sensor(void) {
    /* Host model: no SPL registers to configure. */
}

float read_temperature_sensor(void) {
    float temp = (float)discobot_adc_raw;
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

/* -------------------------------------------------------------------------
 * RNG model
 * ---------------------------------------------------------------------- */

void init_rng(void) {
    /* Host model: no AHB2 RNG peripheral to configure. */
}

void discobot_set_rng(bool ready, uint32_t value) {
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

uint32_t get_random_number(void) {
    /*
     * On real hardware this function waits indefinitely for DRDY.
     * In host mode, a not-ready condition must not be reported as a
     * successful random read. Returning 0 here is a host-only indicator
     * that no valid sample was produced. Tests should call
     * discobot_set_rng(true, value) before expecting a valid random result.
     */
    if (!discobot_rng_ready) {
        return 0;
    }
    return discobot_rng_value;
}

/* -------------------------------------------------------------------------
 * Timed task scheduler
 * ---------------------------------------------------------------------- */

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    long ms;
    int i;

    if (myfunc == NULL) {
        return;
    }

    ms = (long)(interval_sec * 1000.0f);

    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)ms;
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *t = &timed_tasks[i];

        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void) {
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

/* -------------------------------------------------------------------------
 * Circular array
 * ---------------------------------------------------------------------- */

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
        arr->enabled = false;
        arr->n_r = 0;
        arr->n_w = 0;
        return;
    }

    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0 || !arr->enabled) {
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
    char c;

    if (arr == NULL || arr->buf == NULL || arr->size == 0 || !arr->enabled) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    c = arr->buf[arr->n_r % arr->size];
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
    if (arr == NULL || arr->size == 0 || arr->n_r == arr->n_w) {
        return false;
    }

    /* Unsigned subtraction remains correct across uint32_t wrap-around. */
    return (arr->n_w - arr->n_r) >= arr->size;
}

int buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }

    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    char *newbuf;

    if (arr == NULL || newSize <= 0) {
        return false;
    }

    newbuf = (char *)realloc(arr->buf, (size_t)newSize);
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

/* -------------------------------------------------------------------------
 * UART / USART1 model and command dispatch
 * ---------------------------------------------------------------------- */

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "init_usart1: baud is 0, using 9600\n");
    }

    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

    discobot_usart_rxne = false;
    discobot_usart_dr = 0;
}

void USART1_IRQHandler(void) {
    if (discobot_usart_rxne) {
        discobot_usart_rxne = false;
        (void)buf_putbyte(&msg, (char)discobot_usart_dr);
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }

    while (*s) {
        if (usart1_tx_length + 1 < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = (char)*s++;
        } else {
            break;
        }
    }

    if (usart1_tx_length < sizeof(usart1_tx_log)) {
        usart1_tx_log[usart1_tx_length] = '\0';
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
    discobot_usart_dr = value;
    discobot_usart_rxne = rxne;

    if (rxne) {
        USART1_IRQHandler();
    }
}

bool dispatch_uart_command(int command) {
    if (command < 0 || command >= MAXIDSIZE) {
        return false;
    }

    msgid = (_ID)command;
    callme = flookup[command];
    if (callme != NULL) {
        callme();
    }

    return true;
}

/* -------------------------------------------------------------------------
 * System initialization and main-loop equivalent
 * ---------------------------------------------------------------------- */

void SystemInit(void) {
    SystemCoreClock = 168000000U;
}

void init_systick(void) {
    SystemCoreClockUpdate();

    if (SysTick_Config(SystemCoreClock / 1000U) != 0) {
        for (;;) {
            /* SysTick configuration error path. */
        }
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void main_loop_iteration(void) {
    if (usart1_available() > 0U) {
        int command = (int)usart1_readc();
        (void)dispatch_uart_command(command);
    }

    checkbutton();
    execute_tasks();

    /* The required 1-second tick check intentionally has no observable action. */
}

/* -------------------------------------------------------------------------
 * ARM assembly-equivalent functions
 * ---------------------------------------------------------------------- */

int func2(int R0) {
    uint32_t v = (uint32_t)R0 + 1U;
    return (int)v;
}

int func1(int R0) {
    return func2(R0);
}
