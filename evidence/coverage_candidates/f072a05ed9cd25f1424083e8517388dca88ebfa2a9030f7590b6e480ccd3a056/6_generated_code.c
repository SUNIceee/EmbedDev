#include "6_generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Global state definitions
 * ---------------------------------------------------------------------- */
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
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* -------------------------------------------------------------------------
 * Host-state for device models
 * ---------------------------------------------------------------------- */
static CircArray msg;
static volatile bool g_usart_rxne = false;
static uint8_t g_usart_dr = 0;

static int16_t g_accel_raw[3] = {0, 0, 0};
static uint16_t g_adc_raw = 0;
static bool g_rng_ready = false;
static uint32_t g_rng_value = 0;

static ButtonState s_last_button_state = ButtonIsReleased;

/* -------------------------------------------------------------------------
 * System / clock helpers
 * ---------------------------------------------------------------------- */
void SystemInit(void) {
    SystemCoreClock = 168000000;
}

static void SystemCoreClockUpdate(void) {
    /* SystemCoreClock is already set by SystemInit on host. */
}

static int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000) != 0) {
        while (1) {
            /* fatal SysTick init error */
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
    init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

/* -------------------------------------------------------------------------
 * Motor control
 * ---------------------------------------------------------------------- */
void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed; /* no PWM, speed unused */
    if (direc == FORWARD) {
        GPIOA_output[1] = 1;
        GPIOA_output[2] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 1;
    } else {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[3] = 1;
        GPIOA_output[4] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 1;
    } else {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
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

/* -------------------------------------------------------------------------
 * Command dispatch and main loop
 * ---------------------------------------------------------------------- */
bool dispatch_uart_command(int command) {
    if (command >= (int)FWD && command <= (int)STOPCAR) {
        callme = flookup[command];
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    static uint32_t t_prev = 0;

    if (usart1_available() > 0) {
        int command = (int)usart1_readc();
        dispatch_uart_command(command);
    }

    checkbutton();
    execute_tasks();

    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
        /* 1 second idle action: intentionally none */
    }
}

/* -------------------------------------------------------------------------
 * Button and SysTick
 * ---------------------------------------------------------------------- */
int read_buttonc(int i) {
    if (i > 3 || i < 0) {
        return -1;
    }
    return (GPIOA_IDR >> i) & 1u;
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
    if (s_last_button_state != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
        }
        s_last_button_state = buttstate;
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1u;      /* clear PA0 */
    b_i = 0;
    buttstate = ButtonIsReleased;
    s_last_button_state = ButtonIsReleased;
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

/* -------------------------------------------------------------------------
 * LED control
 * ---------------------------------------------------------------------- */
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

/* -------------------------------------------------------------------------
 * Accelerometer model
 * ---------------------------------------------------------------------- */
int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    b[0] = (float)g_accel_raw[0] / 1000.0f;
    b[1] = (float)g_accel_raw[1] / 1000.0f;
    b[2] = (float)g_accel_raw[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    g_accel_raw[0] = x_mg;
    g_accel_raw[1] = y_mg;
    g_accel_raw[2] = z_mg;
}

/* -------------------------------------------------------------------------
 * Temperature sensor model
 * ---------------------------------------------------------------------- */
void init_temperature_sensor(void) {
    /* Host model: no-op */
}

float read_temperature_sensor(void) {
    float temp = (float)g_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    g_adc_raw = raw;
}

/* -------------------------------------------------------------------------
 * RNG model
 * ---------------------------------------------------------------------- */
void init_rng(void) {
    /* Host model: no-op */
}

uint32_t get_random_number(void) {
    while (!g_rng_ready) {
        /* hardware would block until DRDY is set */
    }
    return g_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    g_rng_ready = ready;
    g_rng_value = value;
}

/* -------------------------------------------------------------------------
 * Attitude calculation
 * ---------------------------------------------------------------------- */
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const double pi = 3.14159265358979323846;

    double roll_d = atan2((double)acc_y, (double)acc_z) * 180.0 / pi;
    double pitch_d = atan2(-(double)acc_x,
                           sqrt((double)acc_y * (double)acc_y +
                                (double)acc_z * (double)acc_z)) * 180.0 / pi;

    *roll = (float)roll_d;
    *pitch = (float)pitch_d;
}

/* -------------------------------------------------------------------------
 * Timed task scheduler
 * ---------------------------------------------------------------------- */
void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
    /* 25 slots full: silently fail */
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

/* -------------------------------------------------------------------------
 * Circular buffer (CircArray)
 * ---------------------------------------------------------------------- */
void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: already enabled\n");
        return;
    }

    arr->buf = (char *)calloc((size > 0 ? size : 1), sizeof(char));
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
    return (!buf_empty(arr)) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
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
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL && newSize != 0) {
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

/* -------------------------------------------------------------------------
 * USART1 host model
 * ---------------------------------------------------------------------- */
void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
        fprintf(stderr, "init_usart1: baud is 0, using 9600\n");
    }
    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
    if (g_usart_rxne) {
        char c = (char)g_usart_dr;
        g_usart_rxne = false;
        buf_putbyte(&msg, c);
    }
}

void usart1_send(volatile char *s) {
    while (*s) {
        char c = (char)*s;
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = c;
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
    g_usart_dr = value;
    g_usart_rxne = rxne;
    if (rxne) {
        USART1_IRQHandler();
    }
}

/* -------------------------------------------------------------------------
 * ARM assembly equivalents (host C implementation)
 * ---------------------------------------------------------------------- */
int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}
