#include "6_generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * Global objects required by API
 * ---------------------------------------------------------------------- */
volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
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
 * Internal host state
 * ---------------------------------------------------------------------- */
static CircArray msg;

static ButtonState last_button_state = ButtonIsReleased;

static int16_t discobot_accel_raw[3] = {0, 0, 0};
static uint16_t discobot_adc_raw = 0;

static bool discobot_rng_ready = false;
static uint32_t discobot_rng_value = 0;

static bool host_usart1_rxne = false;
static uint8_t host_usart1_rx_dr = 0;

/* -------------------------------------------------------------------------
 * System initialization
 * ---------------------------------------------------------------------- */
void SystemInit(void)
{
    SystemCoreClock = 168000000;
}

static void SystemCoreClockUpdate_(void)
{
    SystemCoreClock = 168000000;
}

static int SysTick_Config_(uint32_t ticks)
{
    SysTick_reload = ticks;
    return 0;
}

void init_systick(void)
{
    SystemCoreClockUpdate_();
    SysTick_reload = SystemCoreClock / 1000;
    if (SysTick_Config_(SysTick_reload) != 0) {
        /* Failure: target behaviour is a dead loop. */
        for (;;) { }
    }
}

void init_system(void)
{
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

/* -------------------------------------------------------------------------
 * Motor control
 * ---------------------------------------------------------------------- */
void init_GPIO_A1A2A3A4_output(void)
{
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed)
{
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

void set_right_motor_direc(int direc, float speed)
{
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

void move_forward(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_backward(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_forward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_forward_soft_right(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_backward_soft_right(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void)
{
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_spin_left(void)
{
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void stop(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

/* -------------------------------------------------------------------------
 * Button handling
 * ---------------------------------------------------------------------- */
int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_button(void)
{
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void SysTick_Handler(void)
{
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

void checkbutton(void)
{
    if (last_button_state != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
        last_button_state = buttstate;
    }
}

/* -------------------------------------------------------------------------
 * Command dispatch and main loop
 * ---------------------------------------------------------------------- */
bool dispatch_uart_command(int command)
{
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

void main_loop_iteration(void)
{
    static uint32_t t_prev = 0;

    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }

    if (usart1_available() > 0) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }

    checkbutton();
    execute_tasks();
}

/* -------------------------------------------------------------------------
 * LED control
 * ---------------------------------------------------------------------- */
void init_LED_pins(void)
{
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i)
{
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0;
    }
}

/* -------------------------------------------------------------------------
 * Accelerometer
 * ---------------------------------------------------------------------- */
int init_accelerometers(void)
{
    /* Host stub: success. */
    return 0;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    discobot_accel_raw[0] = x_mg;
    discobot_accel_raw[1] = y_mg;
    discobot_accel_raw[2] = z_mg;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }
    b[0] = discobot_accel_raw[0] / 1000.0f;
    b[1] = discobot_accel_raw[1] / 1000.0f;
    b[2] = discobot_accel_raw[2] / 1000.0f;
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z,
                     float *pitch, float *roll)
{
    const float rad_to_deg = 180.0f / 3.14159265358979323846f;

    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * rad_to_deg;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x,
                        sqrtf(acc_y * acc_y + acc_z * acc_z)) * rad_to_deg;
    }
}

/* -------------------------------------------------------------------------
 * Temperature sensor
 * ---------------------------------------------------------------------- */
void init_temperature_sensor(void)
{
    /* Host stub: no actual ADC setup required. */
}

void discobot_set_adc_raw(uint16_t raw)
{
    discobot_adc_raw = raw;
}

float read_temperature_sensor(void)
{
    float temp = (float)discobot_adc_raw;

    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    return temp;
}

/* -------------------------------------------------------------------------
 * RNG
 * ---------------------------------------------------------------------- */
void init_rng(void)
{
    discobot_rng_ready = false;
    discobot_rng_value = 0;
}

void discobot_set_rng(bool ready, uint32_t value)
{
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

uint32_t get_random_number(void)
{
    while (!discobot_rng_ready) {
        /* Block, matching historical no-timeout behaviour. */
    }
    return discobot_rng_value;
}

/* -------------------------------------------------------------------------
 * Timed task scheduler
 * ---------------------------------------------------------------------- */
void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (t->task != NULL && msTicks >= t->last_called + t->msinterval) {
            t->task();
            t->last_called = msTicks;
            t->numcalls++;
        }
    }
}

void printtimes(void)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, timed_tasks[i].numcalls);
        }
    }
}

/* -------------------------------------------------------------------------
 * Circular array
 * ---------------------------------------------------------------------- */
void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL) {
        return;
    }
    if (arr->enabled) {
        fprintf(stderr, "initCircArray: array already enabled\n");
        return;
    }

    if (size < 1) {
        size = 1;
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

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->size == 0) {
        return 0;
    }
    if (buf_full(arr)) {
        return 0;
    }

    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->size == 0) {
        return 0;
    }
    if (buf_empty(arr)) {
        return 0;
    }

    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr)
{
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || arr->size == 0) {
        return false;
    }
    if (buf_empty(arr)) {
        return false;
    }
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr)
{
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize)
{
    if (arr == NULL) {
        return false;
    }
    if (newSize < 1) {
        newSize = 1;
    }

    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) {
        return false;
    }

    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr)
{
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

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return;
    }
    memset(arr->buf, 0, arr->size);
    /* n_r, n_w, size, enabled remain unchanged. */
}

/* -------------------------------------------------------------------------
 * USART1
 * ---------------------------------------------------------------------- */
void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "init_usart1: baud=0 defaulting to 9600\n");
    }

    usart1_baud_used = (uint32_t)baud;

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

    host_usart1_rxne = false;
    host_usart1_rx_dr = 0;
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    host_usart1_rx_dr = value;
    host_usart1_rxne = rxne;

    if (rxne) {
        USART1_IRQHandler();
    }
}

void USART1_IRQHandler(void)
{
    if (host_usart1_rxne) {
        uint8_t c = host_usart1_rx_dr;
        host_usart1_rxne = false;
        /* If buffer is full, buf_putbyte returns 0: data is lost. */
        buf_putbyte(&msg, (char)c);
    }
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s) {
        char ch = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = ch;
        } else {
            break;
        }
    }

    if (usart1_tx_length < sizeof(usart1_tx_log)) {
        usart1_tx_log[usart1_tx_length] = '\0';
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

/* -------------------------------------------------------------------------
 * ARM assembly equivalents in C
 * ---------------------------------------------------------------------- */
int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}
