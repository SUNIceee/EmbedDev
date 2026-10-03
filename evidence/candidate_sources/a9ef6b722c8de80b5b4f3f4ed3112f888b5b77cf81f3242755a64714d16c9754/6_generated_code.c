#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = stop;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = SYSTICK_PERIOD_TICKS;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {0};
static ButtonState s_laststate = ButtonIsReleased;
static uint32_t s_t_prev = 0;
static int16_t s_accel_x_mg = 0;
static int16_t s_accel_y_mg = 0;
static int16_t s_accel_z_mg = 0;
static uint16_t s_adc_raw = 0;
static bool s_rng_ready = true;
static uint32_t s_rng_value = 0;
static uint8_t s_usart1_dr = 0;
static bool s_usart1_rxne = false;
static bool s_accel_initialized = false;
static bool s_temp_initialized = false;
static bool s_rng_initialized = false;
static volatile int s_systick_config_result = 0;
static volatile bool s_systick_config_failed = false;

static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000u; }
static int SysTick_Config(uint32_t ticks) {
    if (ticks == 0u) { s_systick_config_result = 1; return 1; }
    SysTick_reload = ticks;
    s_systick_config_result = 0;
    return 0;
}

static void set_motor_pins(uint8_t pa1, uint8_t pa2, uint8_t pa3, uint8_t pa4) {
    GPIOA_output[1] = pa1 ? 1u : 0u;
    GPIOA_output[2] = pa2 ? 1u : 0u;
    GPIOA_output[3] = pa3 ? 1u : 0u;
    GPIOA_output[4] = pa4 ? 1u : 0u;
}

motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, move_forward_soft_right,
    move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop
};

void SystemInit(void) { SystemCoreClock = 168000000u; }

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000u) != 0) {
        s_systick_config_failed = true;
        while (s_systick_config_failed) { }
    } else {
        s_systick_config_failed = false;
    }
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0; GPIOD_output[13] = 0; GPIOD_output[14] = 0; GPIOD_output[15] = 0;
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    s_laststate = ButtonIsReleased;
}

void init_system(void) {
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

void init_GPIO_A1A2A3A4_output(void) { set_motor_pins(0,0,0,0); }

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
    if ((uint32_t)(msTicks - s_t_prev) > 1000u) { s_t_prev = msTicks; }
    if (usart1_available() > 0u) { dispatch_uart_command((int)usart1_readc()); }
    checkbutton();
    execute_tasks();
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) { return -1; }
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= BUTTON_DEBOUNCE_MS) { buttstate = ButtonIsPressed; }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    if (s_laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed: break;
            case ButtonIsReleased: break;
            default: break;
        }
        s_laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (roll != NULL) { *roll = (float)(atan2((double)acc_y, (double)acc_z) * 180.0 / PI); }
    if (pitch != NULL) { *pitch = (float)(atan2((double)-acc_x, sqrt((double)(acc_y * acc_y + acc_z * acc_z))) * 180.0 / PI); }
}

void discobot_set_button_level(bool high) {
    if (high) { GPIOA_IDR |= 1u; } else { GPIOA_IDR &= ~1u; }
}

void LED_On(int i) { if (i >= 0 && i <= 3) { GPIOD_output[12 + i] = 1; } }
void LED_Off(int i) { if (i >= 0 && i <= 3) { GPIOD_output[12 + i] = 0; } }

int init_accelerometers(void) { s_accel_initialized = true; return 0; }

void read_accelerometers(float b[3]) {
    (void)s_accel_initialized;
    if (b == NULL) { return; }
    b[0] = (float)s_accel_x_mg / 1000.0f;
    b[1] = (float)s_accel_y_mg / 1000.0f;
    b[2] = (float)s_accel_z_mg / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    s_accel_x_mg = x_mg; s_accel_y_mg = y_mg; s_accel_z_mg = z_mg;
}

void init_temperature_sensor(void) { s_temp_initialized = true; }

float read_temperature_sensor(void) {
    (void)s_temp_initialized;
    float temp = (float)s_adc_raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) { s_adc_raw = raw; }

void init_rng(void) { s_rng_initialized = true; }

uint32_t get_random_number(void) {
    (void)s_rng_initialized;
#ifdef DISCOBOT_TARGET
    while (!s_rng_ready) { }
#else
    if (!s_rng_ready) { abort(); }
#endif
    return s_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) { s_rng_ready = ready; s_rng_value = value; }

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
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
        if (timed_tasks[i].task != NULL) { printf("t%d=%d", i, (int)timed_tasks[i].numcalls); }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) { return; }
    if (arr->enabled) { printf("CircArray already enabled"); return; }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) { arr->size = 0; arr->enabled = false; arr->n_r = 0; arr->n_w = 0; return; }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

bool buf_empty(CircArray *arr) {
    if (arr == NULL) { return true; }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0u) { return false; }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) { return 0; }
    if (buf_full(arr)) { return 0; }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) { return 0; }
    if (buf_empty(arr)) { return 0; }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

int buf_available(CircArray *arr) {
    if (arr == NULL) { return 0; }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) { return false; }
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) { return false; }
    arr->buf = (char *)tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL) { return; }
    memset(arr->buf, 0, arr->size);
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) { return false; }
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
    if (baud == 0) { baud = 9600; }
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) { (void)buf_delete(&msg); }
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
}

void USART1_IRQHandler(void) {
    if (s_usart1_rxne) {
        char c = (char)s_usart1_dr;
        (void)buf_putbyte(&msg, c);
        s_usart1_rxne = false;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) { return; }
    while (*s) {
        char c = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = c;
        }
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    s_usart1_dr = value;
    s_usart1_rxne = rxne;
    if (rxne) { USART1_IRQHandler(); }
}

int func2(int R0) { return (int)((uint32_t)R0 + 1u); }
int func1(int R0) { return func2(R0); }