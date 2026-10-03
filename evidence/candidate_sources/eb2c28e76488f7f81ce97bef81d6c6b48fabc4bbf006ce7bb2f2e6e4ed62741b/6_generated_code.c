#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DISCOBOT_TARGET
extern void SystemCoreClockUpdate(void);
extern uint32_t SysTick_Config(uint32_t ticks);
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = stop;
motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left, move_forward_soft_right,
    move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop
};
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {0};
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t discobot_adc_raw = 0;
static volatile bool discobot_rng_ready = true;
static uint32_t discobot_rng_value = 0;
static bool usart1_rxne = false;
static uint8_t usart1_dr = 0;
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_t_prev = 0;
static bool accelerometer_initialized = false;
static bool adc_initialized = false;
static bool rng_initialized = false;
static bool host_systick_config_failed = false;
static bool host_rng_blocked = false;

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000u; }
static uint32_t SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return host_systick_config_failed ? 1u : 0u; }
#endif

static void write_motor_pins(uint8_t pa1, uint8_t pa2, uint8_t pa3, uint8_t pa4) {
    GPIOA_output[1] = pa1 ? 1u : 0u;
    GPIOA_output[2] = pa2 ? 1u : 0u;
    GPIOA_output[3] = pa3 ? 1u : 0u;
    GPIOA_output[4] = pa4 ? 1u : 0u;
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

void SystemInit(void) {
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    uint32_t reload = SystemCoreClock / 1000u;
    SysTick_reload = reload;
    if (SysTick_Config(reload) != 0u) {
#ifdef DISCOBOT_TARGET
        while (1) { }
#else
        host_systick_config_failed = true;
        return;
#endif
    }
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        if (callme != NULL) { callme(); }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if ((uint32_t)(msTicks - main_t_prev) > 1000u) { main_t_prev = msTicks; }
    if (usart1_available() > 0u) { dispatch_uart_command((int)usart1_readc()); }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void) { write_motor_pins(0, 0, 0, 0); }

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { GPIOA_output[1] = 1u; GPIOA_output[2] = 0u; }
    else if (direc == BACKWARD) { GPIOA_output[1] = 0u; GPIOA_output[2] = 1u; }
    else { GPIOA_output[1] = 0u; GPIOA_output[2] = 0u; }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) { GPIOA_output[3] = 1u; GPIOA_output[4] = 0u; }
    else if (direc == BACKWARD) { GPIOA_output[3] = 0u; GPIOA_output[4] = 1u; }
    else { GPIOA_output[3] = 0u; GPIOA_output[4] = 0u; }
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
    if (last_button_state != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed: break;
            case ButtonIsReleased:
            default: break;
        }
        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;
    if (roll != NULL) { *roll = atan2f(acc_y, acc_z) * 180.0f / PI; }
    if (pitch != NULL) { *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI; }
}

void discobot_set_button_level(bool high) { if (high) { GPIOA_IDR |= 1u; } else { GPIOA_IDR &= ~1u; } }

void init_LED_pins(void) {
    GPIOD_output[12] = 0u; GPIOD_output[13] = 0u; GPIOD_output[14] = 0u; GPIOD_output[15] = 0u;
}

void LED_On(int i) { if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 1u; } }
void LED_Off(int i) { if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 0u; } }

int init_accelerometers(void) { accelerometer_initialized = true; return 0; }

void read_accelerometers(float b[3]) {
    (void)accelerometer_initialized;
    if (b == NULL) { return; }
    b[0] = (float)accel_raw_mg[0] / 1000.0f;
    b[1] = (float)accel_raw_mg[1] / 1000.0f;
    b[2] = (float)accel_raw_mg[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) { accel_raw_mg[0] = x_mg; accel_raw_mg[1] = y_mg; accel_raw_mg[2] = z_mg; }

void init_temperature_sensor(void) { adc_initialized = true; }

float read_temperature_sensor(void) {
    (void)adc_initialized;
    uint16_t raw = discobot_adc_raw;
    float temp = (float)raw;
    temp = temp / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) { discobot_adc_raw = raw; }

void init_rng(void) { rng_initialized = true; }

uint32_t get_random_number(void) {
    (void)rng_initialized;
#ifdef DISCOBOT_TARGET
    while (discobot_rng_ready == false) { }
#else
    host_rng_blocked = false;
    if (discobot_rng_ready == false) { host_rng_blocked = true; return discobot_rng_value; }
#endif
    return discobot_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) { discobot_rng_ready = ready; discobot_rng_value = value; }

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (uint32_t i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (uint32_t i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void) {
    for (uint32_t i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) { printf("t%d=%d", (int)i, (int)timed_tasks[i].numcalls); }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) { return; }
    if (arr->enabled) { printf("CircArray already enabled"); return; }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) { arr->size = 0u; arr->enabled = false; arr->n_r = 0u; arr->n_w = 0u; return; }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || buf_full(arr)) { return 0; }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || buf_empty(arr)) { return 0; }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) { if (arr == NULL) { return true; } return arr->n_r == arr->n_w; }

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0u) { return false; }
    return !buf_empty(arr) && (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) { if (arr == NULL) { return 0; } return (int)(arr->n_w - arr->n_r); }

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) { return false; }
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) { return false; }
    arr->buf = (char *)tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) { return false; }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;
    return true;
}

void buf_clear(CircArray *arr) { if (arr == NULL || arr->buf == NULL || arr->size == 0u) { return; } memset(arr->buf, 0, arr->size); }

void init_usart1(int baud) {
    if (baud == 0) { baud = UART_BAUD; printf("baud defaulted to 9600"); }
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) { (void)buf_delete(&msg); }
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_rxne = false;
}

void USART1_IRQHandler(void) {
    if (usart1_rxne) {
        char c = (char)usart1_dr;
        (void)buf_putbyte(&msg, c);
        usart1_rxne = false;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) { return; }
    while (*s) {
        char c = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) { usart1_tx_log[usart1_tx_length++] = c; }
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    usart1_dr = value;
    usart1_rxne = rxne;
    if (rxne) { USART1_IRQHandler(); }
}

int func1(int R0) { return func2(R0); }

int func2(int R0) {
    uint32_t v = (uint32_t)R0;
    v = v + 1u;
    return (int)v;
}