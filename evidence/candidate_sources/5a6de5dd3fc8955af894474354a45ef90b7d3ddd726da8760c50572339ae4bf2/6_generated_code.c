#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USART_SR_TXE 0x40u
#define USART_SR_RXNE 0x20u

volatile uint32_t msTicks;
volatile uint32_t b_i;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
uint8_t GPIOA_output[16];
uint8_t GPIOD_output[16];
uint32_t GPIOA_IDR;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload;
uint32_t usart1_baud_used;
char usart1_tx_log[1024];
size_t usart1_tx_length;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS];

static CircArray msg;
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_loop_prev_ms;
static int16_t accel_raw[3];
static uint16_t adc_raw_value;
static bool rng_ready;
static uint32_t rng_value;
static uint8_t usart1_dr;
static uint32_t usart1_sr;
static bool usart1_enabled;

static void motor_pair(int a, int b, int direction)
{
    if (direction == FORWARD) {
        GPIOA_output[a] = 1u; GPIOA_output[b] = 0u;
    } else if (direction == BACKWARD) {
        GPIOA_output[a] = 0u; GPIOA_output[b] = 1u;
    } else {
        GPIOA_output[a] = 0u; GPIOA_output[b] = 0u;
    }
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

motor_command_fn flookup[9] = { move_forward, move_backward, move_forward_soft_left, move_forward_soft_right, move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop };
motor_command_fn callme = stop;

void SystemInit(void) { SystemCoreClock = 168000000u; }
void init_systick(void) { SystemCoreClock = 168000000u; SysTick_reload = SystemCoreClock / 1000u; }
void init_button(void) { GPIOA_IDR &= ~1u; b_i = 0u; buttstate = ButtonIsReleased; last_button_state = ButtonIsReleased; }
void init_system(void)
{
    SystemInit(); init_systick(); init_LED_pins(); init_button();
    (void)init_accelerometers(); init_rng(); init_temperature_sensor();
    init_GPIO_A1A2A3A4_output(); init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void init_GPIO_A1A2A3A4_output(void) { GPIOA_output[1] = GPIOA_output[2] = GPIOA_output[3] = GPIOA_output[4] = 0u; }
void set_left_motor_direc(int direc, float speed) { (void)speed; motor_pair(1, 2, direc); }
void set_right_motor_direc(int direc, float speed) { (void)speed; motor_pair(3, 4, direc); }

int read_buttonc(int i) { if (i < 0 || i > 3) return -1; return (int)((GPIOA_IDR >> (unsigned)i) & 1u); }
void SysTick_Handler(void)
{
    ++msTicks;
    if (read_buttonc(0)) { ++b_i; if (b_i >= BUTTON_DEBOUNCE_MS) buttstate = ButtonIsPressed; }
    else { b_i = 0u; buttstate = ButtonIsReleased; }
}
void checkbutton(void)
{
    if (last_button_state != buttstate) {
        switch (buttstate) { case ButtonIsPressed: break; case ButtonIsReleased: break; default: break; }
        last_button_state = buttstate;
    }
}
void discobot_set_button_level(bool high) { if (high) GPIOA_IDR |= 1u; else GPIOA_IDR &= ~1u; }

bool dispatch_uart_command(int command)
{
    if (command < 0 || command >= MAXIDSIZE) return false;
    msgid = (_ID)command; callme = flookup[command];
    if (callme) callme();
    return true;
}
void main_loop_iteration(void)
{
    if (usart1_available() != 0u) dispatch_uart_command((int)usart1_readc());
    checkbutton(); execute_tasks();
    if ((uint32_t)(msTicks - main_loop_prev_ms) > 1000u) main_loop_prev_ms = msTicks;
}
void calc_pitch_roll(float x, float y, float z, float *pitch, float *roll)
{
    if (roll) *roll = atan2f(y, z) * 180.0f / 3.14159265358979323846f;
    if (pitch) *pitch = atan2f(-x, sqrtf(y * y + z * z)) * 180.0f / 3.14159265358979323846f;
}

void init_LED_pins(void) { GPIOD_output[12] = GPIOD_output[13] = GPIOD_output[14] = GPIOD_output[15] = 0u; }
void LED_On(int i) { if (i >= 0 && i < 4) GPIOD_output[12 + i] = 1u; }
void LED_Off(int i) { if (i >= 0 && i < 4) GPIOD_output[12 + i] = 0u; }

int init_accelerometers(void) { return 0; }
void read_accelerometers(float b[3]) { if (b) { b[0] = accel_raw[0] / 1000.0f; b[1] = accel_raw[1] / 1000.0f; b[2] = accel_raw[2] / 1000.0f; } }
void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) { accel_raw[0] = x; accel_raw[1] = y; accel_raw[2] = z; }
void init_temperature_sensor(void) { }
float read_temperature_sensor(void)
{
    float t = (float)adc_raw_value;
    t /= 4095.0f; t *= 3.3f; t -= 0.760f; t /= 0.0025f; t += 25.0f;
    return t;
}
void discobot_set_adc_raw(uint16_t raw) { adc_raw_value = raw & 0x0fffu; }
void init_rng(void) { }
void discobot_set_rng(bool ready, uint32_t value) { rng_ready = ready; rng_value = value; }
uint32_t get_random_number(void)
{
#ifdef DISCOBOT_TARGET
    while (!rng_ready) { }
#else
    if (!rng_ready) return 0u;
#endif
    return rng_value;
}

void add_timed_task(void (*task)(void), float interval_sec)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; ++i) if (!timed_tasks[i].task) {
        timed_tasks[i].task = task;
        timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
        timed_tasks[i].last_called = 0u; timed_tasks[i].numcalls = 0u; return;
    }
}
void execute_tasks(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; ++i) if (timed_tasks[i].task && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
        timed_tasks[i].task(); timed_tasks[i].last_called = msTicks; ++timed_tasks[i].numcalls;
    }
}
void printtimes(void) { int i; for (i = 0; i < MAXNUMTASKS; ++i) if (timed_tasks[i].task) printf("t%d=%d", i, (int)timed_tasks[i].numcalls); }

void initCircArray(CircArray *arr, int size)
{
    char *p;
    if (!arr || size <= 0 || arr->enabled) { if (arr && arr->enabled) printf("CircArray already enabled"); return; }
    p = (char *)calloc((size_t)size, 1u); if (!p) return;
    arr->buf = p; arr->size = (uint32_t)size; arr->enabled = true; arr->n_r = arr->n_w = 0u;
}
bool buf_empty(CircArray *arr) { return !arr || arr->n_r == arr->n_w; }
bool buf_full(CircArray *arr) { return arr && arr->size && !buf_empty(arr) && arr->n_r % arr->size == arr->n_w % arr->size; }
int buf_available(CircArray *arr) { return arr ? (int)(arr->n_w - arr->n_r) : 0; }
int buf_putbyte(CircArray *arr, char c) { if (!arr || !arr->enabled || !arr->buf || !arr->size || buf_full(arr)) return 0; arr->buf[arr->n_w++ % arr->size] = c; return 1; }
char buf_getbyte(CircArray *arr) { if (!arr || !arr->enabled || !arr->buf || !arr->size || buf_empty(arr)) return 0; return arr->buf[arr->n_r++ % arr->size]; }
bool buf_resize(CircArray *arr, int size) { char *p; if (!arr || size <= 0) return false; p = (char *)realloc(arr->buf, (size_t)size); if (!p) return false; arr->buf = p; arr->size = (uint32_t)size; return true; }
void buf_clear(CircArray *arr) { if (arr && arr->buf && arr->size) memset(arr->buf, 0, arr->size); }
bool buf_delete(CircArray *arr) { if (!arr) return false; buf_clear(arr); free(arr->buf); memset(arr, 0, sizeof(*arr)); return true; }

void init_usart1(int baud)
{
    if (!baud) { baud = UART_BAUD; printf("Warning: baud 0, using 9600"); }
    usart1_baud_used = (uint32_t)baud; usart1_sr = USART_SR_TXE; usart1_dr = 0u; usart1_enabled = true;
    if (msg.enabled) buf_delete(&msg); initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
}
void USART1_IRQHandler(void) { if (usart1_sr & USART_SR_RXNE) { (void)buf_putbyte(&msg, (char)usart1_dr); usart1_sr &= ~USART_SR_RXNE; } }
void usart1_send(volatile char *s)
{
    if (!s) return;
#ifndef DISCOBOT_TARGET
    if (!usart1_enabled) return;
#endif
    while (*s) {
        while (!(usart1_sr & USART_SR_TXE)) {
#ifndef DISCOBOT_TARGET
            return;
#endif
        }
        if (usart1_tx_length < sizeof(usart1_tx_log)) usart1_tx_log[usart1_tx_length++] = *s;
        ++s;
    }
}
uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)(int8_t)(uint8_t)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }
void discobot_usart1_inject_rx(uint8_t value, bool rxne) { usart1_dr = value; if (rxne) { usart1_sr |= USART_SR_RXNE; USART1_IRQHandler(); } else usart1_sr &= ~USART_SR_RXNE; }

int func2(int R0) { return (int)((uint32_t)R0 + 1u); }
int func1(int R0) { return func2(R0); }