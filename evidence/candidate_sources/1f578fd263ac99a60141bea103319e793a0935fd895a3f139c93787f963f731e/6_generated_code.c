#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = 0;
uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000U;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {0};
static ButtonState laststate = ButtonIsReleased;
static uint32_t main_t_prev = 0;
static int16_t accel_raw_x_mg = 0, accel_raw_y_mg = 0, accel_raw_z_mg = 0;
static uint16_t temperature_adc_raw = 0;
static volatile bool rng_ready = true;
static uint32_t rng_value = 0;
static bool accel_ready = false, temp_ready = false, rng_enabled = false;
static bool rng_blocked_observed = false;
static uint8_t host_usart1_dr = 0;
static bool host_usart1_rxne = false;

motor_command_fn flookup[9] = { move_forward, move_backward, move_forward_soft_left, move_forward_soft_right, move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop };

static void SystemCoreClockUpdate(void) { SystemCoreClock = 168000000U; }
static int SysTick_Config(uint32_t ticks) { SysTick_reload = ticks; return (ticks == 0U) ? 1 : 0; }

static void set_pair(uint8_t a, uint8_t b, int direc) {
    if (direc == FORWARD) { GPIOA_output[a] = 1U; GPIOA_output[b] = 0U; }
    else if (direc == BACKWARD) { GPIOA_output[a] = 0U; GPIOA_output[b] = 1U; }
    else { GPIOA_output[a] = 0U; GPIOA_output[b] = 0U; }
}

void SystemInit(void) { SystemCoreClock = 168000000U; }

void init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U) != 0) {
        for (;;) { }
    }
}

void init_LED_pins(void) { GPIOD_output[12] = 0; GPIOD_output[13] = 0; GPIOD_output[14] = 0; GPIOD_output[15] = 0; }

void init_button(void) { GPIOA_IDR &= ~1U; b_i = 0; buttstate = ButtonIsReleased; laststate = ButtonIsReleased; }

int init_accelerometers(void) { accel_ready = true; (void)accel_ready; return 0; }
void init_rng(void) { rng_enabled = true; (void)rng_enabled; }
void init_temperature_sensor(void) { temp_ready = true; (void)temp_ready; }

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

void init_GPIO_A1A2A3A4_output(void) { GPIOA_output[1] = GPIOA_output[2] = GPIOA_output[3] = GPIOA_output[4] = 0U; }
void set_left_motor_direc(int direc, float speed) { (void)speed; set_pair(1U, 2U, direc); }
void set_right_motor_direc(int direc, float speed) { (void)speed; set_pair(3U, 4U, direc); }
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
        if (callme != NULL) { callme(); }
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if ((uint32_t)(msTicks - main_t_prev) > 1000U) { main_t_prev = msTicks; }
    if (usart1_available() > 0U) { (void)dispatch_uart_command((int)usart1_readc()); }
    checkbutton();
    execute_tasks();
}

int read_buttonc(int i) { if (i > 3 || i < 0) return -1; return (int)((GPIOA_IDR >> (uint32_t)i) & 1U); }

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
    if (laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed: break;
            case ButtonIsReleased: break;
            default: break;
        }
        laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;
    if (roll != NULL) { *roll = atan2f(acc_y, acc_z) * 180.0f / PI; }
    if (pitch != NULL) { *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI; }
}

void discobot_set_button_level(bool high) { if (high) GPIOA_IDR |= 1U; else GPIOA_IDR &= ~1U; }
void LED_On(int i) { if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 1U; } }
void LED_Off(int i) { if (i >= 0 && i < 4) { GPIOD_output[12 + i] = 0U; } }

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) { accel_raw_x_mg = x_mg; accel_raw_y_mg = y_mg; accel_raw_z_mg = z_mg; }
void read_accelerometers(float b[3]) { if (b != NULL) { b[0] = accel_raw_x_mg / 1000.0f; b[1] = accel_raw_y_mg / 1000.0f; b[2] = accel_raw_z_mg / 1000.0f; } }

void discobot_set_adc_raw(uint16_t raw) { temperature_adc_raw = raw; }
float read_temperature_sensor(void) {
    float temp = (float)temperature_adc_raw;
    temp = temp / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_rng(bool ready, uint32_t value) { rng_ready = ready; rng_value = value; rng_blocked_observed = false; }
uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (!rng_ready) { }
    return rng_value;
#else
    if (!rng_ready) {
        rng_blocked_observed = true;
        abort();
    }
    rng_blocked_observed = false;
    return rng_value;
#endif
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0U;
            timed_tasks[i].numcalls = 0U;
            break;
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

void printtimes(void) { for (int i = 0; i < MAXNUMTASKS; ++i) { if (timed_tasks[i].task != NULL) { printf("t%d=%d", i, (int)timed_tasks[i].numcalls); } } }

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL || size <= 0) { return; }
    if (arr->enabled) { printf("CircArray already enabled"); return; }
    arr->buf = (char *)calloc((size_t)size, 1U);
    if (arr->buf == NULL) { arr->size = 0U; arr->enabled = false; arr->n_r = 0U; arr->n_w = 0U; return; }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0U;
    arr->n_w = 0U;
}

bool buf_empty(CircArray *arr) { if (arr == NULL) return true; return arr->n_r == arr->n_w; }
bool buf_full(CircArray *arr) { if (arr == NULL || arr->size == 0U) return false; return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size)); }
int buf_available(CircArray *arr) { if (arr == NULL) return 0; return (int)(arr->n_w - arr->n_r); }

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) return 0;
    if (!buf_full(arr)) { arr->buf[arr->n_w % arr->size] = c; arr->n_w++; return 1; }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0U) return 0;
    if (!buf_empty(arr)) { char c = arr->buf[arr->n_r % arr->size]; arr->n_r++; return c; }
    return 0;
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) return false;
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) return false;
    arr->buf = (char *)tmp;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) { if (arr != NULL && arr->buf != NULL && arr->size > 0U) { memset(arr->buf, 0, arr->size); } }

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    buf_clear(arr);
    free(arr->buf);
    arr->buf = NULL;
    arr->size = 0U;
    arr->n_r = 0U;
    arr->n_w = 0U;
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
    if (host_usart1_rxne) {
        char c = (char)host_usart1_dr;
        (void)buf_putbyte(&msg, c);
        host_usart1_rxne = false;
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s) {
        char c = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) { usart1_tx_log[usart1_tx_length++] = c; }
    }
}

uint8_t usart1_read(void) { return (uint8_t)buf_getbyte(&msg); }
char usart1_readc(void) { return (char)buf_getbyte(&msg); }
uint32_t usart1_available(void) { return (uint32_t)buf_available(&msg); }

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    host_usart1_dr = value;
    host_usart1_rxne = rxne;
    if (rxne) { USART1_IRQHandler(); }
}

int func2(int R0) { return (int)((uint32_t)R0 + 1U); }
int func1(int R0) { return func2(R0); }