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
motor_command_fn flookup[9] = { move_forward, move_backward, move_forward_soft_left, move_forward_soft_right, move_backward_soft_left, move_backward_soft_right, move_spin_right, move_spin_left, stop };
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
static ButtonState laststate = ButtonIsReleased;
static uint32_t main_t_prev = 0;
static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t adc_raw_value = 0;
static bool rng_ready_value = false;
static uint32_t rng_data_value = 0;
static uint8_t host_usart1_dr = 0;
static bool host_usart1_rxne = false;
static bool rng_clock_enabled = false;
static bool rng_peripheral_enabled = false;
static bool adc_temperature_configured = false;
static bool adc_temperature_enabled = false;
static bool adc_conversion_started = false;

static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}

static int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return (ticks == 0u) ? 1 : 0;
}

static void write_motor_pins(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
    GPIOA_output[1] = p1 ? 1u : 0u;
    GPIOA_output[2] = p2 ? 1u : 0u;
    GPIOA_output[3] = p3 ? 1u : 0u;
    GPIOA_output[4] = p4 ? 1u : 0u;
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
    if (SysTick_Config(SystemCoreClock / 1000u) != 0) {
        while (1) { }
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
    if ((uint32_t)(msTicks - main_t_prev) > 1000u) {
        main_t_prev = msTicks;
    }
    if (usart1_available() > 0u) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }
    checkbutton();
    execute_tasks();
}

void init_GPIO_A1A2A3A4_output(void) {
    write_motor_pins(0u, 0u, 0u, 0u);
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[1] = 1u;
        GPIOA_output[2] = 0u;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0u;
        GPIOA_output[2] = 1u;
    } else {
        GPIOA_output[1] = 0u;
        GPIOA_output[2] = 0u;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[3] = 1u;
        GPIOA_output[4] = 0u;
    } else if (direc == BACKWARD) {
        GPIOA_output[3] = 0u;
        GPIOA_output[4] = 1u;
    } else {
        GPIOA_output[3] = 0u;
        GPIOA_output[4] = 0u;
    }
}

void move_forward(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(FORWARD, 1.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(STOP, 0.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(STOP, 0.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD, 1.0f); set_right_motor_direc(BACKWARD, 1.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD, 1.0f); set_right_motor_direc(FORWARD, 1.0f); }
void stop(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(STOP, 0.0f); }

int read_buttonc(int i) {
    if (i > 3) {
        return -1;
    }
    if (i < 0) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
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
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
            default:
                break;
        }
        laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float pi = 3.14159265358979323846f;
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / pi;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / pi;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_LED_pins(void) {
    GPIOD_output[12] = 0u;
    GPIOD_output[13] = 0u;
    GPIOD_output[14] = 0u;
    GPIOD_output[15] = 0u;
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1u;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0u;
    }
}

int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b == NULL) {
        return;
    }
    b[0] = accel_raw_mg[0] / 1000.0f;
    b[1] = accel_raw_mg[1] / 1000.0f;
    b[2] = accel_raw_mg[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg;
    accel_raw_mg[1] = y_mg;
    accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
    adc_temperature_configured = true;
    adc_temperature_enabled = true;
    adc_conversion_started = false;
}

float read_temperature_sensor(void) {
    adc_conversion_started = true;
    while (!adc_temperature_enabled) { }
    uint16_t raw = adc_raw_value;
    float temp = (float)raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw_value = raw;
}

void init_rng(void) {
    rng_clock_enabled = true;
    rng_peripheral_enabled = true;
}

uint32_t get_random_number(void) {
    while (rng_ready_value == false) {
#ifndef DISCOBOT_TARGET
        abort();
#endif
    }
    (void)rng_clock_enabled;
    (void)rng_peripheral_enabled;
    return rng_data_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready_value = ready;
    rng_data_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    for (int i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
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
        printf("CircArray already enabled");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1u);
    if (arr->buf == NULL) {
        arr->size = 0u;
        arr->enabled = false;
        arr->n_r = 0u;
        arr->n_w = 0u;
        return;
    }
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0u;
    arr->n_w = 0u;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
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
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
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
    if (arr == NULL || arr->size == 0u) {
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
    void *tmp = realloc(arr->buf, (size_t)newSize);
    if (tmp == NULL) {
        return false;
    }
    arr->buf = (char *)tmp;
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
    arr->size = 0u;
    arr->n_r = 0u;
    arr->n_w = 0u;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
    }
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) {
        (void)buf_delete(&msg);
    }
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
    if (s == NULL) {
        return;
    }
    while (*s) {
        char c = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = c;
        }
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
    host_usart1_dr = value;
    host_usart1_rxne = rxne;
    USART1_IRQHandler();
}

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    uint32_t v = (uint32_t)R0;
    v += 1u;
    return (int)v;
}