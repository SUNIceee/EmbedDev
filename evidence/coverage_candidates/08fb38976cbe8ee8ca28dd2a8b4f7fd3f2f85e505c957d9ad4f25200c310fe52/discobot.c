/* DiscoBot core host-compatible implementation. */

#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdbool.h>

#define DISCOBOT_PI 3.14159265358979323846f

/* Public global objects. */
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
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* Host model state for injected sensors and button transitions. */
static int16_t host_accel_x_mg = 0;
static int16_t host_accel_y_mg = 0;
static int16_t host_accel_z_mg = 0;
static uint16_t host_adc_raw = 0;
static volatile bool host_adc_eoc = false;
static volatile bool host_rng_ready = false;
static uint32_t host_rng_value = 0;
static volatile bool host_rng_timeout_error = false;
static ButtonState checkbutton_laststate = ButtonIsReleased;
static uint32_t main_loop_last_ms = 0;

static void discobot_SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000u;
}

static int discobot_SysTick_Config(uint32_t ticks)
{
    SysTick_reload = ticks;
    return 0;
}

void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}

void init_systick(void)
{
    discobot_SystemCoreClockUpdate();
    uint32_t ticks = SystemCoreClock / 1000u;
    if (discobot_SysTick_Config(ticks) != 0) {
        for (;;) {
            /* SysTick configuration failure path. */
        }
    }
}

void init_button(void)
{
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    checkbutton_laststate = ButtonIsReleased;
}

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

    uint8_t pa1 = 0;
    uint8_t pa2 = 0;

    if (direc == FORWARD) {
        pa1 = 1;
        pa2 = 0;
    } else if (direc == BACKWARD) {
        pa1 = 0;
        pa2 = 1;
    } else {
        pa1 = 0;
        pa2 = 0;
    }

    GPIOA_output[1] = pa1;
    GPIOA_output[2] = pa2;
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;

    uint8_t pa3 = 0;
    uint8_t pa4 = 0;

    if (direc == FORWARD) {
        pa3 = 1;
        pa4 = 0;
    } else if (direc == BACKWARD) {
        pa3 = 0;
        pa4 = 1;
    } else {
        pa3 = 0;
        pa4 = 0;
    }

    GPIOA_output[3] = pa3;
    GPIOA_output[4] = pa4;
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

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (unsigned int)i) & 1u);
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
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
    ButtonState current = buttstate;

    if (current != checkbutton_laststate) {
        switch (current) {
        case ButtonIsPressed:
            break;
        case ButtonIsReleased:
            break;
        default:
            break;
        }
        checkbutton_laststate = current;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    if (pitch == NULL || roll == NULL) {
        return;
    }

    float denom = sqrtf(acc_y * acc_y + acc_z * acc_z);
    float pitch_val = atan2f(-acc_x, denom) * (180.0f / DISCOBOT_PI);
    float roll_val = atan2f(acc_y, acc_z) * (180.0f / DISCOBOT_PI);

    *pitch = pitch_val;
    *roll = roll_val;
}

void init_LED_pins(void)
{
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 1;
}

void LED_Off(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }
    GPIOD_output[12 + i] = 0;
}

int init_accelerometers(void)
{
    /* Host model: the TM LIS3DSH init is assumed successful. */
    return 0;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }

    b[0] = (float)host_accel_x_mg / 1000.0f;
    b[1] = (float)host_accel_y_mg / 1000.0f;
    b[2] = (float)host_accel_z_mg / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    host_accel_x_mg = x_mg;
    host_accel_y_mg = y_mg;
    host_accel_z_mg = z_mg;
}

void init_temperature_sensor(void)
{
    host_adc_eoc = false;
}

float read_temperature_sensor(void)
{
    /* Start conversion. */
    host_adc_eoc = true;

    /* Wait for EOC. */
    while (host_adc_eoc == false) {
        /* Blocking wait; target hardware behaves the same. */
    }

    uint16_t raw = host_adc_raw;
    float temp = (float)raw;

    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    host_adc_eoc = false;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw)
{
    host_adc_raw = raw;
}

void init_rng(void)
{
    host_rng_ready = false;
    host_rng_value = 0;
    host_rng_timeout_error = false;
}

uint32_t get_random_number(void)
{
#ifndef DISCOBOT_TARGET
    /* Host mode: bounded wait to prevent a hung test suite. */
    uint32_t wait_guard = 0u;

    while (!host_rng_ready) {
        if (++wait_guard >= 1000000u) {
            host_rng_timeout_error = true;
            fprintf(stderr, "RNG not ready after timeout\r\n");
            return 0u;
        }
    }
#else
    /* Target hardware: original unbounded DRDY wait. */
    while (!host_rng_ready) {
        /* Blocking DRDY wait. */
    }
#endif

    host_rng_timeout_error = false;
    return host_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value)
{
    host_rng_ready = ready;
    host_rng_value = value;

    if (ready) {
        host_rng_timeout_error = false;
    }
}

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
    if (usart1_available() > 0u) {
        int c = usart1_readc();
        if (c >= 0 && c <= 8) {
            (void)dispatch_uart_command(c);
        }
    }

    checkbutton();
    execute_tasks();

    if ((msTicks - main_loop_last_ms) > 1000u) {
        main_loop_last_ms = msTicks;
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    if (myfunc == NULL) {
        return;
    }

    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
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
        if (t->task != NULL && msTicks >= (t->last_called + t->msinterval)) {
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
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

