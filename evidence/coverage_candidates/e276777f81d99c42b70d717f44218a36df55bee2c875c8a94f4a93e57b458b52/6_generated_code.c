/* DiscoBot Test-Compatible v3 C11 library implementation. */
#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define USART_SR_TXE 0x40u
#define USART_SR_RXNE 0x20u

typedef struct HostUSART {
    uint32_t SR;
    uint32_t DR;
    bool enabled;
} HostUSART;

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;

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
static HostUSART host_usart1 = {USART_SR_TXE, 0u, false};
static int16_t accel_raw_x_mg = 0;
static int16_t accel_raw_y_mg = 0;
static int16_t accel_raw_z_mg = 0;
static uint16_t adc_raw_value = 0;
static volatile bool rng_ready = false;
static uint32_t rng_value = 0;
static volatile ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_loop_prev_tick = 0;

static void set_pin(uint8_t pins[16], int pin, uint8_t value)
{
    if (pin >= 0 && pin < 16) {
        pins[pin] = value ? 1u : 0u;
    }
}

static void set_left_outputs(uint8_t pa1, uint8_t pa2)
{
    set_pin(GPIOA_output, 1, pa1);
    set_pin(GPIOA_output, 2, pa2);
}

static void set_right_outputs(uint8_t pa3, uint8_t pa4)
{
    set_pin(GPIOA_output, 3, pa3);
    set_pin(GPIOA_output, 4, pa4);
}

void move_forward(void);
void move_backward(void);
void move_forward_soft_left(void);
void move_forward_soft_right(void);
void move_backward_soft_left(void);
void move_backward_soft_right(void);
void move_spin_right(void);
void move_spin_left(void);
void stop(void);

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

motor_command_fn callme = stop;

void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}

static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000u;
}

static int SysTick_Config(uint32_t ticks)
{
    SysTick_reload = ticks;
    return ticks == 0u ? 1 : 0;
}

void init_systick(void)
{
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000u) != 0) {
        for (;;) {
        }
    }
}

void init_system(void)
{
    SystemInit();
    init_systick();
    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}

void init_button(void)
{
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

bool dispatch_uart_command(int command)
{
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        if (callme != NULL) {
            callme();
        }
        return true;
    }

    return false;
}

void main_loop_iteration(void)
{
    if (usart1_available() > 0u) {
        char c = usart1_readc();

        if (c >= 0 && c < MAXIDSIZE) {
            (void)dispatch_uart_command((int)c);
        }
    }

    checkbutton();
    execute_tasks();

    if ((uint32_t)(msTicks - main_loop_prev_tick) > 1000u) {
        main_loop_prev_tick = msTicks;
    }
}

void init_GPIO_A1A2A3A4_output(void)
{
    set_pin(GPIOA_output, 1, 0);
    set_pin(GPIOA_output, 2, 0);
    set_pin(GPIOA_output, 3, 0);
    set_pin(GPIOA_output, 4, 0);
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;

    switch (direc) {
    case FORWARD:
        set_left_outputs(1, 0);
        break;
    case BACKWARD:
        set_left_outputs(0, 1);
        break;
    case STOP:
    default:
        set_left_outputs(0, 0);
        break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;

    switch (direc) {
    case FORWARD:
        set_right_outputs(1, 0);
        break;
    case BACKWARD:
        set_right_outputs(0, 1);
        break;
    case STOP:
    default:
        set_right_outputs(0, 0);
        break;
    }
}

void move_forward(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void move_backward(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_forward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void move_forward_soft_right(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_backward_soft_right(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_spin_left(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void stop(void)
{
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

int read_buttonc(int i)
{
    if (i > 3 || i < 0) {
        return -1;
    }

    return (int)((GPIOA_IDR >> (uint32_t)i) & 1u);
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
        default:
            break;
        }

        last_button_state = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    float computed_roll = atan2f(acc_y, acc_z) * 180.0f / (float)M_PI;
    float yz = sqrtf((acc_y * acc_y) + (acc_z * acc_z));
    float computed_pitch = atan2f(-acc_x, yz) * 180.0f / (float)M_PI;

    if (pitch != NULL) {
        *pitch = computed_pitch;
    }

    if (roll != NULL) {
        *roll = computed_roll;
    }
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void init_LED_pins(void)
{
    set_pin(GPIOD_output, 12, 0);
    set_pin(GPIOD_output, 13, 0);
    set_pin(GPIOD_output, 14, 0);
    set_pin(GPIOD_output, 15, 0);
}

void LED_On(int i)
{
    if (i >= 0 && i < 4) {
        set_pin(GPIOD_output, 12 + i, 1);
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i < 4) {
        set_pin(GPIOD_output, 12 + i, 0);
    }
}

int init_accelerometers(void)
{
    return 0;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }

    b[0] = (float)accel_raw_x_mg / 1000.0f;
    b[1] = (float)accel_raw_y_mg / 1000.0f;
    b[2] = (float)accel_raw_z_mg / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    accel_raw_x_mg = x_mg;
    accel_raw_y_mg = y_mg;
    accel_raw_z_mg = z_mg;
}

void init_temperature_sensor(void)
{
}

float read_temperature_sensor(void)
{
    float temp = (float)adc_raw_value;

    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;

    return temp;
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = (uint16_t)(raw & 0x0FFFu);
}

void init_rng(void)
{
    rng_ready = false;
    rng_value = 0u;
}

uint32_t get_random_number(void)
{
    while (!rng_ready) {
    }

    return rng_value;
}

void discobot_set_rng(bool ready, uint32_t value)
{
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    uint32_t interval_ms;

    if (myfunc == NULL) {
        return;
    }

    if (interval_sec <= 0.0f) {
        interval_ms = 0u;
    } else {
        interval_ms = (uint32_t)(interval_sec * 1000.0f);
    }

    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = interval_ms;
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void)
{
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *task = &timed_tasks[i];

        if (task->task != NULL &&
            msTicks >= task->last_called + task->msinterval) {
            task->task();
            task->last_called = msTicks;
            task->numcalls++;
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

void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL || size <= 0) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
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

bool buf_empty(CircArray *arr)
{
    return arr == NULL || arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || arr->size == 0u || buf_empty(arr)) {
        return false;
    }

    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled) {
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
    char c;

    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;

    return c;
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
    char *new_buf;

    if (arr == NULL || newSize <= 0) {
        return false;
    }

    new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (new_buf == NULL) {
        return false;
    }

    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    if (!arr->enabled) {
        arr->enabled = true;
    }

    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }

    memset(arr->buf, 0, (size_t)arr->size);
}

bool buf_delete(CircArray *arr)
{
    if (arr == NULL) {
        return false;
    }

    buf_clear(arr);
    free(arr->buf);

    arr->buf = NULL;
    arr->size = 0u;
    arr->enabled = false;
    arr->n_r = 0u;
    arr->n_w = 0u;

    return true;
}

void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "USART1 baud 0, using 9600\n");
    }

    usart1_baud_used = (uint32_t)baud;
    host_usart1.SR = USART_SR_TXE;
    host_usart1.DR = 0u;
    host_usart1.enabled = true;

    if (msg.enabled) {
        (void)buf_delete(&msg);
    }

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

    usart1_tx_length = 0u;
    memset(usart1_tx_log, 0, sizeof(usart1_tx_log));
}

void USART1_IRQHandler(void)
{
    if ((host_usart1.SR & USART_SR_RXNE) != 0u) {
        (void)buf_putbyte(&msg, (char)(uint8_t)host_usart1.DR);
        host_usart1.SR &= ~USART_SR_RXNE;
    }
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        while ((host_usart1.SR & USART_SR_TXE) == 0u) {
        }

        if (usart1_tx_length < sizeof(usart1_tx_log) - 1u) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }

        host_usart1.DR = (uint8_t)*s;
        s++;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    return (char)(int8_t)(uint8_t)buf_getbyte(&msg);
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    host_usart1.DR = value;

    if (rxne) {
        host_usart1.SR |= USART_SR_RXNE;
    } else {
        host_usart1.SR &= ~USART_SR_RXNE;
    }

    USART1_IRQHandler();
}

int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}
