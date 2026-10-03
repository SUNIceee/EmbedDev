#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;

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
static ButtonState last_button_state = ButtonIsReleased;
static uint32_t t_prev = 0;
static int16_t acc_raw[3] = {0, 0, 0};
static uint16_t adc_raw_value = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;
static uint8_t usart1_dr = 0;
static bool usart1_rxne = false;
static uint32_t usart1_sr = 0x40u;
static bool systick_failed = false;

static bool gpioa_motor_configured = false;
static bool gpiod_led_configured = false;
static bool button_configured = false;
static bool accelerometer_configured = false;
static bool temperature_configured = false;
static bool rng_configured = false;
static bool usart1_configured = false;
static bool usart1_rxne_interrupt_enabled = false;
static bool usart1_nvic_priority_zero = false;
static bool adc_eoc_ready = true;

static void set_gpioa_pin(unsigned pin, uint8_t value)
{
    if (pin < 16u) {
        GPIOA_output[pin] = value ? 1u : 0u;
    }
}

static void set_gpiod_pin(unsigned pin, uint8_t value)
{
    if (pin < 16u) {
        GPIOD_output[pin] = value ? 1u : 0u;
    }
}

static void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 168000000u;
}

static int SysTick_Config_adapter(uint32_t ticks)
{
    SysTick_reload = ticks;
    return ticks == 0u ? 1 : 0;
}

static void configure_gpioa_motor_outputs(void)
{
    gpioa_motor_configured = true;
}

static void configure_gpiod_led_outputs(void)
{
    gpiod_led_configured = true;
}

static void configure_button_input(void)
{
    button_configured = true;
}

static int configure_accelerometer_device(void)
{
    accelerometer_configured = true;
    return 0;
}

static void configure_temperature_adc(void)
{
    temperature_configured = true;
    adc_eoc_ready = true;
}

static void configure_rng_device(void)
{
    rng_configured = true;
}

static void configure_usart1_device(int baud)
{
    usart1_baud_used = (uint32_t)baud;
    usart1_configured = true;
    usart1_rxne_interrupt_enabled = true;
    usart1_nvic_priority_zero = true;
    usart1_sr = 0x40u;
}

void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}

void init_systick(void)
{
    SystemCoreClockUpdate();
    if (SysTick_Config_adapter(SystemCoreClock / 1000u) != 0) {
        systick_failed = true;
#ifdef DISCOBOT_TARGET
        while (1) {
        }
#endif
    } else {
        systick_failed = false;
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
    configure_button_input();
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> (unsigned)i) & 1u);
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

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    }
    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z))) * 180.0f / PI;
    }
}

void init_GPIO_A1A2A3A4_output(void)
{
    configure_gpioa_motor_outputs();
    set_gpioa_pin(1u, 0u);
    set_gpioa_pin(2u, 0u);
    set_gpioa_pin(3u, 0u);
    set_gpioa_pin(4u, 0u);
}

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        set_gpioa_pin(1u, 1u);
        set_gpioa_pin(2u, 0u);
        break;
    case BACKWARD:
        set_gpioa_pin(1u, 0u);
        set_gpioa_pin(2u, 1u);
        break;
    case STOP:
    default:
        set_gpioa_pin(1u, 0u);
        set_gpioa_pin(2u, 0u);
        break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;
    switch (direc) {
    case FORWARD:
        set_gpioa_pin(3u, 1u);
        set_gpioa_pin(4u, 0u);
        break;
    case BACKWARD:
        set_gpioa_pin(3u, 0u);
        set_gpioa_pin(4u, 1u);
        break;
    case STOP:
    default:
        set_gpioa_pin(3u, 0u);
        set_gpioa_pin(4u, 0u);
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

bool dispatch_uart_command(int command)
{
    if (command >= 0 && command < MAXIDSIZE) {
        msgid = (_ID)command;
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
        (void)dispatch_uart_command((int)usart1_readc());
    }
    checkbutton();
    execute_tasks();
    if ((uint32_t)(msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }
}

void init_LED_pins(void)
{
    configure_gpiod_led_outputs();
    set_gpiod_pin(12u, 0u);
    set_gpiod_pin(13u, 0u);
    set_gpiod_pin(14u, 0u);
    set_gpiod_pin(15u, 0u);
}

void LED_On(int i)
{
    if (i >= 0 && i < 4) {
        set_gpiod_pin((unsigned)(12 + i), 1u);
    }
}

void LED_Off(int i)
{
    if (i >= 0 && i < 4) {
        set_gpiod_pin((unsigned)(12 + i), 0u);
    }
}

int init_accelerometers(void)
{
    return configure_accelerometer_device();
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg)
{
    acc_raw[0] = x_mg;
    acc_raw[1] = y_mg;
    acc_raw[2] = z_mg;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }
    b[0] = ((float)acc_raw[0]) / 1000.0f;
    b[1] = ((float)acc_raw[1]) / 1000.0f;
    b[2] = ((float)acc_raw[2]) / 1000.0f;
}

void init_temperature_sensor(void)
{
    configure_temperature_adc();
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = raw;
    adc_eoc_ready = true;
}

float read_temperature_sensor(void)
{
    float temp;
#ifdef DISCOBOT_TARGET
    while (!adc_eoc_ready) {
    }
#else
    if (!adc_eoc_ready) {
        abort();
    }
#endif
    temp = (float)adc_raw_value;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void init_rng(void)
{
    configure_rng_device();
}

void discobot_set_rng(bool ready, uint32_t value)
{
    rng_ready = ready;
    rng_value = value;
}

uint32_t get_random_number(void)
{
#ifdef DISCOBOT_TARGET
    while (!rng_ready) {
    }
#else
    if (!rng_ready) {
        abort();
    }
#endif
    return rng_value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    int i;
    if (myfunc == NULL) {
        return;
    }
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)((long)(interval_sec * 1000.0f));
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void)
{
    int i;
    for (i = 0; i < MAXNUMTASKS; i++) {
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
        printf("Error: CircArray already enabled\n");
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
    if (arr == NULL) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || arr->size == 0u) {
        return false;
    }
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr)
{
    if (arr == NULL) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr)
{
    char c;
    if (arr == NULL || arr->buf == NULL || arr->size == 0u || !arr->enabled || buf_empty(arr)) {
        return 0;
    }
    c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_resize(CircArray *arr, int newSize)
{
    char *newbuf;
    if (arr == NULL || newSize <= 0) {
        return false;
    }
    newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (newbuf == NULL) {
        return false;
    }
    if ((uint32_t)newSize > arr->size) {
        memset(newbuf + arr->size, 0, (size_t)((uint32_t)newSize - arr->size));
    }
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }
    memset(arr->buf, 0, arr->size);
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
        printf("Warning: baud was 0, using 9600\n");
    }
    configure_usart1_device(baud);
    if (msg.enabled) {
        (void)buf_delete(&msg);
    }
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    usart1_rxne = false;
}

void USART1_IRQHandler(void)
{
    if (usart1_rxne) {
        (void)buf_putbyte(&msg, (char)usart1_dr);
        usart1_rxne = false;
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    usart1_dr = value;
    usart1_rxne = rxne;
    USART1_IRQHandler();
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s != '\0') {
#ifdef DISCOBOT_TARGET
        while ((usart1_sr & 0x40u) == 0u) {
        }
#else
        if ((usart1_sr & 0x40u) == 0u) {
            abort();
        }
#endif
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1u) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }
        s++;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    return (char)((int8_t)buf_getbyte(&msg));
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

int func2(int R0)
{
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0)
{
    return func2(R0);
}