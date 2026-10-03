#include "6_generated_code.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

static CircArray rx_msg;
static bool usart1_rxne;
static uint8_t usart1_dr;
static bool usart1_txe = true;
static int16_t acc_raw_x;
static int16_t acc_raw_y;
static int16_t acc_raw_z;
static uint16_t adc_raw_value;
static volatile bool rng_ready = false;
static volatile uint32_t rng_value = 0;
static ButtonState last_button_state = ButtonIsReleased;

static void SystemCoreClockUpdate(void) {
    /* Host model: SystemCoreClock is initialized by SystemInit(), but tests
       may set it to a low value before calling init_systick() to exercise the
       SysTick_Config error path.  Do not overwrite an injected value. */
}

static int SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    if (ticks == 0) {
        return 1;
    }
    return 0;
}

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    uint32_t ticks = SystemCoreClock / 1000;
    if (SysTick_Config(ticks)) {
#ifdef DISCOBOT_TARGET
        for (;;) {}
#else
        SysTick_reload = 0;
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

void init_LED_pins(void) {
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

int init_accelerometers(void) {
    return 0;
}

void init_rng(void) {
    rng_ready = false;
    rng_value = 0;
}

void init_temperature_sensor(void) {
    /* Host model: ADC configuration is handled by injected raw values. */
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void initCircArray(CircArray *arr, int size) {
    if (!arr) return;
    if (arr->enabled) {
        printf("CircArray already initialized\r\n");
        return;
    }
    if (size <= 0) return;
    arr->buf = (char *)calloc((size_t)size, 1);
    if (!arr->buf) return;
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || arr->size == 0) return 0;
    if (!buf_full(arr)) {
        arr->buf[arr->n_w % arr->size] = c;
        arr->n_w++;
        return 1;
    }
    return 0;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || arr->size == 0) return 0;
    if (!buf_empty(arr)) {
        char c = arr->buf[arr->n_r % arr->size];
        arr->n_r++;
        return c;
    }
    return 0;
}

bool buf_empty(CircArray *arr) {
    if (!arr) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (!arr || arr->size == 0) return false;
    if (buf_empty(arr)) return false;
    return (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

int buf_available(CircArray *arr) {
    if (!arr) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (!arr || !arr->buf || newSize <= 0) return false;
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!newbuf) return false;
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    if (arr->buf) {
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

void buf_clear(CircArray *arr) {
    if (!arr || !arr->buf || arr->size == 0) return;
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = UART_BAUD;
        printf("Warning: invalid baud, using 9600\r\n");
    }
    usart1_baud_used = (uint32_t)baud;
    usart1_rxne = false;
    usart1_dr = 0;
    usart1_txe = true;
    initCircArray(&rx_msg, CIRC_BUFFER_MIN_SIZE);
}

void USART1_IRQHandler(void) {
    if (usart1_rxne) {
        char c = (char)usart1_dr;
        usart1_rxne = false;
        (void)buf_putbyte(&rx_msg, c);
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        usart1_dr = value;
        usart1_rxne = true;
        USART1_IRQHandler();
    } else {
        usart1_rxne = false;
    }
}

void usart1_send(volatile char *s) {
    if (!s) return;
    while (*s) {
        while (!usart1_txe) {}
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length++] = *s;
        }
        s++;
        usart1_txe = true;
    }
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&rx_msg);
}

char usart1_readc(void) {
    return (char)buf_getbyte(&rx_msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&rx_msg);
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) return -1;
    return (int)((GPIOA_IDR >> i) & 1u);
}

void discobot_set_button_level(bool high) {
    if (high) GPIOA_IDR |= 1u;
    else GPIOA_IDR &= ~1u;
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
    if (last_button_state != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
            case ButtonIsReleased:
            default:
                break;
        }
        last_button_state = buttstate;
    }
}

void set_left_motor_direc(int direc, float speed) {
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

void set_right_motor_direc(int direc, float speed) {
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

void move_forward(void) {
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void move_backward(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void stop(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme) callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        int c = (int)usart1_readc();
        (void)dispatch_uart_command(c);
    }
    checkbutton();
    execute_tasks();
    static uint32_t last_second = 0;
    if (msTicks - last_second > 1000) {
        last_second = msTicks;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (roll) {
        *roll = (float)(atan2(acc_y, acc_z) * 180.0 / M_PI);
    }
    if (pitch) {
        *pitch = (float)(atan2(-acc_x, sqrt((double)(acc_y * acc_y + acc_z * acc_z))) * 180.0 / M_PI);
    }
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0;
    }
}

void read_accelerometers(float b[3]) {
    if (!b) return;
    b[0] = acc_raw_x / 1000.0f;
    b[1] = acc_raw_y / 1000.0f;
    b[2] = acc_raw_z / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    acc_raw_x = x_mg;
    acc_raw_y = y_mg;
    acc_raw_z = z_mg;
}

void discobot_set_adc_raw(uint16_t raw) {
    adc_raw_value = raw;
}

float read_temperature_sensor(void) {
    float temp;
    uint16_t raw = adc_raw_value;
    temp = (float)raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

uint32_t get_random_number(void) {
#ifdef DISCOBOT_TARGET
    while (!rng_ready) {}
    return rng_value;
#else
    volatile uint32_t watchdog = 0;
    while (!rng_ready) {
        if (++watchdog >= 1000000UL) {
            fprintf(stderr, "get_random_number: RNG DRDY not ready; host test cannot block forever\r\n");
            abort();
        }
    }
    return rng_value;
#endif
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (!myfunc) return;
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

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *t = &timed_tasks[i];
        if (!t->task) continue;
        if (msTicks >= t->last_called + t->msinterval) {
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

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
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
    init_usart1(UART_BAUD);
    usart1_send((volatile char *)discobot_startup_banner);
}