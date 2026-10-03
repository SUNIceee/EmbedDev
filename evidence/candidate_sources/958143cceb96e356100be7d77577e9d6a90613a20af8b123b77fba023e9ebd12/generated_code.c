#include "generated_code.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------
 * System and host-observable state
 * ------------------------------------------------------------------------- */
uint32_t SystemCoreClock = 168000000;
volatile uint32_t msTicks = 0;
volatile uint32_t SysTick_reload = 0;
volatile int systick_config_error = 0;

volatile uint8_t GPIOA_output[16];
volatile uint8_t GPIOD_output[16];
volatile uint32_t GPIOA_IDR = 0;

volatile ButtonState buttstate = ButtonIsReleased;
volatile ButtonState laststate = ButtonIsReleased;

volatile int16_t discobot_accel_raw[3];
volatile uint16_t discobot_adc_raw = 0;
volatile uint8_t discobot_adc_eoc = 0;
volatile uint8_t discobot_rng_ready = 0;
volatile uint32_t discobot_rng_value = 0;

CircArray msg;
char *usart1_tx_log = NULL;
uint32_t usart1_tx_length = 0;
uint32_t usart1_tx_capacity = 0;
volatile uint32_t usart1_baud = 0;
volatile uint8_t usart1_enabled = 0;
volatile uint8_t usart1_rxne = 0;
volatile uint8_t usart1_rx_dr = 0;
volatile uint8_t usart1_tx_txe = 0;

timed_task_t timed_tasks[MAXNUMTASKS];

static volatile uint32_t b_i = 0;

motor_command_fn flookup[MAXIDSIZE] = {
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

/* -------------------------------------------------------------------------
 * System clock and SysTick
 * ------------------------------------------------------------------------- */
void SystemInit(void) {
    SystemCoreClock = 168000000;
    systick_config_error = 0;
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000;
}

int SysTick_Config(uint32_t ticks) {
    if (ticks == 0 || ticks > 0x00FFFFFFu) {
        systick_config_error = 1;
        return 1;
    }

    SysTick_reload = ticks;
    systick_config_error = 0;
    return 0;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    SysTick_reload = SystemCoreClock / 1000;

    if (SysTick_Config(SysTick_reload) != 0) {
        systick_config_error = 1;
        /* On target this would sit in while(1). Host keeps the error state. */
    } else {
        systick_config_error = 0;
    }
}

/* -------------------------------------------------------------------------
 * Motor control
 * ------------------------------------------------------------------------- */
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
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
        break;
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
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
        break;
    default:
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
        break;
    }
}

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void move_forward(void)             { set_left_motor_direc(FORWARD, 0.0f);  set_right_motor_direc(FORWARD, 0.0f); }
void move_backward(void)            { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_forward_soft_left(void)   { set_left_motor_direc(STOP, 0.0f);     set_right_motor_direc(FORWARD, 0.0f); }
void move_forward_soft_right(void)  { set_left_motor_direc(FORWARD, 0.0f);  set_right_motor_direc(STOP, 0.0f); }
void move_backward_soft_left(void)  { set_left_motor_direc(STOP, 0.0f);     set_right_motor_direc(BACKWARD, 0.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(STOP, 0.0f); }
void move_spin_right(void)          { set_left_motor_direc(FORWARD, 0.0f);  set_right_motor_direc(BACKWARD, 0.0f); }
void move_spin_left(void)           { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void stop(void)                     { set_left_motor_direc(STOP, 0.0f);     set_right_motor_direc(STOP, 0.0f); }

/* -------------------------------------------------------------------------
 * Button and LED
 * ------------------------------------------------------------------------- */
void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

void discobot_set_button_level(uint8_t level) {
    if (level) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
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
    if (laststate == buttstate) {
        return;
    }

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

void init_LED_pins(void) {
    for (int i = 0; i < 4; i++) {
        GPIOD_output[12 + i] = 0;
    }
}

void LED_On(int i) {
    if (i < 0 || i > 3) {
        return;
    }

    GPIOD_output[12 + i] = 1;
}

void LED_Off(int i) {
    if (i < 0 || i > 3) {
        return;
    }

    GPIOD_output[12 + i] = 0;
}

/* -------------------------------------------------------------------------
 * Accelerometers
 * ------------------------------------------------------------------------- */
int init_accelerometers(void) {
    discobot_accel_raw[0] = 0;
    discobot_accel_raw[1] = 0;
    discobot_accel_raw[2] = 0;
    return 0;
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
    discobot_accel_raw[0] = x;
    discobot_accel_raw[1] = y;
    discobot_accel_raw[2] = z;
}

void read_accelerometers(float *acc[3]) {
    if (acc == NULL) {
        return;
    }

    if (acc[0] != NULL) {
        *acc[0] = discobot_accel_raw[0] / 1000.0f;
    }

    if (acc[1] != NULL) {
        *acc[1] = discobot_accel_raw[1] / 1000.0f;
    }

    if (acc[2] != NULL) {
        *acc[2] = discobot_accel_raw[2] / 1000.0f;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    const float PI = 3.14159265358979323846f;

    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * 180.0f / PI;
    }

    if (pitch != NULL) {
        *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI;
    }
}

/* -------------------------------------------------------------------------
 * Temperature sensor
 * ------------------------------------------------------------------------- */
void init_temperature_sensor(void) {
    discobot_adc_raw = 0;
    discobot_adc_eoc = 0;
}

void discobot_set_adc_raw(uint16_t raw) {
    discobot_adc_raw = raw;
    discobot_adc_eoc = 1;
}

float read_temperature_sensor(void) {
    /* 1) ADC_SoftwareStartConv */
    /* 2) Wait for EOC */
    while (discobot_adc_eoc == 0) {
        /* Host must inject a raw ADC value via discobot_set_adc_raw. */
    }

    /* 3) Read conversion value and apply documented conversion steps. */
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
 * ------------------------------------------------------------------------- */
void init_rng(void) {
    discobot_rng_ready = 0;
    discobot_rng_value = 0;
}

void discobot_set_rng(uint8_t ready, uint32_t value) {
    discobot_rng_ready = ready;
    discobot_rng_value = value;
}

uint32_t get_random_number(void) {
    while (discobot_rng_ready == 0) {
        /* Host must set ready/value via discobot_set_rng. */
    }

    return discobot_rng_value;
}

/* -------------------------------------------------------------------------
 * Timed task scheduler
 * ------------------------------------------------------------------------- */
void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) {
        return;
    }

    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        timed_task_t *t = &timed_tasks[i];

        if (t->task == NULL) {
            continue;
        }

        uint32_t next = (uint32_t)((int64_t)t->last_called + t->msinterval);

        if (msTicks >= next) {
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

/* -------------------------------------------------------------------------
 * Ring buffer
 * ------------------------------------------------------------------------- */
void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) {
        return;
    }

    if (arr->enabled) {
        printf("CircArray already enabled\n");
        return;
    }

    if (size <= 0) {
        return;
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

bool buf_empty(CircArray *arr) {
    if (arr == NULL) {
        return true;
    }

    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0 || arr->buf == NULL) {
        return false;
    }

    return !buf_empty(arr) &&
           (arr->n_r % arr->size) == (arr->n_w % arr->size);
}

uint32_t buf_available(CircArray *arr) {
    if (arr == NULL) {
        return 0;
    }

    return arr->n_w - arr->n_r;
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || arr->size == 0 || arr->buf == NULL) {
        return 0;
    }

    if (buf_full(arr)) {
        return 0;
    }

    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || arr->size == 0 || arr->buf == NULL) {
        return 0;
    }

    if (buf_empty(arr)) {
        return 0;
    }

    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
        return;
    }

    memset(arr->buf, 0, arr->size);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) {
        return false;
    }

    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (new_buf == NULL) {
        return false;
    }

    arr->buf = new_buf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) {
        return false;
    }

    buf_clear(arr);

    if (arr->buf != NULL) {
        free(arr->buf);
    }

    arr->buf = NULL;
    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;

    return true;
}

/* -------------------------------------------------------------------------
 * USART
 * ------------------------------------------------------------------------- */
void init_usart1(int baud) {
    if (baud == 0) {
        fprintf(stderr, "Warning: baud rate is 0, using 9600\n");
        baud = 9600;
    }

    usart1_baud = (uint32_t)baud;
    usart1_enabled = 1;
    usart1_tx_txe = 1;
    usart1_rxne = 0;
    usart1_rx_dr = 0;

    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
}

void usart1_send(volatile char *s) {
    if (s == NULL) {
        return;
    }

    while (*s) {
        while (!usart1_tx_txe) {
            /* Target behaviour has no timeout. */
        }

        char c = *s;

        if (usart1_tx_length + 1 >= usart1_tx_capacity) {
            uint32_t new_cap = (usart1_tx_capacity == 0) ? 64 : usart1_tx_capacity * 2;
            char *new_log = (char *)realloc(usart1_tx_log, new_cap);
            if (new_log == NULL) {
                return;
            }

            usart1_tx_log = new_log;
            usart1_tx_capacity = new_cap;
        }

        usart1_tx_log[usart1_tx_length++] = c;
        usart1_tx_log[usart1_tx_length] = '\0';

        s++;
    }
}

void USART1_IRQHandler(void) {
    if (!usart1_enabled) {
        return;
    }

    if (usart1_rxne) {
        char c = (char)usart1_rx_dr;
        buf_putbyte(&msg, c);
        usart1_rxne = 0;
    }
}

void discobot_usart1_inject_rx(uint8_t data) {
    usart1_rx_dr = data;
    usart1_rxne = 1;
    USART1_IRQHandler();
}

uint8_t usart1_read(void) {
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void) {
    if (buf_empty(&msg)) {
        return -1;
    }

    return (signed char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return buf_available(&msg);
}

/* -------------------------------------------------------------------------
 * Main-loop equivalent and system init
 * ------------------------------------------------------------------------- */
void dispatch_uart_command(void) {
    char c = usart1_readc();

    if (c >= 0 && c <= 8) {
        flookup[(int)c]();
    }
}

void main_loop_iteration(void) {
    static uint32_t last_second_tick = 0;

    if (msTicks - last_second_tick > 1000) {
        last_second_tick = msTicks;
    }

    dispatch_uart_command();
}

void init_system(void) {
    SystemInit();
    init_systick();

    if (systick_config_error) {
        /* Target would enter while(1) on SysTick_Config error. */
        return;
    }

    init_LED_pins();
    init_button();
    init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send("UART1 Initialized. @9600bps\r\n");
}

/* -------------------------------------------------------------------------
 * ARM assembly equivalents for host verification
 * ------------------------------------------------------------------------- */
int func2(int R0) {
    uint32_t u = (uint32_t)R0 + 1u;
    return (int)u;
}

int func1(int R0) {
    return func2(R0);
}
