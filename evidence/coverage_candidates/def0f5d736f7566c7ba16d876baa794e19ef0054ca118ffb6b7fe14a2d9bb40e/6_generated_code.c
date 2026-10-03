#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Global Variables Definition */
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
uint32_t SysTick_reload = 168000;
uint32_t usart1_baud_used = 9600;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* Internal Host Simulation State Variables */
static CircArray msg;
static uint8_t simulated_usart1_dr = 0;
static int16_t raw_acc_x = 0;
static int16_t raw_acc_y = 0;
static int16_t raw_acc_z = 0;
static uint16_t raw_adc_val = 943;
static bool rng_ready = true;
static uint32_t rng_val = 0;

/* ============================================================================
 * 2. System Initialization API
 * ============================================================================ */

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SysTick_reload = SystemCoreClock / 1000;
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
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
    init_usart1(9600);
    usart1_send((volatile char *)discobot_startup_banner);
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
    static uint32_t t_prev = 0;

    if (usart1_available() > 0) {
        char c = usart1_readc();
        dispatch_uart_command((int)c);
    }

    checkbutton();
    execute_tasks();

    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
}

/* ============================================================================
 * 3. Motor Control (Car)
 * ============================================================================ */

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[1] = 1;
        GPIOA_output[2] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 1;
    } else {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
    }
}

void set_right_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[3] = 1;
        GPIOA_output[4] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 1;
    } else {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
    }
}

void move_forward(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_backward(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_forward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void move_forward_soft_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_backward_soft_left(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_backward_soft_right(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

void move_spin_right(void) {
    set_left_motor_direc(FORWARD, 0.0f);
    set_right_motor_direc(BACKWARD, 0.0f);
}

void move_spin_left(void) {
    set_left_motor_direc(BACKWARD, 0.0f);
    set_right_motor_direc(FORWARD, 0.0f);
}

void stop(void) {
    set_left_motor_direc(STOP, 0.0f);
    set_right_motor_direc(STOP, 0.0f);
}

/* ============================================================================
 * 4. Command Dispatch and Button
 * ============================================================================ */

int read_buttonc(int i) {
    if (i > 3) return -1;
    return (int)((GPIOA_IDR >> i) & 1u);
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
    static ButtonState laststate = ButtonIsReleased;
    if (laststate != buttstate) {
        switch (buttstate) {
            case ButtonIsPressed:
                break;
            case ButtonIsReleased:
                break;
        }
        laststate = buttstate;
    }
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (roll) {
        *roll = (float)(atan2(acc_y, acc_z) * 180.0 / M_PI);
    }
    if (pitch) {
        *pitch = (float)(atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI);
    }
}

/* ============================================================================
 * 5. LED
 * ============================================================================ */

void init_LED_pins(void) {
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void LED_On(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i <= 3) {
        GPIOD_output[12 + i] = 0;
    }
}

/* ============================================================================
 * 6. Accelerometers
 * ============================================================================ */

int init_accelerometers(void) {
    raw_acc_x = 0;
    raw_acc_y = 0;
    raw_acc_z = 0;
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b) {
        b[0] = raw_acc_x / 1000.0f;
        b[1] = raw_acc_y / 1000.0f;
        b[2] = raw_acc_z / 1000.0f;
    }
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    raw_acc_x = x_mg;
    raw_acc_y = y_mg;
    raw_acc_z = z_mg;
}

/* ============================================================================
 * 7. Temperature Sensor
 * ============================================================================ */

void init_temperature_sensor(void) {
}

float read_temperature_sensor(void) {
    float temp = (float)raw_adc_val / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    raw_adc_val = raw;
}

/* ============================================================================
 * 8. Hardware RNG
 * ============================================================================ */

void init_rng(void) {
}

uint32_t get_random_number(void) {
    while (!rng_ready) {
    }
    return rng_val;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_val = value;
}

/* ============================================================================
 * 9. Timed Task Scheduler
 * ============================================================================ */

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (!myfunc) return;
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls = 0;
            break;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL && msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
            timed_tasks[i].task();
            timed_tasks[i].last_called = msTicks;
            timed_tasks[i].numcalls++;
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%u", i, (unsigned int)timed_tasks[i].numcalls);
        }
    }
}

/* ============================================================================
 * 10. CircArray Ring Buffer
 * ============================================================================ */

void initCircArray(CircArray *arr, int size) {
    if (!arr) return;
    if (arr->enabled) {
        printf("Error: CircArray already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->enabled || !arr->buf) return 0;
    if (buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->enabled || !arr->buf || buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) {
    if (!arr) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (!arr || arr->size == 0) return false;
    return (!buf_empty(arr)) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (!arr) return 0;
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (!arr || newSize <= 0) return false;
    char *newbuf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!newbuf) return false;
    arr->buf = newbuf;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    buf_clear(arr);
    if (arr->buf) {
        free(arr->buf);
        arr->buf = NULL;
    }
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (!arr || !arr->buf) return;
    memset(arr->buf, 0, arr->size);
}

/* ============================================================================
 * 11. USART
 * ============================================================================ */

void init_usart1(int baud) {
    if (baud == 0) {
        usart1_baud_used = 9600;
    } else {
        usart1_baud_used = (uint32_t)baud;
    }

    if (msg.enabled) {
        buf_delete(&msg);
    }
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
    buf_putbyte(&msg, (char)simulated_usart1_dr);
}

void usart1_send(volatile char *s) {
    if (!s) return;
    while (*s) {
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }
        s++;
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
    simulated_usart1_dr = value;
    if (rxne) {
        USART1_IRQHandler();
    }
}

/* ============================================================================
 * 12. ARM Assembly Fallback Functions
 * ============================================================================ */

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}
