#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = stop;
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

TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

static CircArray msg = {NULL, 0, false, 0, 0};
static int16_t host_acc_x = 0;
static int16_t host_acc_y = 0;
static int16_t host_acc_z = 0;
static uint16_t host_adc_raw = 943;
static bool host_rng_ready = true;
static uint32_t host_rng_value = 0;

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

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void SysTick_Handler(void) {
    msTicks++;
    int b = read_buttonc(0);
    if (b == 1) {
        b_i++;
        if (b_i >= 250) {
            buttstate = ButtonIsPressed;
        }
    } else if (b == 0) {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void) {
    static ButtonState laststate = ButtonIsReleased;
    if (buttstate != laststate) {
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
    if (roll != NULL) {
        *roll = (float)(atan2((double)acc_y, (double)acc_z) * 180.0 / M_PI);
    }
    if (pitch != NULL) {
        double denominator = sqrt((double)acc_y * (double)acc_y + (double)acc_z * (double)acc_z);
        *pitch = (float)(atan2(-(double)acc_x, denominator) * 180.0 / M_PI);
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
    for (int i = 12; i <= 15; i++) {
        GPIOD_output[i] = 0;
    }
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

int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b != NULL) {
        b[0] = (float)host_acc_x / 1000.0f;
        b[1] = (float)host_acc_y / 1000.0f;
        b[2] = (float)host_acc_z / 1000.0f;
    }
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    host_acc_x = x_mg;
    host_acc_y = y_mg;
    host_acc_z = z_mg;
}

void init_temperature_sensor(void) {
}

float read_temperature_sensor(void) {
    float temp = (float)host_adc_raw / 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    host_adc_raw = raw;
}

void init_rng(void) {
}

uint32_t get_random_number(void) {
    while (!host_rng_ready) {
    }
    return host_rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    host_rng_ready = ready;
    host_rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == NULL) return;
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            break;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != NULL) {
            if (msTicks >= timed_tasks[i].last_called + timed_tasks[i].msinterval) {
                timed_tasks[i].task();
                timed_tasks[i].last_called = msTicks;
                timed_tasks[i].numcalls++;
            }
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

void initCircArray(CircArray *arr, int size) {
    if (arr == NULL) return;
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

bool buf_empty(CircArray *arr) {
    if (arr == NULL) return true;
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == NULL || arr->size == 0) return false;
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (arr == NULL) return 0;
    return (int)(arr->n_w - arr->n_r);
}

int buf_putbyte(CircArray *arr, char c) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL) return 0;
    if (buf_full(arr)) return 0;
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (arr == NULL || !arr->enabled || arr->buf == NULL) return 0;
    if (buf_empty(arr)) return 0;
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == NULL || newSize <= 0) return false;
    char *p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p == NULL) return false;
    arr->buf = p;
    arr->size = (uint32_t)newSize;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == NULL || arr->buf == NULL || arr->size == 0) return;
    memset(arr->buf, 0, arr->size);
}

bool buf_delete(CircArray *arr) {
    if (arr == NULL) return false;
    buf_clear(arr);
    if (arr->buf != NULL) {
        free(arr->buf);
        arr->buf = NULL;
    }
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;
    return true;
}

void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
        printf("Warning: Baud rate 0 requested, default 9600 used\n");
    }
    usart1_baud_used = (uint32_t)baud;
    if (msg.enabled) {
        buf_delete(&msg);
    }
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        buf_putbyte(&msg, (char)value);
    }
}

void usart1_send(volatile char *s) {
    if (s == NULL) return;
    while (*s != '\0') {
        char c = *s++;
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = c;
            usart1_tx_log[usart1_tx_length] = '\0';
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

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < 9) {
        callme = flookup[command];
        msgid = (_ID)command;
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0) {
        char c = usart1_readc();
        if (c >= 0 && c <= 8) {
            dispatch_uart_command((int)c);
        }
    }
    checkbutton();
    execute_tasks();
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

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}