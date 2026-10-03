#include "6_generated_code.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef DISCOBOT_PI
#define DISCOBOT_PI 3.14159265358979323846f
#endif

volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = 0;
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
uint32_t GPIOA_IDR = 0u;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0u;
uint32_t usart1_baud_used = 0u;
char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";
TimedTask timed_tasks[MAXNUMTASKS] = {0};

static int16_t accel_raw_mg[3] = {0, 0, 0};
static uint16_t next_adc_raw = 0u;
static bool rng_ready = false;
static uint32_t rng_value = 0u;
static CircArray msg;
static ButtonState laststate = ButtonIsReleased;
static uint32_t t_prev = 0u;

struct HostUSART { volatile uint32_t SR; volatile uint32_t DR; };
static struct HostUSART host_usart1 = {0x40u, 0u};
#define USART1 (&host_usart1)
#define USART1_RXNE_BIT (1u << 5)
#define USART1_TXE_BIT 0x40u

#ifndef DISCOBOT_TARGET
static void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000u;
}
static int SysTick_Config(uint32_t ticks) {
    if (ticks != (SystemCoreClock / 1000u)) {
        return 1;
    }
    SysTick_reload = ticks;
    return 0;
}
static volatile uint32_t host_adc_eoc_flag = 1u;
static void ADC_SoftwareStartConv(void) {
    host_adc_eoc_flag = 1u;
}
static uint16_t ADC_GetConversionValue(void) {
    return next_adc_raw;
}
#define RESET 0u
#define ADC_FLAG_EOC host_adc_eoc_flag
#endif

void SystemInit(void) {
    SystemCoreClock = 168000000u;
}

void init_systick(void) {
    SystemCoreClockUpdate();
    uint32_t ticks = SystemCoreClock / 1000u;
    if (SysTick_Config(ticks) != 0) {
        while (1) {}
    }
    SysTick_reload = ticks;
    msTicks = 0u;
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0u;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
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

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
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

void move_forward(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void move_backward(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_forward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void move_forward_soft_right(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(STOP, 0.0f); }
void move_backward_soft_left(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_backward_soft_right(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(STOP, 0.0f); }
void move_spin_right(void) { set_left_motor_direc(FORWARD, 0.0f); set_right_motor_direc(BACKWARD, 0.0f); }
void move_spin_left(void) { set_left_motor_direc(BACKWARD, 0.0f); set_right_motor_direc(FORWARD, 0.0f); }
void stop(void) { set_left_motor_direc(STOP, 0.0f); set_right_motor_direc(STOP, 0.0f); }

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void discobot_set_button_level(bool high) {
    GPIOA_IDR = (GPIOA_IDR & ~1u) | (high ? 1u : 0u);
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250u) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0u;
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
    if (pitch == 0 || roll == 0) {
        return;
    }
    *roll = atan2f(acc_y, acc_z) * 180.0f / DISCOBOT_PI;
    *pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / DISCOBOT_PI;
}

int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float b[3]) {
    if (b == 0) {
        return;
    }
    b[0] = (float)accel_raw_mg[0] / 1000.0f;
    b[1] = (float)accel_raw_mg[1] / 1000.0f;
    b[2] = (float)accel_raw_mg[2] / 1000.0f;
}

void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg) {
    accel_raw_mg[0] = x_mg;
    accel_raw_mg[1] = y_mg;
    accel_raw_mg[2] = z_mg;
}

void init_temperature_sensor(void) {
}

float read_temperature_sensor(void) {
    ADC_SoftwareStartConv();
    while (ADC_FLAG_EOC == RESET) {}
    uint16_t raw = ADC_GetConversionValue();
    float temp = (float)raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void discobot_set_adc_raw(uint16_t raw) {
    next_adc_raw = raw;
}

void init_rng(void) {
}

uint32_t get_random_number(void) {
    while (!rng_ready) {}
    return rng_value;
}

void discobot_set_rng(bool ready, uint32_t value) {
    rng_ready = ready;
    rng_value = value;
}

void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (myfunc == 0) {
        return;
    }
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == 0) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(long)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }
}

void execute_tasks(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        TimedTask *p = &timed_tasks[i];
        if (p->task != 0 && msTicks >= (p->last_called + p->msinterval)) {
            p->task();
            p->last_called = msTicks;
            p->numcalls++;
        }
    }
}

void printtimes(void) {
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task != 0) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

void initCircArray(CircArray *arr, int size) {
    if (arr == 0) {
        return;
    }
    if (arr->enabled) {
        printf("CircArray already initialized\r\n");
        return;
    }
    if (size <= 0) {
        return;
    }
    arr->buf = (char *)calloc((size_t)size, 1);
    if (arr->buf == 0) {
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
    if (arr == 0 || arr->buf == 0 || arr->size == 0u || !arr->enabled) {
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
    if (arr == 0 || arr->buf == 0 || arr->size == 0u || !arr->enabled) {
        return 0;
    }
    if (buf_empty(arr)) {
        return 0;
    }
    char c = arr->buf[arr->n_r % arr->size];
    arr->n_r++;
    return c;
}

bool buf_empty(CircArray *arr) {
    if (arr == 0) {
        return true;
    }
    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr) {
    if (arr == 0 || arr->buf == 0 || arr->size == 0u) {
        return false;
    }
    return (!buf_empty(arr)) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_available(CircArray *arr) {
    if (arr == 0) {
        return 0;
    }
    return (int)(arr->n_w - arr->n_r);
}

bool buf_resize(CircArray *arr, int newSize) {
    if (arr == 0 || newSize <= 0) {
        return false;
    }
    char *p = (char *)realloc(arr->buf, (size_t)newSize);
    if (p == 0) {
        return false;
    }
    arr->buf = p;
    arr->size = (uint32_t)newSize;
    return true;
}

bool buf_delete(CircArray *arr) {
    if (arr == 0) {
        return false;
    }
    buf_clear(arr);
    free(arr->buf);
    arr->buf = 0;
    arr->size = 0u;
    arr->n_r = 0u;
    arr->n_w = 0u;
    arr->enabled = false;
    return true;
}

void buf_clear(CircArray *arr) {
    if (arr == 0 || arr->buf == 0 || arr->size == 0u) {
        return;
    }
    memset(arr->buf, 0, arr->size);
}

void init_usart1(int baud) {
    if (baud == 0) {
        printf("Warning: baud 0 defaults to 9600\r\n");
        baud = UART_BAUD;
    }
    usart1_baud_used = (uint32_t)baud;
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    USART1->SR = USART1_TXE_BIT;
    USART1->DR = 0u;
}

void USART1_IRQHandler(void) {
    if (USART1->SR & USART1_RXNE_BIT) {
        uint8_t c = (uint8_t)USART1->DR;
        USART1->SR &= ~USART1_RXNE_BIT;
        (void)buf_putbyte(&msg, (char)c);
    }
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne) {
    if (rxne) {
        USART1->DR = value;
        USART1->SR |= USART1_RXNE_BIT;
        USART1_IRQHandler();
    }
}

void usart1_send(volatile char *s) {
    if (s == 0) {
        return;
    }
    while (*s) {
        while (!(USART1->SR & USART1_TXE_BIT)) {}
        if (usart1_tx_length < sizeof(usart1_tx_log)) {
            usart1_tx_log[usart1_tx_length] = *s;
            usart1_tx_length++;
        } else {
            break;
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

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < MAXIDSIZE) {
        callme = flookup[command];
        msgid = (_ID)command;
        callme();
        return true;
    }
    return false;
}

void main_loop_iteration(void) {
    if (usart1_available() > 0u) {
        int command = (signed char)usart1_readc();
        if (command >= 0 && command < MAXIDSIZE) {
            (void)dispatch_uart_command(command);
        }
    }
    checkbutton();
    execute_tasks();
    if ((msTicks - t_prev) > 1000u) {
        t_prev = msTicks;
    }
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
    usart1_send("UART1 Initialized. @9600bps\r\n");
}

int func1(int R0) {
    return func2(R0);
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}