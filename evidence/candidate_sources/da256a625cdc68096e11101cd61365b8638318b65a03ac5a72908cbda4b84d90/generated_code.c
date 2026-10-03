/* SOURCE */
#include "generated_code.h"

/* Global State Definitions */
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 0;
volatile uint32_t msTicks = 0;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;

volatile int b_i = 0;
volatile int buttstate = ButtonIsReleased;
volatile int laststate = ButtonIsReleased;

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

TimedTask timed_tasks[MAXNUMTASKS] = {{0}};
CircArray msg = {NULL, 0, false, 0, 0};

char usart1_tx_log[1024] = {0};
uint32_t usart1_tx_length = 0;

/* Host Model Hardware Internal States */
static int16_t host_acc_raw_x = 0;
static int16_t host_acc_raw_y = 0;
static int16_t host_acc_raw_z = 1000;
static uint16_t host_adc_raw = 943;
static bool host_rng_ready = true;
static uint32_t host_rng_value = 0x12345678;
static char host_usart1_dr = 0;
static uint32_t t_prev = 0;

/* Motor Control Implementation */
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

void init_GPIO_A1A2A3A4_output(void) {
    GPIOA_output[1] = 0;
    GPIOA_output[2] = 0;
    GPIOA_output[3] = 0;
    GPIOA_output[4] = 0;
}

/* System, Clock, Button, LED, Math */
void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 168000000;
}

uint32_t SysTick_Config(uint32_t ticks) {
    SysTick_reload = ticks;
    return 0;
}

int init_systick(void) {
    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000) != 0) {
        while (1) {}
    }
    return 0;
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

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
}

int read_buttonc(int i) {
    if (i < 0 || i > 3) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void checkbutton(void) {
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
    if (roll) {
        *roll = (float)(atan2((double)acc_y, (double)acc_z) * 180.0 / (double)PI);
    }
    if (pitch) {
        double xy_sq = (double)acc_y * (double)acc_y + (double)acc_z * (double)acc_z;
        *pitch = (float)(atan2(-(double)acc_x, sqrt(xy_sq)) * 180.0 / (double)PI);
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

/* Sensors and RNG */
int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float acc[3]) {
    if (!acc) return;
    acc[0] = (float)host_acc_raw_x / 1000.0f;
    acc[1] = (float)host_acc_raw_y / 1000.0f;
    acc[2] = (float)host_acc_raw_z / 1000.0f;
}

void init_temperature_sensor(void) {
}

float read_temperature_sensor(void) {
    /* Step-by-step floating calculation according to spec */
    float temp = (float)host_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void init_rng(void) {
}

uint32_t get_random_number(void) {
    int timeout = 10000;
    while (!host_rng_ready && --timeout > 0) {}
    return host_rng_value;
}

/* Timed Task Scheduler */
void add_timed_task(void (*myfunc)(void), float interval_sec) {
    if (!myfunc) return;
    for (int i = 0; i < MAXNUMTASKS; i++) {
        if (timed_tasks[i].task == NULL) {
            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)(interval_sec * 1000.0f);
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
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
            printf("t{%d}=%u\n", i, (unsigned int)timed_tasks[i].numcalls);
        }
    }
}

/* Ring Buffer (CircArray) */
void initCircArray(CircArray *arr, int size) {
    if (!arr) return;
    if (arr->enabled) {
        printf("CircArray error: already enabled\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, sizeof(char));
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->enabled || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->enabled || buf_empty(arr)) {
        return 0;
    }
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
    return !buf_empty(arr) && ((arr->n_r % arr->size) == (arr->n_w % arr->size));
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

void buf_clear(CircArray *arr) {
    if (arr && arr->buf && arr->size > 0) {
        memset(arr->buf, 0, arr->size);
    }
}

bool buf_delete(CircArray *arr) {
    if (!arr) return false;
    buf_clear(arr);
    if (arr->buf) {
        free(arr->buf);
        arr->buf = NULL;
    }
    arr->size = 0;
    arr->enabled = false;
    arr->n_r = 0;
    arr->n_w = 0;
    return true;
}

/* USART1 */
void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
        printf("Warning: Baud rate 0 specified, defaulting to 9600\n");
    }
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
    buf_putbyte(&msg, host_usart1_dr);
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
    if (buf_empty(&msg)) {
        return -1;
    }
    return buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

/* Assembly C implementations */
int func2(int R0) {
    return (int)((uint32_t)R0 + 1u);
}

int func1(int R0) {
    return func2(R0);
}

/* Host Verification & Simulation Helpers */
void discobot_set_button_level(int level) {
    if (level) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
    host_acc_raw_x = x;
    host_acc_raw_y = y;
    host_acc_raw_z = z;
}

void discobot_set_adc_raw(uint16_t raw) {
    host_adc_raw = raw;
}

void discobot_set_rng(bool ready, uint32_t val) {
    host_rng_ready = ready;
    host_rng_value = val;
}

void discobot_usart1_inject_rx(char c) {
    host_usart1_dr = c;
    USART1_IRQHandler();
}

void dispatch_uart_command(void) {
    char c = usart1_readc();
    if (c >= 0 && c <= 8) {
        motor_command_fn callme = flookup[(int)c];
        if (callme) {
            callme();
        }
    }
}

void main_loop_iteration(void) {
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
    dispatch_uart_command();
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
    usart1_send("UART1 Initialized. @9600bps\r\n");
}
