/* SOURCE */
#include "generated_code.h"

/* Global Hardware State Representations */
uint8_t GPIOA_output[5] = {0, 0, 0, 0, 0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
volatile uint32_t msTicks = 0;
uint32_t SysTick_reload = 0;
char usart1_tx_log[1024] = {0};
uint32_t usart1_tx_length = 0;

volatile ButtonState buttstate = ButtonIsReleased;
volatile ButtonState laststate = ButtonIsReleased;
volatile uint32_t b_i = 0;

CircArray msg = {NULL, 0, false, 0, 0};
TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

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

/* Internal Host Mock State */
static int16_t mock_acc_x = 0;
static int16_t mock_acc_y = 0;
static int16_t mock_acc_z = 0;
static uint16_t mock_adc_raw = 943;
static bool mock_rng_ready = true;
static uint32_t mock_rng_value = 0x12345678;

/* Motor Control */
void set_left_motor_direc(int direc, float speed) {
    (void)speed;
    if (direc == FORWARD) {
        GPIOA_output[1] = 1;
        GPIOA_output[2] = 0;
    } else if (direc == BACKWARD) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 1;
    } else if (direc == STOP) {
        GPIOA_output[1] = 0;
        GPIOA_output[2] = 0;
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
    } else if (direc == STOP) {
        GPIOA_output[3] = 0;
        GPIOA_output[4] = 0;
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

/* Button & Debounce */
int read_buttonc(int i) {
    if (i > 3 || i < 0) {
        return -1;
    }
    return (int)((GPIOA_IDR >> i) & 1u);
}

void init_button(void) {
    GPIOA_IDR &= ~1u;
    b_i = 0;
    buttstate = ButtonIsReleased;
    laststate = ButtonIsReleased;
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

/* Attitude calculation */
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    float r = atan2f(acc_y, acc_z) * 180.0f / (float)M_PI;
    float p = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / (float)M_PI;
    if (roll) {
        *roll = r;
    }
    if (pitch) {
        *pitch = p;
    }
}

/* LED Control */
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

/* Sensors */
int init_accelerometers(void) {
    return 0;
}

void read_accelerometers(float acc[3]) {
    if (acc) {
        acc[0] = (float)mock_acc_x / 1000.0f;
        acc[1] = (float)mock_acc_y / 1000.0f;
        acc[2] = (float)mock_acc_z / 1000.0f;
    }
}

void init_temperature_sensor(void) {
    /* Temperature sensor initialization sequence wrapper */
}

float read_temperature_sensor(void) {
    float temp = (float)mock_adc_raw;
    temp /= 4095.0f;
    temp *= 3.3f;
    temp -= 0.760f;
    temp /= 0.0025f;
    temp += 25.0f;
    return temp;
}

void init_rng(void) {
    /* RNG clock enabling sequence wrapper */
}

uint32_t get_random_number(void) {
    while (!mock_rng_ready) {
        /* Wait simulation */
        break;
    }
    return mock_rng_value;
}

/* Scheduler */
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
            printf("t%d=%u\n", i, (unsigned int)timed_tasks[i].numcalls);
        }
    }
}

/* CircArray Implementation */
void initCircArray(CircArray *arr, int size) {
    if (!arr) return;
    if (arr->enabled) {
        printf("Error: CircArray already enabled.\n");
        return;
    }
    arr->buf = (char *)calloc((size_t)size, sizeof(char));
    arr->size = (uint32_t)size;
    arr->enabled = true;
    arr->n_r = 0;
    arr->n_w = 0;
}

int buf_putbyte(CircArray *arr, char c) {
    if (!arr || !arr->enabled || !arr->buf || buf_full(arr)) {
        return 0;
    }
    arr->buf[arr->n_w % arr->size] = c;
    arr->n_w++;
    return 1;
}

char buf_getbyte(CircArray *arr) {
    if (!arr || !arr->enabled || !arr->buf || buf_empty(arr)) {
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
    char *new_buf = (char *)realloc(arr->buf, (size_t)newSize);
    if (!new_buf) return false;
    arr->buf = new_buf;
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
    if (!arr || !arr->buf || arr->size == 0) return;
    memset(arr->buf, 0, arr->size);
}

/* USART Functions */
void init_usart1(int baud) {
    if (baud == 0) {
        baud = 9600;
    }
    initCircArray(&msg, 200);
}

void USART1_IRQHandler(void) {
    /* RXNE Interrupt simulated handling */
}

void usart1_send(volatile char *s) {
    if (!s) return;
    while (*s != '\0') {
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
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void) {
    return (uint32_t)buf_available(&msg);
}

/* System Initialization & Systick */
void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        b_i++;
        if (b_i >= 250) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0;
        buttstate = ButtonIsReleased;
    }
}

int SystemInit(void) {
    return 0;
}

int init_systick(void) {
    SysTick_reload = 168000;
    return 0;
}

void init_system(void) {
    SystemInit();
    if (init_systick() != 0) {
        while (1) {}
    }
    init_LED_pins();
    init_button();
    init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(9600);
    usart1_send((volatile char *)"UART1 Initialized. @9600bps\r\n");
}

/* Command Dispatcher & Main Loop */
void dispatch_uart_command(void) {
    if (usart1_available() > 0) {
        char c = usart1_readc();
        if (c >= 0 && c <= 8) {
            flookup[(int)c]();
        }
    }
}

void main_loop_iteration(void) {
    static uint32_t t_prev = 0;
    if (msTicks - t_prev > 1000) {
        t_prev = msTicks;
    }
    dispatch_uart_command();
    checkbutton();
    execute_tasks();
}

/* Assembly Functions Emulation */
int func2(int R0) {
    return R0 + 1;
}

int func1(int R0) {
    return func2(R0);
}

/* Host Injectors */
void discobot_set_button_level(int level) {
    if (level) {
        GPIOA_IDR |= 1u;
    } else {
        GPIOA_IDR &= ~1u;
    }
}

void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z) {
    mock_acc_x = x;
    mock_acc_y = y;
    mock_acc_z = z;
}

void discobot_set_adc_raw(uint16_t raw) {
    mock_adc_raw = raw;
}

void discobot_set_rng(bool ready, uint32_t value) {
    mock_rng_ready = ready;
    mock_rng_value = value;
}

void discobot_usart1_inject_rx(char c) {
    buf_putbyte(&msg, c);
}
