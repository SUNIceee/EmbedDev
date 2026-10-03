#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;

motor_command_fn callme = NULL;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0;
uint32_t usart1_baud_used = 0;

char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0;

const char *discobot_startup_banner =
    "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

/* ------------------------------------------------------------------------- */
/* Host-side hardware state                                                   */
/* ------------------------------------------------------------------------- */

static int16_t accelerometer_raw[3] = {0, 0, 0};
static uint16_t adc_raw_value = 0;
static bool rng_ready = false;
static uint32_t rng_value = 0;

static ButtonState last_button_state = ButtonIsReleased;
static uint32_t main_loop_t_previous = 0;

static CircArray msg;
static bool usart1_rxne = false;
static uint8_t usart1_dr = 0;

/* ------------------------------------------------------------------------- */
/* Forward declarations                                                       */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* System initialization                                                      */
/* ------------------------------------------------------------------------- */

void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}

void init_systick(void)
{
    /*
     * The host model always succeeds in configuring SysTick. The observable
     * reload value is identical to SystemCoreClock / 1000.
     */
    SystemCoreClock = 168000000u;
    SysTick_reload = SystemCoreClock / 1000u;
}

void init_LED_pins(void)
{
    GPIOD_output[12] = 0;
    GPIOD_output[13] = 0;
    GPIOD_output[14] = 0;
    GPIOD_output[15] = 0;
}

void init_button(void)
{
    GPIOA_IDR &= ~UINT32_C(1);
    b_i = 0;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

int init_accelerometers(void)
{
    return 0;
}

void init_rng(void)
{
    rng_ready = false;
}

void init_temperature_sensor(void)
{
    adc_raw_value = 0;
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

    msTicks = 0;
    main_loop_t_previous = 0;
    usart1_tx_length = 0;
    memset(usart1_tx_log, 0, sizeof(usart1_tx_log));

    usart1_send((volatile char *)discobot_startup_banner);
}

/* ------------------------------------------------------------------------- */
/* Motor control                                                              */
/* ------------------------------------------------------------------------- */

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

void set_right_motor_direc(int direc, float speed)
{
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

/* ------------------------------------------------------------------------- */
/* UART command dispatch                                                      */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* Button handling and SysTick                                                */
/* ------------------------------------------------------------------------- */

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }

    return (int)((GPIOA_IDR >> i) & UINT32_C(1));
}

void discobot_set_button_level(bool high)
{
    if (high) {
        GPIOA_IDR |= UINT32_C(1);
    } else {
        GPIOA_IDR &= ~UINT32_C(1);
    }
}

void SysTick_Handler(void)
{
    ++msTicks;

    if (read_buttonc(0) == 1) {
        ++b_i;

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
    if (last_button_state == buttstate) {
        return;
    }

    switch (buttstate) {
    case ButtonIsPressed:
        last_button_state = buttstate;
        break;

    case ButtonIsReleased:
        last_button_state = buttstate;
        break;

    default:
        last_button_state = buttstate;
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* LED handling                                                               */
/* ------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------- */
/* Accelerometer and orientation                                              */
/* ------------------------------------------------------------------------- */

void discobot_set_accelerometer_raw(int16_t x_mg,
                                    int16_t y_mg,
                                    int16_t z_mg)
{
    accelerometer_raw[0] = x_mg;
    accelerometer_raw[1] = y_mg;
    accelerometer_raw[2] = z_mg;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }

    b[0] = (float)accelerometer_raw[0] / 1000.0f;
    b[1] = (float)accelerometer_raw[1] / 1000.0f;
    b[2] = (float)accelerometer_raw[2] / 1000.0f;
}

void calc_pitch_roll(float acc_x,
                     float acc_y,
                     float acc_z,
                     float *pitch,
                     float *roll)
{
    const float radians_to_degrees = 180.0f / 3.14159265358979323846f;

    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * radians_to_degrees;
    }

    if (pitch != NULL) {
        *pitch = atan2f(-acc_x,
                       sqrtf((acc_y * acc_y) + (acc_z * acc_z))) *
                 radians_to_degrees;
    }
}

/* ------------------------------------------------------------------------- */
/* Temperature sensor                                                         */
/* ------------------------------------------------------------------------- */

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = raw & UINT16_C(0x0fff);
}

float read_temperature_sensor(void)
{
    float temperature;

    /*
     * This host model completes the conversion immediately. The arithmetic
     * order follows the specified ADC conversion sequence.
     */
    temperature = (float)adc_raw_value / 4095.0f;
    temperature *= 3.3f;
    temperature -= 0.760f;
    temperature /= 0.0025f;
    temperature += 25.0f;

    return temperature;
}

/* ------------------------------------------------------------------------- */
/* RNG                                                                        */
/* ------------------------------------------------------------------------- */

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
    /*
     * The public API has no timeout or error return. Host tests use
     * discobot_set_rng(true, value) for a completed conversion. Returning
     * zero for an unready host model keeps an accidental test call bounded.
     */
    if (!rng_ready) {
        return 0;
    }
#endif

    return rng_value;
}

/* ------------------------------------------------------------------------- */
/* Timed task scheduler                                                       */
/* ------------------------------------------------------------------------- */

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            long interval_ms = (long)(interval_sec * 1000.0f);

            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)interval_ms;
            timed_tasks[i].last_called = 0;
            timed_tasks[i].numcalls = 0;
            return;
        }
    }
}

void execute_tasks(void)
{
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        TimedTask *task = &timed_tasks[i];

        if (task->task != NULL &&
            msTicks >= task->last_called + task->msinterval) {
            task->task();
            task->last_called = msTicks;
            ++task->numcalls;
        }
    }
}

void printtimes(void)
{
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task != NULL) {
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Circular buffer                                                            */
/* ------------------------------------------------------------------------- */

void initCircArray(CircArray *arr, int size)
{
    if (arr == NULL || arr->enabled) {
        if (arr != NULL && arr->enabled) {
            fprintf(stderr, "CircArray is already enabled\n");
        }
        return;
    }

    if (size <= 0) {
        return;
    }

    arr->buf = (char *)calloc((size_t)size, sizeof(char));
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

bool buf_empty(CircArray *arr)
{
    if (arr == NULL) {
        return true;
    }

    return arr->n_r == arr->n_w;
}

bool buf_full(CircArray *arr)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0) {
        return false;
    }

    return !buf_empty(arr) &&
           ((arr->n_r % arr->size) == (arr->n_w % arr->size));
}

int buf_putbyte(CircArray *arr, char c)
{
    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0 || buf_full(arr)) {
        return 0;
    }

    arr->buf[arr->n_w % arr->size] = c;
    ++arr->n_w;

    return 1;
}

char buf_getbyte(CircArray *arr)
{
    char value;

    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0 || buf_empty(arr)) {
        return 0;
    }

    value = arr->buf[arr->n_r % arr->size];
    ++arr->n_r;

    return value;
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
    arr->enabled = true;

    return true;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0) {
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
    arr->size = 0;
    arr->n_r = 0;
    arr->n_w = 0;
    arr->enabled = false;

    return true;
}

/* ------------------------------------------------------------------------- */
/* USART1                                                                     */
/* ------------------------------------------------------------------------- */

void init_usart1(int baud)
{
    if (msg.enabled) {
        buf_delete(&msg);
    }

    memset(&msg, 0, sizeof(msg));
    initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);

    if (baud == 0) {
        baud = UART_BAUD;
    }

    usart1_baud_used = (uint32_t)baud;
    usart1_rxne = false;
    usart1_dr = 0;
}

void USART1_IRQHandler(void)
{
    if (!usart1_rxne) {
        return;
    }

    (void)buf_putbyte(&msg, (char)usart1_dr);
    usart1_rxne = false;
}

void discobot_usart1_inject_rx(uint8_t value, bool rxne)
{
    usart1_dr = value;
    usart1_rxne = rxne;

    if (rxne) {
        USART1_IRQHandler();
    }
}

void usart1_send(volatile char *s)
{
    if (s == NULL) {
        return;
    }

    while (*s != '\0') {
        if (usart1_tx_length < sizeof(usart1_tx_log) - 1) {
            usart1_tx_log[usart1_tx_length++] = *s;
            usart1_tx_log[usart1_tx_length] = '\0';
        }

        ++s;
    }
}

uint8_t usart1_read(void)
{
    return (uint8_t)buf_getbyte(&msg);
}

char usart1_readc(void)
{
    return (char)buf_getbyte(&msg);
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

/* ------------------------------------------------------------------------- */
/* Main-loop single-step host equivalent                                      */
/* ------------------------------------------------------------------------- */

void main_loop_iteration(void)
{
    if ((uint32_t)(msTicks - main_loop_t_previous) > 1000u) {
        main_loop_t_previous = msTicks;
    }

    if (usart1_available() > 0u) {
        char command = usart1_readc();

        if ((unsigned char)command <= STOPCAR) {
            (void)dispatch_uart_command((int)command);
        }
    }

    checkbutton();
    execute_tasks();
}

/* ------------------------------------------------------------------------- */
/* ARM assembly-compatible functions                                         */
/* ------------------------------------------------------------------------- */

int func2(int R0)
{
    uint32_t value = (uint32_t)R0;
    value += UINT32_C(1);

    return (int)(int32_t)value;
}

int func1(int R0)
{
    return func2(R0);
}
