#include "6_generated_code.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* ------------------------------------------------------------------------- */
/* Global host-observable state                                             */
/* ------------------------------------------------------------------------- */

volatile uint32_t msTicks = 0u;
volatile uint32_t b_i = 0u;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0u;

uint32_t SystemCoreClock = 168000000u;
uint32_t SysTick_reload = 0u;
uint32_t usart1_baud_used = 0u;

char usart1_tx_log[1024] = {0};
size_t usart1_tx_length = 0u;

const char *discobot_startup_banner =
    "UART1 Initialized. @9600bps\r\n";

/* ------------------------------------------------------------------------- */
/* Simulated peripheral state                                               */
/* ------------------------------------------------------------------------- */

static bool systick_config_failed = false;

static int16_t accelerometer_raw_x = 0;
static int16_t accelerometer_raw_y = 0;
static int16_t accelerometer_raw_z = 0;

static uint16_t adc_raw_value = 0u;
static bool adc_eoc = true;

static bool rng_ready = false;
static uint32_t rng_value = 0u;

static uint8_t usart1_dr = 0u;
static bool usart1_rxne = false;
static bool usart1_txe = true;

static CircArray msg;

/* Used by the simulated main-loop timer. */
static uint32_t main_loop_previous_tick = 0u;

/* Used by checkbutton(). */
static ButtonState last_button_state = ButtonIsReleased;

/* ------------------------------------------------------------------------- */
/* Motor control                                                             */
/* ------------------------------------------------------------------------- */

void set_left_motor_direc(int direc, float speed)
{
    (void)speed;

    switch (direc) {
        case FORWARD:
            GPIOA_output[1] = 1u;
            GPIOA_output[2] = 0u;
            break;

        case BACKWARD:
            GPIOA_output[1] = 0u;
            GPIOA_output[2] = 1u;
            break;

        case STOP:
        default:
            GPIOA_output[1] = 0u;
            GPIOA_output[2] = 0u;
            break;
    }
}

void set_right_motor_direc(int direc, float speed)
{
    (void)speed;

    switch (direc) {
        case FORWARD:
            GPIOA_output[3] = 1u;
            GPIOA_output[4] = 0u;
            break;

        case BACKWARD:
            GPIOA_output[3] = 0u;
            GPIOA_output[4] = 1u;
            break;

        case STOP:
        default:
            GPIOA_output[3] = 0u;
            GPIOA_output[4] = 0u;
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
    set_left_motor_direc(STOP, 1.0f);
    set_right_motor_direc(FORWARD, 1.0f);
}

void move_forward_soft_right(void)
{
    set_left_motor_direc(FORWARD, 1.0f);
    set_right_motor_direc(STOP, 1.0f);
}

void move_backward_soft_left(void)
{
    set_left_motor_direc(STOP, 1.0f);
    set_right_motor_direc(BACKWARD, 1.0f);
}

void move_backward_soft_right(void)
{
    set_left_motor_direc(BACKWARD, 1.0f);
    set_right_motor_direc(STOP, 1.0f);
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
    set_left_motor_direc(STOP, 1.0f);
    set_right_motor_direc(STOP, 1.0f);
}

void init_GPIO_A1A2A3A4_output(void)
{
    GPIOA_output[1] = 0u;
    GPIOA_output[2] = 0u;
    GPIOA_output[3] = 0u;
    GPIOA_output[4] = 0u;
}

/* ------------------------------------------------------------------------- */
/* Command dispatch                                                          */
/* ------------------------------------------------------------------------- */

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

motor_command_fn callme = move_forward;

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
/* System initialization                                                     */
/* ------------------------------------------------------------------------- */

void SystemInit(void)
{
    SystemCoreClock = 168000000u;
}

void init_systick(void)
{
    /*
     * Equivalent to SystemCoreClockUpdate() followed by:
     *
     *     SysTick_Config(SystemCoreClock / 1000);
     *
     * The host model records the reload value.  The simulated configuration
     * succeeds unless an implementation-specific failure is introduced.
     */
    SystemCoreClock = 168000000u;
    SysTick_reload = SystemCoreClock / 1000u;
    systick_config_failed = (SysTick_reload == 0u);

    if (systick_config_failed) {
        /*
         * The target implementation enters its documented error path here.
         * The host model preserves the failure state without hanging the
         * test process indefinitely.
         */
        return;
    }
}

void init_LED_pins(void)
{
    GPIOD_output[12] = 0u;
    GPIOD_output[13] = 0u;
    GPIOD_output[14] = 0u;
    GPIOD_output[15] = 0u;
}

void init_button(void)
{
    GPIOA_IDR &= ~UINT32_C(1);
    b_i = 0u;
    buttstate = ButtonIsReleased;
    last_button_state = ButtonIsReleased;
}

void init_system(void)
{
    SystemInit();
    init_systick();

    if (systick_config_failed) {
        return;
    }

    init_LED_pins();
    init_button();
    (void)init_accelerometers();
    init_rng();
    init_temperature_sensor();
    init_GPIO_A1A2A3A4_output();
    init_usart1(UART_BAUD);

    usart1_send((volatile char *)discobot_startup_banner);
}

/* ------------------------------------------------------------------------- */
/* Button and SysTick                                                        */
/* ------------------------------------------------------------------------- */

int read_buttonc(int i)
{
    if (i < 0 || i > 3) {
        return -1;
    }

    return (int)((GPIOA_IDR >> (unsigned)i) & 1u);
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
    int button_level;

    msTicks++;

    button_level = read_buttonc(0);

    if (button_level == 1) {
        b_i++;

        if (b_i >= BUTTON_DEBOUNCE_MS) {
            buttstate = ButtonIsPressed;
        }
    } else {
        b_i = 0u;
        buttstate = ButtonIsReleased;
    }
}

void checkbutton(void)
{
    if (buttstate != last_button_state) {
        last_button_state = buttstate;

        switch (buttstate) {
            case ButtonIsPressed:
                break;

            case ButtonIsReleased:
                break;

            default:
                break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* Main-loop processing                                                      */
/* ------------------------------------------------------------------------- */

void main_loop_iteration(void)
{
    if ((uint32_t)(msTicks - main_loop_previous_tick) > 1000u) {
        main_loop_previous_tick = msTicks;
        /* One-second idle timer: no action is currently required. */
    }

    if (usart1_available() > 0u) {
        char command = usart1_readc();
        (void)dispatch_uart_command((int)command);
    }

    checkbutton();
    execute_tasks();
}

/* ------------------------------------------------------------------------- */
/* LED control                                                               */
/* ------------------------------------------------------------------------- */

void LED_On(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }

    GPIOD_output[12 + i] = 1u;
}

void LED_Off(int i)
{
    if (i < 0 || i >= 4) {
        return;
    }

    GPIOD_output[12 + i] = 0u;
}

/* ------------------------------------------------------------------------- */
/* Accelerometer                                                             */
/* ------------------------------------------------------------------------- */

int init_accelerometers(void)
{
    /*
     * Host equivalent of:
     * TM_LIS302DL_LIS3DSH_Init(
     *     TM_LIS3DSH_Sensitivity_2G,
     *     TM_LIS3DSH_Filter_50Hz);
     */
    return 0;
}

void discobot_set_accelerometer_raw(int16_t x_mg,
                                    int16_t y_mg,
                                    int16_t z_mg)
{
    accelerometer_raw_x = x_mg;
    accelerometer_raw_y = y_mg;
    accelerometer_raw_z = z_mg;
}

void read_accelerometers(float b[3])
{
    if (b == NULL) {
        return;
    }

    b[0] = (float)accelerometer_raw_x / 1000.0f;
    b[1] = (float)accelerometer_raw_y / 1000.0f;
    b[2] = (float)accelerometer_raw_z / 1000.0f;
}

void calc_pitch_roll(float acc_x,
                     float acc_y,
                     float acc_z,
                     float *pitch,
                     float *roll)
{
    const float degrees_per_radian = 180.0f / 3.14159265358979323846f;

    if (roll != NULL) {
        *roll = atan2f(acc_y, acc_z) * degrees_per_radian;
    }

    if (pitch != NULL) {
        *pitch = atan2f(-acc_x,
                       sqrtf((acc_y * acc_y) + (acc_z * acc_z)))
                 * degrees_per_radian;
    }
}

/* ------------------------------------------------------------------------- */
/* Temperature sensor                                                        */
/* ------------------------------------------------------------------------- */

void init_temperature_sensor(void)
{
    /*
     * Host model: ADC configuration is represented by an immediately
     * completed conversion state.  The injected raw value is used by the
     * subsequent read operation.
     */
    adc_eoc = true;
}

void discobot_set_adc_raw(uint16_t raw)
{
    adc_raw_value = raw & 0x0FFFu;
    adc_eoc = true;
}

float read_temperature_sensor(void)
{
    float temperature;
    uint16_t raw;

    /* 1. ADC_SoftwareStartConv() */
    adc_eoc = true;

    /* 2. Wait for ADC_FLAG_EOC. */
    while (!adc_eoc) {
        /* Target hardware waits here. */
    }

    /* 3. ADC_GetConversionValue() */
    raw = adc_raw_value;

    /* 4 through 8: preserve the specified operation order. */
    temperature = (float)raw / 4095.0f;
    temperature *= 3.3f;
    temperature -= 0.760f;
    temperature /= 0.0025f;
    temperature += 25.0f;

    return temperature;
}

/* ------------------------------------------------------------------------- */
/* RNG                                                                       */
/* ------------------------------------------------------------------------- */

void init_rng(void)
{
    rng_ready = false;
    rng_value = 0u;
}

void discobot_set_rng(bool ready, uint32_t value)
{
    rng_ready = ready;
    rng_value = value;
}

uint32_t get_random_number(void)
{
    while (!rng_ready) {
        /*
         * This is the documented blocking DRDY wait.  Tests that call this
         * function must inject a ready RNG state.
         */
    }

    return rng_value;
}

/* ------------------------------------------------------------------------- */
/* Timed task scheduler                                                      */
/* ------------------------------------------------------------------------- */

TimedTask timed_tasks[MAXNUMTASKS] = {{0}};

void add_timed_task(void (*myfunc)(void), float interval_sec)
{
    int i;

    for (i = 0; i < MAXNUMTASKS; ++i) {
        if (timed_tasks[i].task == NULL) {
            long interval_ms = (long)(interval_sec * 1000.0f);

            timed_tasks[i].task = myfunc;
            timed_tasks[i].msinterval = (uint32_t)interval_ms;
            timed_tasks[i].last_called = 0u;
            timed_tasks[i].numcalls = 0u;
            return;
        }
    }

    /* All slots are occupied: documented silent failure. */
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
            task->numcalls++;
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
    char *new_buffer;

    if (arr == NULL || size <= 0) {
        return;
    }

    if (arr->enabled) {
        fprintf(stderr, "CircArray already enabled\n");
        return;
    }

    new_buffer = (char *)calloc((size_t)size, sizeof(char));

    if (new_buffer == NULL) {
        return;
    }

    arr->buf = new_buffer;
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
    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0u) {
        return false;
    }

    return !buf_empty(arr) &&
           ((arr->n_r % arr->size) == (arr->n_w % arr->size));
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
    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0u) {
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
    char value;

    if (arr == NULL || !arr->enabled || arr->buf == NULL ||
        arr->size == 0u || buf_empty(arr)) {
        return (char)0;
    }

    value = arr->buf[arr->n_r % arr->size];
    arr->n_r++;

    return value;
}

void buf_clear(CircArray *arr)
{
    if (arr == NULL || arr->buf == NULL || arr->size == 0u) {
        return;
    }

    memset(arr->buf, 0, (size_t)arr->size);
}

bool buf_resize(CircArray *arr, int newSize)
{
    char *new_buffer;

    if (arr == NULL || newSize <= 0) {
        return false;
    }

    new_buffer = (char *)realloc(arr->buf, (size_t)newSize);

    if (new_buffer == NULL) {
        return false;
    }

    arr->buf = new_buffer;
    arr->size = (uint32_t)newSize;

    return true;
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
    arr->n_r = 0u;
    arr->n_w = 0u;
    arr->enabled = false;

    return true;
}

/* ------------------------------------------------------------------------- */
/* USART1                                                                    */
/* ------------------------------------------------------------------------- */

void init_usart1(int baud)
{
    if (baud == 0) {
        baud = UART_BAUD;
        fprintf(stderr, "USART1 baud was zero; using 9600\n");
    }

    usart1_baud_used = (uint32_t)baud;

    /*
     * The hardware configuration corresponds to PB6/PB7 alternate-function
     * GPIO, USART1 8N1, TX/RX enabled, RXNE interrupt enabled, and NVIC
     * priority 0/0.
     */
    usart1_txe = true;
    usart1_rxne = false;

    if (!msg.enabled) {
        initCircArray(&msg, CIRC_BUFFER_MIN_SIZE);
    }
}

void USART1_IRQHandler(void)
{
    uint8_t received;

    if (!usart1_rxne) {
        return;
    }

    received = usart1_dr;
    usart1_rxne = false;

    (void)buf_putbyte(&msg, (char)received);
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
        while (!usart1_txe) {
            /*
             * Target behavior: wait indefinitely for USART SR bit 0x40.
             */
        }

        if (usart1_tx_length < sizeof(usart1_tx_log) - 1u) {
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
    uint8_t value = (uint8_t)buf_getbyte(&msg);

    /*
     * Preserve the specified host-visible behavior in which 0xFF is
     * interpreted as -1 by the command-dispatch path.
     */
    if (value == 0xFFu) {
        return (char)-1;
    }

    return (char)value;
}

uint32_t usart1_available(void)
{
    return (uint32_t)buf_available(&msg);
}

/* ------------------------------------------------------------------------- */
/* ARM assembly semantic equivalents                                         */
/* ------------------------------------------------------------------------- */

int func2(int R0)
{
    /*
     * Implement R0 + 1 with defined wrap behavior rather than relying on
     * signed-integer overflow.
     */
    if (R0 == INT_MAX) {
        return INT_MIN;
    }

    return R0 + 1;
}

int func1(int R0)
{
    return func2(R0);
}
