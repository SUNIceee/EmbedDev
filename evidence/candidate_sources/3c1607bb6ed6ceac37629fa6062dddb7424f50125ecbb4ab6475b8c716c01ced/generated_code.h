#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define START_BYTE            0x24
#define STOP_BYTE             0x2A
#define UART_BAUD             9600
#define BUTTON_DEBOUNCE_MS    250
#define SYSTICK_PERIOD_TICKS  168000
#define CIRC_BUFFER_MIN_SIZE  200
#define MAXNUMTASKS           25
#define MAXIDSIZE             9

typedef enum {
    FORWARD = 0,
    BACKWARD = 1,
    STOP = 2
} Directions;

typedef enum _ID {
    FWD = 0,
    BWD = 1,
    FWDLEFT = 2,
    FWDRIGHT = 3,
    BCKLEFT = 4,
    BCKRIGHT = 5,
    SPINRIGHT = 6,
    SPINLEFT = 7,
    STOPCAR = 8
} _ID;

typedef void (*task_fn)(void);
typedef void (*motor_command_fn)(void);

typedef struct CircArray {
    char *buf;
    uint32_t size;
    bool enabled;
    uint32_t n_r;
    uint32_t n_w;
} CircArray;

typedef struct {
    task_fn task;
    long msinterval;
    uint32_t last_called;
    uint32_t numcalls;
} timed_task_t;

typedef enum {
    ButtonIsReleased = 0,
    ButtonIsPressed = 1
} ButtonState;

/* System state */
extern uint32_t SystemCoreClock;
extern volatile uint32_t msTicks;
extern volatile uint32_t SysTick_reload;
extern volatile int systick_config_error;

/* Host observable GPIO state */
extern volatile uint8_t GPIOA_output[16];
extern volatile uint8_t GPIOD_output[16];
extern volatile uint32_t GPIOA_IDR;

/* Button debouncing state */
extern volatile ButtonState buttstate;
extern volatile ButtonState laststate;

/* Sensor injection/state */
extern volatile int16_t discobot_accel_raw[3];
extern volatile uint16_t discobot_adc_raw;
extern volatile uint8_t discobot_adc_eoc;
extern volatile uint8_t discobot_rng_ready;
extern volatile uint32_t discobot_rng_value;

/* USART state */
extern CircArray msg;
extern char *usart1_tx_log;
extern uint32_t usart1_tx_length;
extern uint32_t usart1_tx_capacity;
extern volatile uint32_t usart1_baud;
extern volatile uint8_t usart1_enabled;
extern volatile uint8_t usart1_rxne;
extern volatile uint8_t usart1_rx_dr;
extern volatile uint8_t usart1_tx_txe;

/* Scheduler and command table */
extern timed_task_t timed_tasks[MAXNUMTASKS];
extern motor_command_fn flookup[MAXIDSIZE];

/* System initialization and main-loop equivalent */
void SystemInit(void);
void SystemCoreClockUpdate(void);
int SysTick_Config(uint32_t ticks);
void init_systick(void);
void init_system(void);
void main_loop_iteration(void);
void SysTick_Handler(void);

/* Motor control */
void set_left_motor_direc(int direc, float speed);
void set_right_motor_direc(int direc, float speed);
void move_forward(void);
void move_backward(void);
void move_forward_soft_left(void);
void move_forward_soft_right(void);
void move_backward_soft_left(void);
void move_backward_soft_right(void);
void move_spin_right(void);
void move_spin_left(void);
void stop(void);
void init_GPIO_A1A2A3A4_output(void);

/* Button and LED */
void init_button(void);
int read_buttonc(int i);
void checkbutton(void);
void discobot_set_button_level(uint8_t level);
void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);

/* Sensors */
int init_accelerometers(void);
void read_accelerometers(float *acc[3]);
void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z);
void init_temperature_sensor(void);
float read_temperature_sensor(void);
void discobot_set_adc_raw(uint16_t raw);
void init_rng(void);
uint32_t get_random_number(void);
void discobot_set_rng(uint8_t ready, uint32_t value);

/* Scheduler */
void add_timed_task(void (*myfunc)(void), float interval_sec);
void execute_tasks(void);
void printtimes(void);

/* USART */
void init_usart1(int baud);
void USART1_IRQHandler(void);
void usart1_send(volatile char *s);
uint8_t usart1_read(void);
char usart1_readc(void);
uint32_t usart1_available(void);
void discobot_usart1_inject_rx(uint8_t data);

/* Ring buffer */
void initCircArray(CircArray *arr, int size);
int buf_putbyte(CircArray *arr, char c);
char buf_getbyte(CircArray *arr);
bool buf_empty(CircArray *arr);
bool buf_full(CircArray *arr);
uint32_t buf_available(CircArray *arr);
bool buf_resize(CircArray *arr, int newSize);
bool buf_delete(CircArray *arr);
void buf_clear(CircArray *arr);

/* Command dispatch */
void dispatch_uart_command(void);

/* Attitude calculation and ARM-equivalent helpers */
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);
int func1(int R0);
int func2(int R0);

#ifdef __cplusplus
}
#endif

#endif
