#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define START_BYTE 0x24u
#define STOP_BYTE 0x2Au
#define UART_BAUD 9600
#define BUTTON_DEBOUNCE_MS 250u
#define SYSTICK_PERIOD_TICKS 168000u
#define CIRC_BUFFER_MIN_SIZE 200
#define MAXNUMTASKS 25
#define MAXIDSIZE 9
#define DISCOBOT_USART_TX_LOG_CAPACITY 4096u
#define PI 3.14159265358979323846f

#define USART_SR_RXNE 0x20u
#define USART_SR_TXE  0x40u

typedef enum { FORWARD = 0, BACKWARD = 1, STOP = 2 } Directions;

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

typedef enum ButtonState {
    ButtonIsReleased = 0,
    ButtonIsPressed = 1
} ButtonState;

typedef void (*motor_command_fn)(void);

typedef struct CircArray {
    char *buf;
    uint32_t size;
    bool enabled;
    uint32_t n_r;
    uint32_t n_w;
} CircArray;

typedef struct timed_task {
    void (*task)(void);
    long msinterval;
    uint32_t last_called;
    uint32_t numcalls;
} timed_task;

typedef struct USART_TypeDef {
    volatile uint32_t SR;
    volatile uint32_t DR;
} USART_TypeDef;

extern uint32_t SystemCoreClock;
extern uint32_t SysTick_reload;
extern uint32_t msTicks;
extern uint8_t GPIOA_output[16];
extern uint8_t GPIOD_output[16];
extern uint32_t GPIOA_IDR;

extern int gpioa_clock_enabled;
extern int gpiob_clock_enabled;
extern int gpiod_clock_enabled;
extern int rng_clock_enabled;
extern int adc1_clock_enabled;
extern int usart1_enabled;
extern int usart1_baud;
extern int usart1_rxne_interrupt_enabled;
extern int nvic_usart1_preempt_priority;
extern int nvic_usart1_sub_priority;
extern int systick_failed;
extern int systick_config_result;

extern int button_pin_initialized;
extern int led_pins_initialized;
extern int motor_pins_initialized;
extern int accelerometer_initialized;
extern int rng_initialized;
extern int temperature_sensor_initialized;

extern uint32_t b_i;
extern ButtonState buttstate;
extern ButtonState laststate;

extern CircArray msg;
extern timed_task timed_tasks[MAXNUMTASKS];
extern motor_command_fn flookup[MAXIDSIZE];

extern USART_TypeDef USART1_instance;
extern USART_TypeDef *USART1;

extern char usart1_tx_log[DISCOBOT_USART_TX_LOG_CAPACITY];
extern uint32_t usart1_tx_length;
extern int usart1_tx_overflow;

extern int16_t discobot_accel_raw_x;
extern int16_t discobot_accel_raw_y;
extern int16_t discobot_accel_raw_z;
extern uint16_t discobot_adc_raw;
extern bool discobot_adc_eoc;
extern bool discobot_rng_ready;
extern uint32_t discobot_rng_value;
extern bool discobot_rng_blocked;

extern int init_sequence_log[16];
extern uint32_t init_sequence_count;

void SystemInit(void);
void SystemCoreClockUpdate(void);
int SysTick_Config(uint32_t ticks);

void init_systick(void);
void init_system(void);
void main_loop_iteration(void);
void dispatch_uart_command(void);

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

int read_buttonc(int i);
void SysTick_Handler(void);
void checkbutton(void);
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);

void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);

int init_accelerometers(void);
void read_accelerometers(float *acc[3]);
void discobot_set_accelerometer_raw(int16_t x, int16_t y, int16_t z);

void init_temperature_sensor(void);
float read_temperature_sensor(void);
void discobot_set_adc_raw(uint16_t raw);

void init_rng(void);
uint32_t get_random_number(void);
void discobot_set_rng(bool ready, uint32_t value);

void add_timed_task(void (*myfunc)(void), float interval_sec);
void execute_tasks(void);
void printtimes(void);

void init_usart1(int baud);
void USART1_IRQHandler(void);
void usart1_send(volatile char *s);
uint8_t usart1_read(void);
char usart1_readc(void);
uint32_t usart1_available(void);
void discobot_usart1_inject_rx(uint8_t c);
void discobot_clear_usart1_tx_log(void);

void initCircArray(CircArray *arr, int size);
int buf_putbyte(CircArray *arr, char c);
char buf_getbyte(CircArray *arr);
bool buf_empty(CircArray *arr);
bool buf_full(CircArray *arr);
int buf_available(CircArray *arr);
bool buf_resize(CircArray *arr, int newSize);
bool buf_delete(CircArray *arr);
void buf_clear(CircArray *arr);

void discobot_set_button_level(int high);
void discobot_reset_host_state(void);

int func1(int R0);
int func2(int R0);

#endif
