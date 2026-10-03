#ifndef FSE_FROZEN_API_H
#define FSE_FROZEN_API_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum _ID {
    FWD=0, BWD=1, FWDLEFT=2, FWDRIGHT=3,
    BCKLEFT=4, BCKRIGHT=5, SPINRIGHT=6, SPINLEFT=7, STOPCAR=8
} _ID;
#define MAXIDSIZE 9
typedef enum Directions { FORWARD=0, BACKWARD=1, STOP=2 } Directions;
typedef enum ButtonState { ButtonIsReleased=0, ButtonIsPressed=1 } ButtonState;
typedef void (*motor_command_fn)(void);

extern volatile uint32_t msTicks;
extern volatile uint32_t b_i;
extern volatile ButtonState buttstate;
extern volatile _ID msgid;
extern motor_command_fn callme;
extern motor_command_fn flookup[9];
extern uint8_t GPIOA_output[16];
extern uint8_t GPIOD_output[16];
extern uint32_t GPIOA_IDR;
extern uint32_t SystemCoreClock;
extern uint32_t SysTick_reload;
extern uint32_t usart1_baud_used;
extern char usart1_tx_log[1024];
extern size_t usart1_tx_length;
extern const char *discobot_startup_banner;

#define START_BYTE 0x24
#define STOP_BYTE 0x2A
#define UART_BAUD 9600
#define BUTTON_DEBOUNCE_MS 250
#define SYSTICK_PERIOD_TICKS 168000
#define CIRC_BUFFER_MIN_SIZE 200
#define MAXNUMTASKS 25

void init_system(void);
void SystemInit(void);
void init_systick(void);
void init_button(void);
bool dispatch_uart_command(int command);
void main_loop_iteration(void);
void init_GPIO_A1A2A3A4_output(void);
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
int read_buttonc(int i);
void SysTick_Handler(void);
void checkbutton(void);
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);
void discobot_set_button_level(bool high);
void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);
int init_accelerometers(void);
void read_accelerometers(float b[3]);
void discobot_set_accelerometer_raw(int16_t x_mg, int16_t y_mg, int16_t z_mg);
void init_temperature_sensor(void);
float read_temperature_sensor(void);
void discobot_set_adc_raw(uint16_t raw);
void init_rng(void);
uint32_t get_random_number(void);
void discobot_set_rng(bool ready, uint32_t value);

typedef struct TimedTask {
    void (*task)(void);
    uint32_t msinterval;
    uint32_t last_called;
    uint32_t numcalls;
} TimedTask;
extern TimedTask timed_tasks[MAXNUMTASKS];
void add_timed_task(void (*myfunc)(void), float interval_sec);
void execute_tasks(void);
void printtimes(void);

typedef struct CircArray {
    char *buf;
    uint32_t size;
    bool enabled;
    uint32_t n_r;
    uint32_t n_w;
} CircArray;
void initCircArray(CircArray *arr, int size);
int buf_putbyte(CircArray *arr, char c);
char buf_getbyte(CircArray *arr);
bool buf_empty(CircArray *arr);
bool buf_full(CircArray *arr);
int buf_available(CircArray *arr);
bool buf_resize(CircArray *arr, int newSize);
bool buf_delete(CircArray *arr);
void buf_clear(CircArray *arr);

void init_usart1(int baud);
void USART1_IRQHandler(void);
void usart1_send(volatile char *s);
uint8_t usart1_read(void);
char usart1_readc(void);
uint32_t usart1_available(void);
void discobot_usart1_inject_rx(uint8_t value, bool rxne);

int func1(int R0);
int func2(int R0);

#endif