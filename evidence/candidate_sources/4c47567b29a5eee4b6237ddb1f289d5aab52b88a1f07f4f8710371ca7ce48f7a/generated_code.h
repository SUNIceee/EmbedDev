#ifndef GENERATED_CODE_H
#define GENERATED_CODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define START_BYTE             0x24
#define STOP_BYTE              0x2A
#define UART_BAUD              9600
#define BUTTON_DEBOUNCE_MS     250
#define SYSTICK_PERIOD_TICKS   168000
#define CIRC_BUFFER_MIN_SIZE   200
#define MAXNUMTASKS            25
#define MAXIDSIZE              9

#define ButtonIsPressed        1
#define ButtonIsReleased       0

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

typedef struct CircArray {
    char *buf;
    uint32_t size;
    bool enabled;
    uint32_t n_r;
    uint32_t n_w;
} CircArray;

typedef void (*motor_command_fn)(void);

typedef struct {
    void (*task)(void);
    long msinterval;
    uint32_t last_called;
    uint32_t numcalls;
} timed_task_t;

/* Host-observable state */
extern uint8_t GPIOA_output[5];   /* index 1..4 are motor pins */
extern uint8_t GPIOD_output[16];  /* index 12..15 are LED pins */
extern uint32_t GPIOA_IDR;        /* bit0 is button */
extern uint32_t SysTick_reload;
extern volatile uint32_t msTicks;
extern CircArray msg;

extern char *usart1_tx_log;
extern size_t usart1_tx_length;
extern size_t usart1_tx_capacity;
extern uint32_t usart1_baud;
extern volatile bool usart1_rxne_pending;
extern uint8_t usart1_rx_data;

extern motor_command_fn flookup[9];
extern timed_task_t timed_tasks[MAXNUMTASKS];

/* Button state */
extern int buttstate;
extern int laststate;
extern uint32_t b_i;
extern uint32_t t_prev;

/* Sensor injection state */
extern int32_t discobot_accel_raw[3];
extern uint16_t discobot_adc_raw;
extern uint32_t discobot_rng_value;
extern bool discobot_rng_ready;

/* Car */
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

/* LED */
void init_LED_pins(void);
void LED_On(int i);
void LED_Off(int i);

/* Button */
int read_buttonc(int i);
void SysTick_Handler(void);
void checkbutton(void);
void init_button(void);

/* Accelerometer */
int init_accelerometers(void);
void read_accelerometers(float *acc[3]);
void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll);

/* Temperature */
void init_temperature_sensor(void);
float read_temperature_sensor(void);

/* RNG */
void init_rng(void);
uint32_t get_random_number(void);

/* USART */
void init_usart1(int baud);
void USART1_IRQHandler(void);
void usart1_send(volatile char *s);
uint8_t usart1_read(void);
char usart1_readc(void);
uint32_t usart1_available(void);

/* CircArray */
void initCircArray(CircArray *arr, int size);
int buf_putbyte(CircArray *arr, char c);
char buf_getbyte(CircArray *arr);
bool buf_empty(CircArray *arr);
bool buf_full(CircArray *arr);
int buf_available(CircArray *arr);
bool buf_resize(CircArray *arr, int newSize);
bool buf_delete(CircArray *arr);
void buf_clear(CircArray *arr);

/* Task scheduler */
void add_timed_task(void (*myfunc)(void), float interval_sec);
void execute_tasks(void);
void printtimes(void);

/* System / main-loop host model */
void SystemInit(void);
void init_systick(void);
void init_system(void);
void dispatch_uart_command(void);
void main_loop_iteration(void);

/* Test injection helpers */
void discobot_set_button_level(int level);
void discobot_set_accelerometer_raw(int x, int y, int z);
void discobot_set_adc_raw(uint16_t raw);
void discobot_set_rng(uint32_t value, bool ready);
void discobot_usart1_inject_rx(uint8_t byte);

/* ARM assembly equivalents */
int func1(int R0);
int func2(int R0);

#ifdef __cplusplus
}
#endif

#endif /* GENERATED_CODE_H */
