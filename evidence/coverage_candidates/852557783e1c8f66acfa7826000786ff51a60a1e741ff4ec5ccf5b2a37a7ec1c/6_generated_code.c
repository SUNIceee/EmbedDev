/* Core system, motor control, task scheduling, and main logic for DiscoBot */
#include "6_generated_code.h"

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = STOPCAR;
motor_command_fn callme = stop;
motor_command_fn flookup[9] = {
    move_forward, move_backward, move_forward_soft_left,
    move_forward_soft_right, move_backward_soft_left,
    move_backward_soft_right, move_spin_right, move_spin_left, stop
};

uint8_t GPIOA_output[16] = {0};
uint8_t GPIOD_output[16] = {0};
uint32_t GPIOA_IDR = 0;
uint32_t SystemCoreClock = 168000000;
uint32_t SysTick_reload = 168000;
const char *discobot_startup_banner = "UART1 Initialized. @9600bps\r\n";

TimedTask timed_tasks[MAXNUMTASKS] = {0};

void SystemInit(void) {
    SystemCoreClock = 168000000;
}

void init_systick(void) {
    SysTick_reload = SystemCoreClock / 1000;
}

void init_button(void) {
    GPIOA_IDR &= ~1U;
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
    if (i > 3 || i < 0) return -1;
    return (int)((GPIOA_IDR >> i) & 1U);
}

void discobot_set_button_level(bool high) {
    if (high) {
        GPIOA_IDR |= 1U;
    } else {
        GPIOA_IDR &= ~1U;
    }
}

void SysTick_Handler(void) {
    msTicks++;
    if (read_buttonc(0) == 1) {
        if (b_i < UINT32_MAX) b_i++;
        if (b_i >= 250) {
            buttstate = ButtonIsPressed;
        }
    } else {
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
        }
        laststate = buttstate;
    }
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    if (!pitch || !roll) return;
    *roll = (float)(atan2(acc_y, acc_z) * 180.0 / M_PI);
    *pitch = (float)(atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI);
}

void init_LED_pins(void) {
    for (int i = 12; i <= 15; i++) {
        GPIOD_output[i] = 0;
    }
}

void LED_On(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 1;
    }
}

void LED_Off(int i) {
    if (i >= 0 && i < 4) {
        GPIOD_output[12 + i] = 0;
    }
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command <= 8) {
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
            printf("t%d=%d", i, (int)timed_tasks[i].numcalls);
        }
    }
}
