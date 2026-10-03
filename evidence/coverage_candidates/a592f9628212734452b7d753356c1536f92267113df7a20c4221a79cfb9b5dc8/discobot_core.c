/* Implementation of DiscoBot core motor control, task scheduling, command dispatching, math, and assembly helpers */
#include "6_generated_code.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile uint32_t msTicks = 0;
volatile uint32_t b_i = 0;
volatile ButtonState buttstate = ButtonIsReleased;
volatile _ID msgid = FWD;
motor_command_fn callme = NULL;
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

TimedTask timed_tasks[MAXNUMTASKS] = {0};

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
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void move_backward(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_forward_soft_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void move_forward_soft_right(void) {
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void move_backward_soft_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_backward_soft_right(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void move_spin_right(void) {
    GPIOA_output[1] = 1; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 1;
}

void move_spin_left(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 1;
    GPIOA_output[3] = 1; GPIOA_output[4] = 0;
}

void stop(void) {
    GPIOA_output[1] = 0; GPIOA_output[2] = 0;
    GPIOA_output[3] = 0; GPIOA_output[4] = 0;
}

void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll) {
    double pi_val = 3.14159265358979323846;
    if (roll) {
        *roll = (float)(atan2((double)acc_y, (double)acc_z) * 180.0 / pi_val);
    }
    if (pitch) {
        double denom = sqrt((double)acc_y * (double)acc_y + (double)acc_z * (double)acc_z);
        *pitch = (float)(atan2(-(double)acc_x, denom) * 180.0 / pi_val);
    }
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
            printf("t%d=%u", i, (unsigned int)timed_tasks[i].numcalls);
        }
    }
}

bool dispatch_uart_command(int command) {
    if (command >= 0 && command < 9) {
        msgid = (_ID)command;
        callme = flookup[command];
        if (callme) {
            callme();
        }
        return true;
    }
    return false;
}

int func2(int R0) {
    return (int)((uint32_t)R0 + 1U);
}

int func1(int R0) {
    return func2(R0);
}
