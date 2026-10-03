/* src/scheduler.c */
#include "disco_core.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file scheduler.c
 * @brief Implementation of the task scheduler based on SysTick-like execution.
 */

#define MAX_TASKS 8U

typedef struct {
  void (*task_fn)(void);
  uint32_t period;
  uint32_t last_run;
} Task;

static Task s_task_list[MAX_TASKS];
static uint8_t s_task_count = 0U;
static uint32_t s_current_tick = 0U;

/**
 * @brief Initializes or resets the scheduler state.
 */
void scheduler_init(void) {
  s_task_count = 0U;
  s_current_tick = 0U;
  for (size_t i = 0U; i < MAX_TASKS; ++i) {
    s_task_list[i].task_fn = NULL;
    s_task_list[i].period = 0U;
    s_task_list[i].last_run = 0U;
  }
}

/**
 * @brief Registers a task to be executed at a specific interval.
 * 
 * @param fn Pointer to the function to be executed.
 * @param period Execution interval in ticks.
 * @return true if registration succeeded, false otherwise.
 */
bool register_task(void (*fn)(void), uint32_t period) {
  if (fn == NULL || s_task_count >= MAX_TASKS) {
    return false;
  }

  s_task_list[s_task_count].task_fn = fn;
  s_task_list[s_task_count].period = (period == 0U) ? 1U : period;
  s_task_list[s_task_count].last_run = s_current_tick;
  s_task_count++;

  return true;
}

/**
 * @brief Process the scheduler tick.
 * This should be called by the system timer interrupt or main loop.
 */
void tick(void) {
  s_current_tick++;

  for (uint8_t i = 0U; i < s_task_count; ++i) {
    /* Check if the task is due for execution (handles uint32 overflow safely) */
    if ((s_current_tick - s_task_list[i].last_run) >= s_task_list[i].period) {
      if (s_task_list[i].task_fn != NULL) {
        s_task_list[i].task_fn();
      }
      s_task_list[i].last_run = s_current_tick;
    }
  }
}
