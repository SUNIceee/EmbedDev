/* include/disco_core.h */
#ifndef INCLUDE_DISCO_CORE_H_
#define INCLUDE_DISCO_CORE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file disco_core.h
 * @brief Global core definitions, macros, constants, and types for Discobot.
 */

/* ========================================================================== */
/* Target Compile Configuration                                               */
/* ========================================================================== */

/**
 * @brief DISCOBOT_TARGET compile option.
 * 0: Host / Mock target for unit testing
 * 1: Real Hardware target
 */
#ifndef DISCOBOT_TARGET
#define DISCOBOT_TARGET 0
#endif

/* ========================================================================== */
/* System Constants                                                           */
/* ========================================================================== */

#define DISCO_MOTOR_SPEED_MIN 0
#define DISCO_MOTOR_SPEED_MAX 255
#define DISCO_MOTOR_SAFE_SPEED 0

#define DISCO_RING_BUFFER_SIZE 128U
#define DISCO_SCHEDULER_MAX_TASKS 16U

/* ========================================================================== */
/* Core Data Types & Enums                                                    */
/* ========================================================================== */

/**
 * @brief System-wide status and return codes.
 */
typedef enum {
  DISCO_OK = 0,
  DISCO_ERROR = -1,
  DISCO_ERR_INVALID_ARG = -2,
  DISCO_ERR_FULL = -3,
  DISCO_ERR_EMPTY = -4,
  DISCO_ERR_UNINITIALIZED = -5,
  DISCO_ERR_SAFETY_VIOLATION = -6
} disco_status_t;

/**
 * @brief Task function prototype for the scheduler.
 */
typedef void (*disco_task_fn)(void);

/**
 * @brief Telemetry data structure for system state.
 */
typedef struct {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  uint32_t entropy;
  uint8_t motor_speed;
  bool is_safe;
} disco_telemetry_t;

#ifdef __cplusplus
}
#endif

#endif  /* INCLUDE_DISCO_CORE_H_ */
