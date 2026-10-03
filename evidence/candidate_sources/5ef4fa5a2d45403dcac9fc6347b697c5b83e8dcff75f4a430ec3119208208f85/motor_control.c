/* src/motor_control.c */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "disco_core.h"
#include "disco_hw_interface.h"

/**
 * @file motor_control.c
 * @brief Motor control module implementation.
 *
 * Implements the motor control state machine, safety checks, and hardware abstraction layer (HAL) separation.
 * Follows the Google C Style coding conventions.
 */

/* ------------------------------------------------------------------------- */
/* Private macros and internal constants. */
/* ------------------------------------------------------------------------- */

#define MOTOR_MAX_SPEED (100)
#define MOTOR_MIN_SPEED (-100)
#define MOTOR_SAFE_SPEED_LIMIT (100)

/* ------------------------------------------------------------------------- */
/* Static private function declarations. */
/* ------------------------------------------------------------------------- */

/**
 * @brief Check safety constraints for the requested motor speed and state.
 * @param ctrl Motor controller handle.
 * @param speed Requested motor speed (-100 to 100).
 * @return true Safe state and valid parameters; false indicates a safety violation.
 */
static bool is_safe_state(const MotorCtrl *ctrl, int8_t speed);

/**
 * @brief Determine the direction-pin state from the motor speed.
 * @param speed Motor speed.
 * @return PinState High means forward; low means reverse.
 */
static PinState speed_to_direction(int8_t speed);

/**
 * @brief Convert signed speed to PWM duty cycle (0 to 100%).
 * @param speed Motor speed (-100 to 100).
 * @return uint8_t Duty-cycle percentage.
 */
static uint8_t speed_to_pwm_duty(int8_t speed);

/* ------------------------------------------------------------------------- */
/* Private function implementations. */
/* ------------------------------------------------------------------------- */

static bool is_safe_state(const MotorCtrl *ctrl, int8_t speed) {
  if (ctrl == NULL) {
    return false;
  }

  // 1. Check whether the controller is in a fault state.
  if (ctrl->state == MOTOR_STATE_FAULT) {
    return false;
  }

  // 2. Check the hardware interface pointer.
  if (ctrl->hw_if == NULL || ctrl->hw_if->set_gpio == NULL ||
      ctrl->hw_if->set_pwm == NULL) {
    return false;
  }

  // 3. Check speed bounds (-100 to 100).
  if (speed < MOTOR_MIN_SPEED || speed > MOTOR_MAX_SPEED) {
    return false;
  }

  // 4. Guard against reversal or abrupt direction changes; high-speed reversal may be checked here.
  if (ctrl->is_emergency_stopped) {
    return false;
  }

  return true;
}

static PinState speed_to_direction(int8_t speed) {
  return (speed >= 0) ? PIN_STATE_HIGH : PIN_STATE_LOW;
}

static uint8_t speed_to_pwm_duty(int8_t speed) {
  if (speed < 0) {
    return (uint8_t)(-speed);
  }
  return (uint8_t)speed;
}

/* ------------------------------------------------------------------------- */
/* Public function implementations. */
/* ------------------------------------------------------------------------- */

bool motor_ctrl_init(MotorCtrl *ctrl, const HardwareInterface *hw_if) {
  if (ctrl == NULL || hw_if == NULL) {
    return false;
  }

  // Validate the required HardwareInterface function pointers.
  if (hw_if->set_gpio == NULL || hw_if->set_pwm == NULL) {
    return false;
  }

  ctrl->hw_if = hw_if;
  ctrl->target_speed = 0;
  ctrl->current_speed = 0;
  ctrl->state = MOTOR_STATE_UNINITIALIZED;
  ctrl->is_emergency_stopped = false;

  // Attempt to reset hardware outputs.
  if (!ctrl->hw_if->set_gpio(GPIO_PIN_MOTOR_DIR, PIN_STATE_LOW) ||
      !ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0)) {
    ctrl->state = MOTOR_STATE_FAULT;
    return false;
  }

  ctrl->state = MOTOR_STATE_IDLE;
  return true;
}

bool motor_ctrl_set_speed(MotorCtrl *ctrl, int8_t speed) {
  if (ctrl == NULL) {
    return false;
  }

  // Check the safety state.
  if (!is_safe_state(ctrl, speed)) {
    ctrl->state = MOTOR_STATE_FAULT;
    // Immediately disable drive output on error for safety.
    if (ctrl->hw_if != NULL && ctrl->hw_if->set_pwm != NULL) {
      ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0);
    }
    return false;
  }

  ctrl->target_speed = speed;

  if (speed == 0) {
    if (ctrl->state != MOTOR_STATE_FAULT) {
      ctrl->state = MOTOR_STATE_IDLE;
    }
  } else {
    ctrl->state = MOTOR_STATE_RUNNING;
  }

  return true;
}

bool motor_ctrl_update(MotorCtrl *ctrl) {
  if (ctrl == NULL || ctrl->hw_if == NULL) {
    return false;
  }

  if (ctrl->state == MOTOR_STATE_FAULT || ctrl->is_emergency_stopped) {
  // In a fault or emergency-stop state, ensure zero output.
    ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0);
    ctrl->current_speed = 0;
    return false;
  }

  // Update the current speed directly; ramp smoothing may be added later.
  ctrl->current_speed = ctrl->target_speed;

  // 1. Set the direction GPIO level.
  PinState dir_state = speed_to_direction(ctrl->current_speed);
  if (!ctrl->hw_if->set_gpio(GPIO_PIN_MOTOR_DIR, dir_state)) {
    ctrl->state = MOTOR_STATE_FAULT;
    ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0);
    return false;
  }

  // 2. Set the PWM duty cycle.
  uint8_t duty = speed_to_pwm_duty(ctrl->current_speed);
  if (!ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, duty)) {
    ctrl->state = MOTOR_STATE_FAULT;
    ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0);
    return false;
  }

  return true;
}

bool motor_ctrl_emergency_stop(MotorCtrl *ctrl) {
  if (ctrl == NULL) {
    return false;
  }

  ctrl->is_emergency_stopped = true;
  ctrl->target_speed = 0;
  ctrl->current_speed = 0;
  ctrl->state = MOTOR_STATE_FAULT;

  if (ctrl->hw_if != NULL && ctrl->hw_if->set_pwm != NULL) {
    ctrl->hw_if->set_pwm(PWM_CHANNEL_MOTOR, 0);
  }

  return true;
}

MotorState motor_ctrl_get_state(const MotorCtrl *ctrl) {
  if (ctrl == NULL) {
    return MOTOR_STATE_FAULT;
  }
  return ctrl->state;
}
