#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "hal/mcpwm_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#include "shared.h"

#define BUFFER_SIZE_ITEMS 1500
#define TICK_RESOLUTION 650
#define UPDATE_LOOP_TICK_FREQUENCY 5
#define UPDATE_PROFILE_ACCEL_AMOUNT 10
#define UPDATE_PROFILE_DECEL_AMOUNT 20
#define PWM_LEFT_OUT GPIO_NUM_23
#define PWM_RIGHT_OUT GPIO_NUM_27

static uint8_t message_buffer_storage[BUFFER_SIZE_ITEMS * sizeof(control_t)];
StaticMessageBuffer_t message_buffer_structure;
MessageBufferHandle_t message_buffer;
mcpwm_timer_handle_t pwm_timer = NULL;
mcpwm_oper_handle_t pwm_operator = NULL;
mcpwm_cmpr_handle_t pwm_comparator_left = NULL;
mcpwm_cmpr_handle_t pwm_comparator_right = NULL;
mcpwm_gen_handle_t pwm_generator_left = NULL;
mcpwm_gen_handle_t pwm_generator_right = NULL;

/**
 * Simple conversion from the 9-bit controller value to the amount of ticks that would have to occur to generate a 
 * pulse width duty cycle of the same representative percentage.
 *
 * When our potentiometers come in, this will then also be scaled to a defined maximum value based on the potentiometer
 * value to make this range less twitchy if it proves that the maximum speeds are always too high.
 */
static inline uint32_t drive_magnitude_to_ticks(uint16_t magnitude) {
  return TICK_RESOLUTION * magnitude / 512;
}

/**
 * Creates an initial, idle value that doesn't rocket a child into the audience.
 */
control_t idle_control_value(void) {
  control_t idle = {
    .left_direction = NEUTRAL,
    .right_direction = NEUTRAL,
    .left_magnitude = 0,
    .right_magnitude = 0
  };

  return idle;
}

/**
 * Determines if two control structures are equal.
 */
uint8_t control_equals(control_t* a, control_t* b) {
  if (
    a->left_direction == b->left_direction
    && a->right_direction == b->right_direction
    && a->left_magnitude == b->left_magnitude
    && a->right_magnitude == b->right_magnitude
  ) {
    return 1;
  }

  return 0;
}

/**
  * Initializes the buffer that allows data to be sent back from the controller/BT core to our drive core.
  */
void initialize_message_buffer(void) {
  message_buffer = xMessageBufferCreateStatic(
    sizeof(message_buffer_storage),
    message_buffer_storage,
    &message_buffer_structure
  );
}

/**
 * Initializes the PWM motor control generators and associated infrastructure around those.
 */
void initialize_drive_mcpwm(void) {
  // Very conveniently, esp32 pwm timers can power multiple generators. Since our two motor control circuits are the
  // same, we'll get hardware-level synchronized pulses. Timers across groups can also be synchronized, but we just
  // need a single timer to run both controllers at the same frequency.
  mcpwm_timer_config_t pwm_timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 10000000, // 10MHz - 0.1us/tick
    .period_ticks = 650, // 650 ticks - 65us - 15kHz
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP
  };

  mcpwm_new_timer(&pwm_timer_config, &pwm_timer);

  // Assign the operator to the same group to access the timer defined above, then, connect the timer to the operator.
  mcpwm_operator_config_t pwm_operator_config = {
    .group_id = 0
  };

  mcpwm_new_operator(&pwm_operator_config, &pwm_operator);
  mcpwm_operator_connect_timer(pwm_operator, pwm_timer);

  // Define our comparators, which will allow us to define the behavior of the pulses generated. We need two
  // comparators here because we will be driving the motors independently.
  mcpwm_comparator_config_t pwm_comparator_config = {
    .flags.update_cmp_on_tez = true
  };

  mcpwm_new_comparator(pwm_operator, &pwm_comparator_config, &pwm_comparator_left);
  mcpwm_new_comparator(pwm_operator, &pwm_comparator_config, &pwm_comparator_right);

  // Similarly, we will create two generators, because we want to generate asymmetric waveforms. By following the
  // configuration structure below, we'll obviously need to configure these separately as we're mapping out hardware
  // at this point and need to put the signal out on different pins.
  mcpwm_generator_config_t pwm_generator_config_left = {
    .gen_gpio_num = PWM_LEFT_OUT
  };

  mcpwm_generator_config_t pwm_generator_config_right = {
    .gen_gpio_num = PWM_RIGHT_OUT
  };

  mcpwm_new_generator(pwm_operator, &pwm_generator_config_left, &pwm_generator_left);
  mcpwm_new_generator(pwm_operator, &pwm_generator_config_right, &pwm_generator_right);

  // Initialize our drive system to zero. You know, so we don't flip a switch and send a kid flying off the drop zone.
  mcpwm_comparator_set_compare_value(pwm_comparator_left, drive_magnitude_to_ticks(0));
  mcpwm_comparator_set_compare_value(pwm_comparator_right, drive_magnitude_to_ticks(0));

  // Define the timer behaviors that drive our generator. To build simple active-high asymmetric signals, we'll just
  // count up and go low when the comparator threshold is hit.
  mcpwm_generator_set_action_on_timer_event(
    pwm_generator_left,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
  );

  mcpwm_generator_set_action_on_timer_event(
    pwm_generator_right,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
  );

  mcpwm_generator_set_action_on_compare_event(
    pwm_generator_left, 
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_comparator_left, MCPWM_GEN_ACTION_LOW)
  );

  mcpwm_generator_set_action_on_compare_event(
    pwm_generator_right, 
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_comparator_right, MCPWM_GEN_ACTION_LOW)
  );

  // Fingers crossed, fire up the timer.
  mcpwm_timer_enable(pwm_timer);
  mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP);
}

/**
 * As the name implies, updates the current magnitude one unit such that it approaches the desired value while 
 * accounting for our acceleration profile. If the value is already at the correct value, this method is a no-op.
 */
void accelerate(int16_t desired, int16_t* current) {
  // If accelerating, increase the magnitude by the acceleration profile.
  *current += UPDATE_PROFILE_ACCEL_AMOUNT;

  // But don't let it exceed the actual amount desired.
  if (*current > desired) {
    *current = desired;
  }
}

/**
 * Similar to accelerate, updates the current magnitude one unit such that it approaches the desired value while
 * account for our deceleration profile. If we're already at the correct value, this method is a no-op.
 */
void decelerate(int16_t desired, int16_t* current) {
  // If decelerating, same thing, but take into account the deceleration profile.
  *current -= UPDATE_PROFILE_DECEL_AMOUNT;

  // Don't let it undercut the desired value.
  if (*current < desired) {
    *current = desired;
  }
}

void calculate_drive_state(
  direction_t desired_direction,
  direction_t* current_direction,
  int16_t desired_magnitude,
  int16_t* current_magnitude
) {
  // In this branch, the desired state is to be stopped. No matter the direction, decelerate toward zero. 
  if (NEUTRAL == desired_direction || (desired_direction != *current_direction && NEUTRAL != *current_direction)) {
    decelerate(0, current_magnitude);

    // Once we hit zero, we can switch our current state to NEUTRAL as well.
    if (0 == *current_magnitude) {
      *current_direction = NEUTRAL;
    }
  }

  // Desired state is to continue in the same direction as the motor is currently being driven, or the motor is being
  // driven from an idle state.
  else if (desired_direction == *current_direction || NEUTRAL == *current_direction) {
    // Update the direction the motor is going.
    *current_direction = desired_direction;

    if (desired_magnitude > *current_magnitude) {
      accelerate(desired_magnitude, current_magnitude);
    }

    else if (desired_magnitude < *current_magnitude) {
      decelerate(desired_magnitude, current_magnitude);
    }
  }
}

/**
 * This is our core two main loop if you will, and should never return.
 */
void drive_loop(void* _pv_parameters) {
  // This stores the current state of our _intended_ drive control.
  control_t desired_state = idle_control_value();

  // And this stores our actual state. This is where the fun of hardware and software coming together is at. While pure
  // software can update value more-or-less instantaneously (and from our actual code perspective, this is effectively
  // true because the program counter won't actually advance instructions otherwise) -- hardware has physical 
  // characteristics to overcome. Some of these might be technical (instantly stopping the motor is hard on it), and
  // some of these are self-imposed (we probably want to limit the acceleration and deceleration profiles of the motors
  // as they'll be hauling humans around).
  //
  // To get around that, we'll keep track of the intended state above, which will always be "instantaneously" accurate,
  // but have a means of actually driving the motors in a more damped fashion. Maybe think of it as a spring force on
  // the motor. I guess technically we could implement this as a spring function. Let's do linear though. Linear is
  // easier.
  control_t current_state = idle_control_value();

  // This field stores the number of bytes that was read from picking a message off the buffer. It should, in correct
  // operating mode, read either zero (no new data was received from the controller) or sizeof(control_t); Any other
  // value is corruption.
  size_t received_bytes = 0;

  // This counter stores our acceleration/deceleration profile value. Basically, rather than immediately jumping to
  // the next value, we will tween the value from the current state to the desired state. There are two values to 
  // adjust here to change the feel of the input, both statically defined:
  // 
  // UPDATE_PROFILE_ACCEL_AMOUNT
  //   Defines, when an update does occur, by how much that value can change when _accelerating_. For example, after
  //   UPDATE_PROFILE_TICKS, if the current state has a magnitude of 10, and the desired state has a magnitude of 100,
  //   the current state will be updated to be 10 + UPDATE_PROFILE_TICKS (but no more than 100). The counter then 
  //   resets, and this logic is ran over again until reaching equilibrium (or will continuously chase a value if that
  //   desired value continues to evade the current state).
  // 
  // UPDATE_PROFILE_DECEL_AMOUNT
  //   The same description as UPDATE_PROFILE_ACCEL_AMOUNT, but for deceleration. This allows the system to decelerate
  //   with a different profile from acceleration, in case that is desirable.
  TickType_t last_wake_time = xTaskGetTickCount();

  // This loop follows a pretty Elm-like message -> update -> render loop, but with hardware as our "render" portion.
  // We start each event loop by querying our message buffer for updates to process. We apply those processes to our
  // state (with accomodations for an "actual" (current) and desired state to account for the real world / human factor
  // that isn't present in pure software). Then finally we "render" those values by adjusting the hardware outputs to
  // appropriately represent our state.
  for (;;) {
    // Attempt to read a new desired state from the other core. We'll fast-forward through the whole queue, as any
    // intermediate values we weren't able to receive previously mean nothing to us now. We'll do this every loop cycle
    // (even if we have no intention this cycle of doing anything) so that we keep the buffer flowing.
    do {
      received_bytes = xMessageBufferReceive(message_buffer, &desired_state, sizeof(desired_state), 0);
    } while (0 != received_bytes);

    // update
    calculate_drive_state(
      desired_state.left_direction,
      &current_state.left_direction,
      desired_state.left_magnitude,
      &current_state.left_magnitude
    );

    calculate_drive_state(
      desired_state.right_direction,
      &current_state.right_direction,
      desired_state.right_magnitude,
      &current_state.right_magnitude
    );

    mcpwm_comparator_set_compare_value(pwm_comparator_left, drive_magnitude_to_ticks(current_state.left_magnitude));
    mcpwm_comparator_set_compare_value(pwm_comparator_right, drive_magnitude_to_ticks(current_state.right_magnitude));

    vTaskDelayUntil(&last_wake_time, UPDATE_LOOP_TICK_FREQUENCY);
  }
}
