#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "freertos/projdefs.h"
#include "hal/mcpwm_types.h"
#include "soc/gpio_num.h"

#include "shared.h"

#define BUFFER_SIZE_ITEMS 1500
#define PWM_LEFT_OUT GPIO_NUM_23
#define PWM_RIGHT_OUT GPIO_NUM_34

static uint8_t message_buffer_storage[BUFFER_SIZE_ITEMS * sizeof(control_t)];
StaticMessageBuffer_t message_buffer_structure;
MessageBufferHandle_t message_buffer;
mcpwm_timer_handle_t pwm_timer = NULL;
mcpwm_oper_handle_t pwm_operator = NULL;
mcpwm_cmpr_handle_t pwm_comparator = NULL;
mcpwm_gen_handle_t pwm_generator = NULL;

static inline uint32_t drive_magnitude_to_ticks(uint16_t magnitude) {
  return 650 * magnitude / 512;
}

void initialize_message_buffer(void) {
  message_buffer = xMessageBufferCreateStatic(
    sizeof(message_buffer_storage),
    message_buffer_storage,
    &message_buffer_structure
  );
}

void initialize_drive_mcpwm(void) {
  mcpwm_timer_config_t pwm_timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 10000000, // 10MHz - 0.1us/tick
    .period_ticks = 650, // 650 ticks - 65us - 15kHz
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP
  };

  mcpwm_new_timer(&pwm_timer_config, &pwm_timer);

  mcpwm_operator_config_t pwm_operator_config = {
    .group_id = 0
  };

  mcpwm_new_operator(&pwm_operator_config, &pwm_operator);
  mcpwm_operator_connect_timer(pwm_operator, pwm_timer);

  mcpwm_comparator_config_t pwm_comparator_config = {
    .flags.update_cmp_on_tez = true
  };

  mcpwm_new_comparator(pwm_operator, &pwm_comparator_config, &pwm_comparator);

  mcpwm_generator_config_t pwm_generator_config = {
    .gen_gpio_num = PWM_LEFT_OUT
  };

  mcpwm_new_generator(pwm_operator, &pwm_generator_config, &pwm_generator);

  mcpwm_comparator_set_compare_value(pwm_comparator, drive_magnitude_to_ticks(0));

  mcpwm_generator_set_action_on_timer_event(
    pwm_generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
  );

  mcpwm_generator_set_action_on_compare_event(
    pwm_generator, 
    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_comparator, MCPWM_GEN_ACTION_LOW)
  );

  mcpwm_timer_enable(pwm_timer);
  mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP);
}

void drive_loop(void* _pv_parameters) {
  control_t control = {
    .left_direction = NEUTRAL,
    .right_direction = NEUTRAL,
    .left_magnitude = 0,
    .right_magnitude = 0
  };

  size_t received_bytes = 0;

  for (;;) {
    received_bytes = xMessageBufferReceive(message_buffer, &control, sizeof(control), pdMS_TO_TICKS(1));

    if (received_bytes == sizeof(control)) {
      mcpwm_comparator_set_compare_value(pwm_comparator, drive_magnitude_to_ticks(control.left_magnitude));
      // if (control.left_magnitude > 0 || control.right_magnitude > 0) {
        // gpio_set_level(GPIO_NUM_23, 1);
      // }
      // else {
        // gpio_set_level(GPIO_NUM_23, 0);
      // }
    }
    
    received_bytes = 0;
  }
}
