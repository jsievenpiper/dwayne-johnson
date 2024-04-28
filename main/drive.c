#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#include "shared.h"

#define BUFFER_SIZE_ITEMS 1500

static uint8_t message_buffer_storage[BUFFER_SIZE_ITEMS * sizeof(control_t)];
StaticMessageBuffer_t message_buffer_structure;
MessageBufferHandle_t message_buffer;

void initialize_message_buffer(void) {
  message_buffer = xMessageBufferCreateStatic(
    sizeof(message_buffer_storage),
    message_buffer_storage,
    &message_buffer_structure
  );
}

void drive_loop(void* _pv_parameters) {
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
  
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
      if (control.left_magnitude > 0 || control.right_magnitude > 0) {
        gpio_set_level(GPIO_NUM_23, 1);
      }
      else {
        gpio_set_level(GPIO_NUM_23, 0);
      }
    }
    
    received_bytes = 0;
  }
}
