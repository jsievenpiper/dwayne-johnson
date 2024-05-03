#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>

#include "sdkconfig.h"
#include "shared.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

int app_main(void) {
  initialize_message_buffer();
  initialize_drive_mcpwm();
  initialize_speed_control_adc();
  initialize_display();

  // Projects are no fun without unnecessary cute additions. This is one of those cute additions.
  write_string("This fish was built", 0);
  write_string("for speed!", 1);
  write_string("waiting...", 3);
  update_display();

  // Bluetooth is totally plausible but still a lot for the ESP32. Give one whole core to bluetooth processing to keep
  // it responsive and we'll settle for the other core. Remember when having two cores in a personal computer was a
  // huge deal and now you can get it packaged in a $5 chip?
  xTaskCreatePinnedToCore(
    drive_loop,
    "drive",
    1000,
    NULL,
    0,
    NULL,
    1
  );

  // Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
  btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART

  // Configure BTstack for ESP32 VHCI Controller
  btstack_init();

  // Must be called before uni_init()
  uni_platform_set_custom(get_my_platform());

  // Init Bluepad32.
  uni_init(0, NULL);

  // Does not return and will pin to core 0.
  btstack_run_loop_execute();
  return 0;
}
