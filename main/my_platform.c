#include <stdlib.h>
#include <string.h>
#include <uni.h>

#include <freertos/FreeRTOS.h>
#include "controller/uni_controller.h"
#include "controller/uni_gamepad.h"
#include "freertos/projdefs.h"
#include "shared.h"

typedef struct my_platform_instance_s {
  uni_gamepad_seat_t gamepad_seat;
  control_t last_control_value;
} my_platform_instance_t;

// Deadzone. Couple of controllers I threw at this idle ~+/-5 and doing so much as breathing  near them will toss
// another couple of pts. Moving 512 units over ~1/4 or 1/2 of throw means the values can change super quick and are 
// quite sensitive, so 50 is actually a smaller deadzone than you'd expect.
const int32_t DEAD_ZONE = 50;

static void trigger_event_on_gamepad(uni_hid_device_t*);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t*);
static uint8_t display_gamepad_battery_status(uni_hid_device_t*);

/**
 * Executed when the system is first initialized -- nothing fancy to do here.
 */
static void my_platform_init(int argc, const char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  logi("custom: init()\n");
}

/**
 * Executed when the system initialization is complete. Here we will enable Bluetooth connections to occur.
 */
static void my_platform_on_init_complete(void) {
  logi("custom: on_init_complete()\n");

  // Safe to call "unsafe" functions since they are called from BT thread
  // Start scanning
  uni_bt_enable_new_connections_unsafe(true);
  uni_bt_list_keys_unsafe();
}

/**
 * Once a device has connected, we'll prevent other devices from connecting as a precautionary measure.
 */
static void my_platform_on_device_connected(uni_hid_device_t* hid) {
  logi("device connected, disabling other devices from connecting: %p\n", hid);
  uni_bt_enable_new_connections_safe(false);
}

/**
 * When a device disconnects, we'll allow connections again in case a controller accidentally drops out of range or
 * something like that.
 */
static void my_platform_on_device_disconnected(uni_hid_device_t* hid) {
  logi("device disconnected, reenabling connections: %p\n", hid);
  uni_bt_enable_new_connections_safe(true);
}

/**
 * Invoked when a controller is connected. We just do some light weight bookkeeping and then a small pulse on the
 * gamepad to let the operator know we're ready to rumble.
 */
static uni_error_t my_platform_on_device_ready(uni_hid_device_t* hid) {
  logi("custom: device ready: %p\n", hid);
  my_platform_instance_t* instance = get_my_platform_instance(hid);
  instance->gamepad_seat = GAMEPAD_SEAT_A;

  trigger_event_on_gamepad(hid);
  return UNI_ERROR_SUCCESS;
}

/**
 * Invoked specifically when the controller's right trigger is pulled.
 */
static control_t my_platform_on_throttle_data(uni_gamepad_t* gamepad) {
  return (control_t) {
    .left_direction = FORWARD,
    .right_direction = REVERSE,
    .left_magnitude = gamepad->throttle / 2,
    .right_magnitude = gamepad->throttle / 2
  };
}

/**
 * Invoked specifically when the controller's left trigger is pulled.
 */
static control_t my_platform_on_brake_data(uni_gamepad_t* gamepad) {
  return (control_t) {
    .left_direction = REVERSE,
    .right_direction = FORWARD,
    .left_magnitude = gamepad->brake / 2,
    .right_magnitude = gamepad->brake / 2
  };
}

/**
 * Invoked in all other cases, and reads the desired state from the analog sticks.
 */
static control_t my_platform_on_axis_data(uni_gamepad_t* gamepad) {
  return (control_t) {
    .left_direction = gamepad->axis_y > DEAD_ZONE ? REVERSE : gamepad->axis_y < -DEAD_ZONE ? FORWARD : NEUTRAL,
    .right_direction = gamepad->axis_ry > DEAD_ZONE ? REVERSE : gamepad->axis_ry < -DEAD_ZONE ? FORWARD : NEUTRAL,
    .left_magnitude = abs(gamepad->axis_y) > DEAD_ZONE ? abs(gamepad->axis_y) : 0,
    .right_magnitude = abs(gamepad->axis_ry) > DEAD_ZONE ? abs(gamepad->axis_ry) : 0
  };
}

/**
 * Invoked every time there is new controller data. This will delegate to other more specialized handlers (above) as
 * necessary.
 */
static void my_platform_on_controller_data(uni_hid_device_t* hid, uni_controller_t* controller) {
  static uint8_t display_on = 1;
  static uint8_t battery_status_updated = 0;
  uni_gamepad_t* gamepad;
  my_platform_instance_t* instance = get_my_platform_instance(hid);

  if (0 == battery_status_updated) {
    battery_status_updated = display_gamepad_battery_status(hid);
  }

  switch (controller->klass) {
    case UNI_CONTROLLER_CLASS_GAMEPAD:
      gamepad = &controller->gamepad;

      control_t control = gamepad->throttle > DEAD_ZONE 
        ? my_platform_on_throttle_data(gamepad)
        : gamepad->brake > DEAD_ZONE
        ? my_platform_on_brake_data(gamepad)
        : my_platform_on_axis_data(gamepad);

      uint16_t rumble_magnitude = control.left_magnitude > control.right_magnitude 
        ? control.left_magnitude : control.right_magnitude;
      
      uint8_t rumble_amount = 255 * rumble_magnitude / 512;

      if (NULL != hid->report_parser.play_dual_rumble && rumble_amount > 0) {
        hid->report_parser.play_dual_rumble(
          hid, 
          0 /* delayed start ms */, 
          250 /* duration ms */,
          rumble_amount /* weak magnitude */, 
          40 /* strong magnitude */
        );
      }

      if (0 == control_equals(&instance->last_control_value, &control)) {
        xMessageBufferSend(message_buffer, &control, sizeof(control), pdMS_TO_TICKS(1));
        instance->last_control_value = control;
      }

      if ((gamepad->buttons & BUTTON_A) && display_on) {
        turn_off_display();
        display_on = 0;
      }

      else if ((gamepad->buttons & BUTTON_B) && !display_on) {
        turn_on_display();
        display_on = 1;
      }

      break;

    default:
      break;
  }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
  ARG_UNUSED(idx);
  return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
  switch (event) {
    case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
      uni_hid_device_t* d = data;

      if (d == NULL) {
        loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
        return;
      }

      display_gamepad_battery_status(d);
      break;
    }

    case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
      logi("custom: Bluetooth enabled: %d\n", (bool)(data));
      break;

    default:
      logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
      break;
  }
}

/**
 * Helper function to get internal state related to an HID device (controller, in our case).
 */
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
  return (my_platform_instance_t*)&d->platform_data[0];
}

/**
 * For the given controller, relay it's battery levels onto our OLED.
 */
static uint8_t display_gamepad_battery_status(uni_hid_device_t* d) {
  if (0 == d->controller.battery) {
    return 0;
  }

  uint16_t battery_percentage = (d->controller.battery * 100) / 254;

  char hundreds = battery_percentage / 100;
  char tens = (battery_percentage % 100) / 10;
  char ones = (battery_percentage % 10);


  logi("%d %d battery (%d %d %d)", battery_percentage, hundreds, tens, ones);

  write_string("Ready!", 0);
  write_string("---------------------", 1);
  write_string("Controller", 2);
  
  if (1 == hundreds) {
    // bit of a special case, but if we have a hundreds place there's no need to actually wonder about the full battery
    // percentage if we did our math correctly.
    write_string("Battery: 100%", 3);
  }

  // And here's our regular ol' 0-99 battery percentage display logic.
  else {
    if (0 == tens) {
      write_string("Battery:  %", 3);
      write_char('0' + ones, 3, 9);
    }

    else {
      write_string("Battery:   %", 3);
      write_char('0' + tens, 3, 9);
      write_char('0' + ones, 3, 10);
    }
  }

  update_display();
  return 1;
}

/**
 * Small little routine when the controller connects to rumble a bit and light up pink.
 */
static void trigger_event_on_gamepad(uni_hid_device_t* d) {
  if (d->report_parser.play_dual_rumble != NULL) {
    d->report_parser.play_dual_rumble(
      d,
      0 /* delayed start ms */, 
      150 /* duration ms */, 
      128 /* weak magnitude */,
      40 /* strong magnitude */
    );
  }

  if (d->report_parser.set_lightbar_color != NULL) {
    d->report_parser.set_lightbar_color(d, 0xFF, 0x00, 0x80);
  }
}

/**
 * Defines a series of function pointers that allow us to have callbacks into the controller connection and data
 * procedures. This will be how we primarily obtain data from the controller for use in the rest of the system.
 */
struct uni_platform* get_my_platform(void) {
  static struct uni_platform plat = {
    .name = "custom",
    .init = my_platform_init,
    .on_init_complete = my_platform_on_init_complete,
    .on_device_connected = my_platform_on_device_connected,
    .on_device_disconnected = my_platform_on_device_disconnected,
    .on_device_ready = my_platform_on_device_ready,
    .on_oob_event = my_platform_on_oob_event,
    .on_controller_data = my_platform_on_controller_data,
    .get_property = my_platform_get_property,
  };

  return &plat;
}
