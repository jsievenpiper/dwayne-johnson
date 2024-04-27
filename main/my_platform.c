#include <stdlib.h>
#include <string.h>
#include <uni.h>

typedef struct my_platform_instance_s {
  uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

static void my_platform_init(int argc, const char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  logi("custom: init()\n");
}

static void my_platform_on_init_complete(void) {
  logi("custom: on_init_complete()\n");

  // Safe to call "unsafe" functions since they are called from BT thread
  // Start scanning
  uni_bt_enable_new_connections_unsafe(true);
  uni_bt_list_keys_unsafe();
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
  logi("device connected, disabling other devices from connecting: %p\n", d);
  uni_bt_enable_new_connections_safe(false);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
  logi("device disconnected, reenabling connections: %p\n", d);
  uni_bt_enable_new_connections_safe(true);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
  logi("custom: device ready: %p\n", d);
  my_platform_instance_t* ins = get_my_platform_instance(d);
  ins->gamepad_seat = GAMEPAD_SEAT_A;

  trigger_event_on_gamepad(d);
  return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
  uni_gamepad_t* gp;

  switch (ctl->klass) {
    case UNI_CONTROLLER_CLASS_GAMEPAD:
      gp = &ctl->gamepad;

      // uint8_t l_forward = gp->axis_y < 0 ? 1 : 0;
      // uint8_t r_forward = gp->axis_ry < 0 ? 1 : 0;
      uint16_t directional_magnitude = abs(gp->axis_y) > abs(gp->axis_ry) ? abs(gp->axis_y) : abs(gp->axis_ry);
      
      // deadzone
      directional_magnitude = directional_magnitude > 50 ? directional_magnitude : 0;
      
      uint8_t rumble_amount = 255 * directional_magnitude / 512;

      if (NULL != d->report_parser.play_dual_rumble && rumble_amount > 0) {
        d->report_parser.play_dual_rumble(
          d, 
          0 /* delayed start ms */, 
          250 /* duration ms */,
          rumble_amount /* weak magnitude */, 
          40 /* strong magnitude */
        );
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

      logi("custom: on_device_oob_event(): %d\n", event);

      my_platform_instance_t* ins = get_my_platform_instance(d);
      ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

      trigger_event_on_gamepad(d);
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

//
// Helpers
//
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
  return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
  // my_platform_instance_t* ins = get_my_platform_instance(d);

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

//
// Entry Point
//
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
