#ifndef DWAYNE_JOHNSON_SHARED
#define DWAYNE_JOHNSON_SHARED

#include <stdint.h>
#include <freertos/FreeRTOS.h>
extern const int32_t DEAD_ZONE;

typedef enum direction {
  NEUTRAL,
  FORWARD,
  REVERSE
} direction_t;

typedef struct {
  direction_t left_direction;
  direction_t right_direction;
  int16_t left_magnitude;
  int16_t right_magnitude;
} control_t;

extern MessageBufferHandle_t message_buffer;

// platform
struct uni_platform* get_my_platform(void);

// drive
control_t idle_control_value(void);
uint8_t control_equals(control_t*, control_t*);
void drive_loop(void*);
void initialize_message_buffer(void);
void initialize_drive_mcpwm(void);

// display
void initialize_display(void);
void turn_on_display(void);
void turn_off_display(void);
void write_char(char, uint8_t, uint8_t);
void write_string(char*, uint8_t);
void update_display(void);

#endif // !DWAYNE_JOHNSON_SHAREDDWAYNE_JOHNSON_SHARED
