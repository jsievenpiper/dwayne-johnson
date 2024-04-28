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
  uint16_t left_magnitude;
  uint16_t right_magnitude;
} control_t;

extern MessageBufferHandle_t message_buffer;

struct uni_platform* get_my_platform(void);
void drive_loop(void*);
void initialize_message_buffer(void);

#endif // !DWAYNE_JOHNSON_SHAREDDWAYNE_JOHNSON_SHARED
