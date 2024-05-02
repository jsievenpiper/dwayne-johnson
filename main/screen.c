#include <driver/i2c_master.h>
#include <stdbool.h>
#include <stdint.h>
#include <driver/i2c_types.h>
#include <hal/i2c_types.h>
#include <string.h>

#define HORIZONTAL_RESOLUTION 128
#define DISPLAY_PAGES 8
#define FONT_WIDTH 6
// There are technically twice as many pages as there are available to fit on one screen, but we don't really want to
// leverage them, nor bother allocating the memory for them.
#define LCD_CACHE_SIZE ((DISPLAY_PAGES * HORIZONTAL_RESOLUTION) + 1)
#define SSD1306_ADDRESS 0x3C
#define MAX_CHARS_PER_LINE (HORIZONTAL_RESOLUTION / FONT_WIDTH)

static const uint8_t FONT[][FONT_WIDTH] = {
  {0x00,0x00,0x00,0x00,0x00,0x00},	// 0x20
  {0x00,0x00,0x06,0x5F,0x06,0x00},	// 0x21
  {0x00,0x07,0x03,0x00,0x07,0x03},	// 0x22
  {0x00,0x24,0x7E,0x24,0x7E,0x24},	// 0x23
  {0x00,0x24,0x2B,0x6A,0x12,0x00},	// 0x24
  {0x00,0x63,0x13,0x08,0x64,0x63},	// 0x25
  {0x00,0x36,0x49,0x56,0x20,0x50},	// 0x26
  {0x00,0x00,0x07,0x03,0x00,0x00},	// 0x27
  {0x00,0x00,0x3E,0x41,0x00,0x00},	// 0x28
  {0x00,0x00,0x41,0x3E,0x00,0x00},	// 0x29
  {0x00,0x08,0x3E,0x1C,0x3E,0x08},	// 0x2A
  {0x00,0x08,0x08,0x3E,0x08,0x08},	// 0x2B
  {0x00,0x00,0xE0,0x60,0x00,0x00},	// 0x2C
  {0x00,0x08,0x08,0x08,0x08,0x08},	// 0x2D
  {0x00,0x00,0x60,0x60,0x00,0x00},	// 0x2E
  {0x00,0x20,0x10,0x08,0x04,0x02},	// 0x2F
  {0x00,0x3E,0x51,0x49,0x45,0x3E},	// 0x30
  {0x00,0x00,0x42,0x7F,0x40,0x00},	// 0x31
  {0x00,0x62,0x51,0x49,0x49,0x46},	// 0x32
  {0x00,0x22,0x49,0x49,0x49,0x36},	// 0x33
  {0x00,0x18,0x14,0x12,0x7F,0x10},	// 0x34
  {0x00,0x2F,0x49,0x49,0x49,0x31},	// 0x35
  {0x00,0x3C,0x4A,0x49,0x49,0x30},	// 0x36
  {0x00,0x01,0x71,0x09,0x05,0x03},	// 0x37
  {0x00,0x36,0x49,0x49,0x49,0x36},	// 0x38
  {0x00,0x06,0x49,0x49,0x29,0x1E},	// 0x39
  {0x00,0x00,0x6C,0x6C,0x00,0x00},	// 0x3A
  {0x00,0x00,0xEC,0x6C,0x00,0x00},	// 0x3B
  {0x00,0x08,0x14,0x22,0x41,0x00},	// 0x3C
  {0x00,0x24,0x24,0x24,0x24,0x24},	// 0x3D
  {0x00,0x00,0x41,0x22,0x14,0x08},	// 0x3E
  {0x00,0x02,0x01,0x59,0x09,0x06},	// 0x3F
  {0x00,0x3E,0x41,0x5D,0x55,0x1E},	// 0x40
  {0x00,0x7E,0x11,0x11,0x11,0x7E},	// 0x41
  {0x00,0x7F,0x49,0x49,0x49,0x36},	// 0x42
  {0x00,0x3E,0x41,0x41,0x41,0x22},	// 0x43
  {0x00,0x7F,0x41,0x41,0x41,0x3E},	// 0x44
  {0x00,0x7F,0x49,0x49,0x49,0x41},	// 0x45
  {0x00,0x7F,0x09,0x09,0x09,0x01},	// 0x46
  {0x00,0x3E,0x41,0x49,0x49,0x7A},	// 0x47
  {0x00,0x7F,0x08,0x08,0x08,0x7F},	// 0x48
  {0x00,0x00,0x41,0x7F,0x41,0x00},	// 0x49
  {0x00,0x30,0x40,0x40,0x40,0x3F},	// 0x4A
  {0x00,0x7F,0x08,0x14,0x22,0x41},	// 0x4B
  {0x00,0x7F,0x40,0x40,0x40,0x40},	// 0x4C
  {0x00,0x7F,0x02,0x04,0x02,0x7F},	// 0x4D
  {0x00,0x7F,0x02,0x04,0x08,0x7F},	// 0x4E
  {0x00,0x3E,0x41,0x41,0x41,0x3E},	// 0x4F
  {0x00,0x7F,0x09,0x09,0x09,0x06},	// 0x50
  {0x00,0x3E,0x41,0x51,0x21,0x5E},	// 0x51
  {0x00,0x7F,0x09,0x09,0x19,0x66},	// 0x52
  {0x00,0x26,0x49,0x49,0x49,0x32},	// 0x53
  {0x00,0x01,0x01,0x7F,0x01,0x01},	// 0x54
  {0x00,0x3F,0x40,0x40,0x40,0x3F},	// 0x55
  {0x00,0x1F,0x20,0x40,0x20,0x1F},	// 0x56
  {0x00,0x3F,0x40,0x3C,0x40,0x3F},	// 0x57
  {0x00,0x63,0x14,0x08,0x14,0x63},	// 0x58
  {0x00,0x07,0x08,0x70,0x08,0x07},	// 0x59
  {0x00,0x71,0x49,0x45,0x43,0x00},	// 0x5A
  {0x00,0x00,0x7F,0x41,0x41,0x00},	// 0x5B
  {0x00,0x02,0x04,0x08,0x10,0x20},	// 0x5C
  {0x00,0x00,0x41,0x41,0x7F,0x00},	// 0x5D
  {0x00,0x04,0x02,0x01,0x02,0x04},	// 0x5E
  {0x80,0x80,0x80,0x80,0x80,0x80},	// 0x5F
  {0x00,0x00,0x03,0x07,0x00,0x00},	// 0x60
  {0x00,0x20,0x54,0x54,0x54,0x78},	// 0x61
  {0x00,0x7F,0x44,0x44,0x44,0x38},	// 0x62
  {0x00,0x38,0x44,0x44,0x44,0x28},	// 0x63
  {0x00,0x38,0x44,0x44,0x44,0x7F},	// 0x64
  {0x00,0x38,0x54,0x54,0x54,0x08},	// 0x65
  {0x00,0x08,0x7E,0x09,0x09,0x00},	// 0x66
  {0x00,0x18,0xA4,0xA4,0xA4,0x7C},	// 0x67
  {0x00,0x7F,0x04,0x04,0x78,0x00},	// 0x68
  {0x00,0x00,0x00,0x7D,0x40,0x00},	// 0x69
  {0x00,0x40,0x80,0x84,0x7D,0x00},	// 0x6A
  {0x00,0x7F,0x10,0x28,0x44,0x00},	// 0x6B
  {0x00,0x00,0x00,0x7F,0x40,0x00},	// 0x6C
  {0x00,0x7C,0x04,0x18,0x04,0x78},	// 0x6D
  {0x00,0x7C,0x04,0x04,0x78,0x00},	// 0x6E
  {0x00,0x38,0x44,0x44,0x44,0x38},	// 0x6F
  {0x00,0xFC,0x44,0x44,0x44,0x38},	// 0x70
  {0x00,0x38,0x44,0x44,0x44,0xFC},	// 0x71
  {0x00,0x44,0x78,0x44,0x04,0x08},	// 0x72
  {0x00,0x08,0x54,0x54,0x54,0x20},	// 0x73
  {0x00,0x04,0x3E,0x44,0x24,0x00},	// 0x74
  {0x00,0x3C,0x40,0x20,0x7C,0x00},	// 0x75
  {0x00,0x1C,0x20,0x40,0x20,0x1C},	// 0x76
  {0x00,0x3C,0x60,0x30,0x60,0x3C},	// 0x77
  {0x00,0x6C,0x10,0x10,0x6C,0x00},	// 0x78
  {0x00,0x9C,0xA0,0x60,0x3C,0x00},	// 0x79
  {0x00,0x64,0x54,0x54,0x4C,0x00},	// 0x7A
  {0x00,0x08,0x3E,0x41,0x41,0x00},	// 0x7B
  {0x00,0x00,0x00,0x77,0x00,0x00},	// 0x7C
  {0x00,0x00,0x41,0x41,0x3E,0x08},	// 0x7D
  {0x00,0x02,0x01,0x02,0x01,0x00},	// 0x7E
  {0x00,0x3C,0x26,0x23,0x26,0x3C},	// 0x7F
};

static uint8_t* lcd;
i2c_master_bus_handle_t master_handle;
i2c_master_dev_handle_t display_handle;

void update_display(void) {
  i2c_master_transmit(display_handle, lcd, LCD_CACHE_SIZE, -1);
  i2c_master_bus_wait_all_done(master_handle, -1);
}

void clear_display(void) {
  memset(lcd, 0x00, LCD_CACHE_SIZE);

  lcd[0] = 0x40;
  update_display();
}

void write_char(char c, uint8_t page, uint8_t idx) {
  const uint8_t* character_glyph = FONT[c - 0x20];
  uint8_t i = 0;

  for (i = 0; i < FONT_WIDTH; i++) {
    lcd[(page * HORIZONTAL_RESOLUTION) + (idx * FONT_WIDTH) + i + 1] = character_glyph[i];
  }
}

void write_string(char* string, uint8_t page) {
  uint8_t i = 0;

  while (string[i] != '\0') {
    write_char(string[i], page, i);
    i++;
  }

  // Blank out the rest of the line to avoid needing to clear out the whole display just to rewrite a line.
  while (i < MAX_CHARS_PER_LINE) {
    write_char(' ', page, i++);
  }
}

void turn_on_display(void) {
  i2c_master_transmit(display_handle, (uint8_t[]) { 0x80, 0xAF }, 2, -1);
  i2c_master_bus_wait_all_done(master_handle, -1);
}

void turn_off_display(void) {
  i2c_master_transmit(display_handle, (uint8_t[]) { 0x80, 0xAE }, 2, -1);
  i2c_master_bus_wait_all_done(master_handle, -1);
}

void initialize_display(void) {
  lcd = malloc(LCD_CACHE_SIZE);

  i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  i2c_new_master_bus(&i2c_config, &master_handle);

  i2c_device_config_t display_config = {
    .dev_addr_length = I2C_ADDR_BIT_7,
    .device_address = SSD1306_ADDRESS,
    .scl_speed_hz = 100000 // could also possibly be 400kHz
  };

  i2c_master_bus_add_device(master_handle, &display_config, &display_handle);

  uint8_t initialization_routine[] = {
    0x80, 0xAE, // turn off display
    0x80, 0xA8, 0x80, 0x1F, // set display to 128x32 mode
    0x80, 0x20, 0x80, 0x00, // address memory horizontally
    0x80, 0x40, // set start line
    0x80, 0xD3, 0x80, 0x00, // set display offset
    0x80, 0xA0, // remap
    0x80, 0xC0, // scan direction
    0x80, 0xDA, 0x80, 0x02, // COM pin configuration for 128x32
    0x80, 0x81, 0x80, 0x7F, // set contrast
    0x80, 0xA4,
    0x80, 0xA6,
    0x80, 0xD5, 0x80, 0x80, // oscillator frequency
    0x80, 0xD9, 0x80, 0xC2, // display precharge: higher argument value, less blinky
    0x80, 0xDB, 0x80, 0x20,
    0x80, 0x8D, 0x80, 0x14,
    0x80, 0x2E,
    0x80, 0xAF // display on
  };

  i2c_master_transmit(display_handle, initialization_routine, sizeof(initialization_routine), -1);
  i2c_master_bus_wait_all_done(master_handle, -1);
  clear_display();
}



