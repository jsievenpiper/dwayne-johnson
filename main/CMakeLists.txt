set(srcs "main.c" "my_platform.c" "drive.c" "screen.c")

set(requires "bluepad32" "btstack" "esp_adc")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
