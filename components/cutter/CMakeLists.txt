#cutter

idf_component_register(
    SRCS "cutter.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
)