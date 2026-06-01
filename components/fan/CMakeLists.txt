#fan

idf_component_register(
    SRCS "fan.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
)