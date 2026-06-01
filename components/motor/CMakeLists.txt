#motor

idf_component_register(
    SRCS "motor.c" "extruder_ctrl.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_rmt esp_timer
)
