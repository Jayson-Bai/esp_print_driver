#pid

idf_component_register(
    SRCS "pid_ctrl.c"
    INCLUDE_DIRS "."
    REQUIRES 
                esp_driver_ledc
                esp_driver_gpio
)