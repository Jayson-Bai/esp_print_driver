#temperature

idf_component_register(
    SRCS "temperature.c"
    INCLUDE_DIRS "."
    REQUIRES 
                esp_adc
)