idf_component_register(
    
    SRCS "serial_reader.c" "uart_tx_dispatch.c" "telemetry_scheduler.c"

    INCLUDE_DIRS "." 

        #REQUIRES 自定义组件
        REQUIRES    
                    motor 
                    temperature 
                    pid_ctrl
                    cutter
                    driver
                    fan

    
    )