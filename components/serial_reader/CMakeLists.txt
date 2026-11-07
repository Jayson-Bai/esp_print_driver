idf_component_register(
    
    SRCS "serial_reader.c"

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