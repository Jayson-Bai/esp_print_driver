idf_component_register(SRCS
    screens/ui_HomeScreen.c
    screens/ui_AxisScreen.c
    screens/ui_TempScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_home_fill_png.c
    images/ui_img_emergency_png.c
    images/ui_img_browser_extruder3_png.c
    images/ui_img_browser_extruder1_png.c
    images/ui_img_fan_png.c
    images/ui_img_fan1_png.c
    images/ui_img_browser_extruder2_png.c
    fonts/ui_font_Font1.c


    INCLUDE_DIRS "." 

        #REQUIRES 自定义组件
        REQUIRES    
                    lvgl
                    motor 
                    temperature 
                    pid_ctrl
                    cutter
                    fan
    
    )


