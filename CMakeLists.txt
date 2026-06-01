#根目录  CMakeLists.txt   此文件中声明的变量全局有效

cmake_minimum_required(VERSION 3.5)

if (NOT DEFINED PROJECT_NAME)   
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)   #根目录：$ENV{IDF_PATH} = /home/jaysonbai/esp/esp-idf
    

    #项目额外组件
    set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components  
                             ${CMAKE_CURRENT_SOURCE_DIR}/lvgl 
                             ${CMAKE_CURRENT_SOURCE_DIR}/lvgl_esp32_drivers 
                             #${CMAKE_CURRENT_SOURCE_DIR}/ui
                             ${CMAKE_CURRENT_SOURCE_DIR}/components
                             )

    project(print_head)
else()
    message(FATAL_ERROR "LV PORT ESP32: This must be a project's main CMakeLists.txt.")
endif()
