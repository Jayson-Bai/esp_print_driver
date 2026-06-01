
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "lvgl_helpers.h"
#include "core/lv_disp.h"
#include "freertos/projdefs.h"
#include "hal/lv_hal_disp.h"
#include "freertos/FreeRTOSConfig.h"

#include "lvgl_touch/touch_driver.h"
#include "lvgl_touch/ft6x36.h"
#include "lvgl_i2c/i2c_manager.h"
#include "driver/i2c.h"
#include "esp_log.h"


#include "ui.h"
#include "motor.h"
#include "temperature.h"
#include "pid_ctrl.h"
#include "fan.h"
#include "serial_reader.h"
#include "cutter.h"
#include "extruder_ctrl.h"


#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"


/*********************
 *      DEFINES
 *********************/
#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
void motor1_task(void *pvParameters);
void motor2_task(void *pvParameters);
void motor3_task(void *pvParameters);
void motor4_task(void *pvParameters);
void stop_monitor_task(void *arg);
void PID1_Task(void *pvParameters);
void PID2_Task(void *pvParameters);
void cutter_task(void *pvParameters);
void fan_task(void *pvParameters);

TaskHandle_t motor3_task_handle = NULL;
TaskHandle_t motor4_task_handle = NULL;

void app_main() {
    uart0_init(115200);

    //温度采样后台任务
    temperature_task_start();

    //串口接收/解析任务
    serial_tasks_start();
    command_dispatch_task_start();
    status_task_start();
    
    //纤维轴电机控制任务     （任务函数，     任务名称，       栈大小，  参数， 优先级，任务句柄，核心）
    xTaskCreatePinnedToCore(motor1_task, "motor1_ctrl",   4096,   NULL,   5,    NULL,   1);

    xTaskCreatePinnedToCore(motor2_task, "motor2_ctrl",   4096,   NULL,   5,    NULL,   1);

    vTaskDelay(pdMS_TO_TICKS(200));

    xTaskCreatePinnedToCore(motor3_task, "motor3_ctrl",   4096,   NULL,   5,    &motor3_task_handle,   1);

    xTaskCreatePinnedToCore(motor4_task, "motor4_ctrl",   4096,   NULL,   5,    &motor4_task_handle,   1);

    extruder_task_start();

    //电机限位任务
    xTaskCreatePinnedToCore(stop_monitor_task, "motor_stop",   4096,   NULL,   5,    NULL,   1);

    //gui常驻任务（非临时栈，需要持续运行）
    xTaskCreatePinnedToCore(guiTask, "gui", 8192*2, NULL, 4, NULL, 0);

    //cutter任务
    xTaskCreatePinnedToCore(cutter_task, "cutter", 4096, NULL, 1, NULL, 1);
 
    //纤维加热任务
    xTaskCreatePinnedToCore(PID1_Task, "PID1", 8192*2, NULL, 3, NULL, 1);

    //树脂加热任务
    xTaskCreatePinnedToCore(PID2_Task, "PID2", 8192*2, NULL, 3, NULL, 1);

    //风扇任务
    xTaskCreatePinnedToCore(fan_task, "fan", 4096, NULL, 1, NULL, 1);

}




/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

//gui任务
static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    // 初始化lvgl库
    lv_init();

    // 初始化显示驱动使用的SPI或I2C总线
    lvgl_driver_init();


    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_draw_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

    
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */ //注册触摸驱动（包含初始化触摸驱动、初始化I2C）
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    //创建自定义UI
    ui_init();

    TickType_t last_wake = xTaskGetTickCount();
    //ui任务持续循环运行
    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}

//lvgl时钟
static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
