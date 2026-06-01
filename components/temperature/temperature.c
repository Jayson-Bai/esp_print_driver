#include "temperature.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "TEMP";

static adc_oneshot_unit_handle_t adc_handle_1 = NULL;
static adc_cali_handle_t cali_handle_1 = NULL;
static adc_oneshot_unit_handle_t adc_handle_2 = NULL;
static adc_cali_handle_t cali_handle_2 = NULL;

static float g_temp1 = 25.0f;
static float g_temp2 = 25.0f;
static portMUX_TYPE g_temp_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_temp_task_started = false;

static esp_err_t init_adc1_internal(void);
static esp_err_t init_adc2_internal(void);
static esp_err_t read_temperature(adc_oneshot_unit_handle_t handle,
                                  adc_cali_handle_t cali_handle,
                                  adc_channel_t channel,
                                  float *out_temp);
static void temperature_task(void *pvParameters);


// 初始化ADC1
void init_adc1()
{
    esp_err_t ret = init_adc1_internal();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 init failed: %s", esp_err_to_name(ret));
    }
}

// 读取温度（°C）
float get_temperature1()
{
    float temp;
    portENTER_CRITICAL(&g_temp_mux);
    temp = g_temp1;
    portEXIT_CRITICAL(&g_temp_mux);
    return temp;
}

// 释放ADC资源
void deinit_adc1() {
    if (cali_handle_1 != NULL) {
        adc_cali_delete_scheme_curve_fitting(cali_handle_1);
        cali_handle_1 = NULL;
    }
    if (adc_handle_1 != NULL) {
        adc_oneshot_del_unit(adc_handle_1);
        adc_handle_1 = NULL;
    }
}





 // ------- 通道 2：ADC2, 通道0（GPIO11） -------
void init_adc2()
{
    esp_err_t ret = init_adc2_internal();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC2 init failed: %s", esp_err_to_name(ret));
    }
}

// 读取树脂温度（°C）
float get_temperature2()
{
    float temp;
    portENTER_CRITICAL(&g_temp_mux);
    temp = g_temp2;
    portEXIT_CRITICAL(&g_temp_mux);
    return temp;
}

// 释放ADC资源
void deinit_adc2() {
    if (cali_handle_2 != NULL) {
        adc_cali_delete_scheme_curve_fitting(cali_handle_2);
        cali_handle_2 = NULL;
    }
    if (adc_handle_2 != NULL) {
        adc_oneshot_del_unit(adc_handle_2);
        adc_handle_2 = NULL;
    }
}

void temperature_task_start(void)
{
    if (g_temp_task_started) {
        return;
    }
    g_temp_task_started = true;
    xTaskCreatePinnedToCore(temperature_task, "temperature", 4096, NULL, 1, NULL, 1);
}

static esp_err_t init_adc1_internal(void)
{
    if (adc_handle_1 != NULL) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit1_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit1_config, &adc_handle_1);
    if (ret != ESP_OK) {
        return ret;
    }

    adc_oneshot_chan_cfg_t chan1_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle_1, ADC_CHANNEL_1, &chan1_config);
    if (ret != ESP_OK) {
        return ret;
    }

    adc_cali_curve_fitting_config_t cali1_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    return adc_cali_create_scheme_curve_fitting(&cali1_config, &cali_handle_1);
}

static esp_err_t init_adc2_internal(void)
{
    if (adc_handle_2 != NULL) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit2_config = {
        .unit_id = ADC_UNIT_2,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit2_config, &adc_handle_2);
    if (ret != ESP_OK) {
        return ret;
    }

    adc_oneshot_chan_cfg_t chan2_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ret = adc_oneshot_config_channel(adc_handle_2, ADC_CHANNEL_2, &chan2_config);
    if (ret != ESP_OK) {
        return ret;
    }

    adc_cali_curve_fitting_config_t cali2_config = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    return adc_cali_create_scheme_curve_fitting(&cali2_config, &cali_handle_2);
}

static esp_err_t read_temperature(adc_oneshot_unit_handle_t handle,
                                  adc_cali_handle_t cali_handle,
                                  adc_channel_t channel,
                                  float *out_temp)
{
    int raw_sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int raw;
        esp_err_t ret = adc_oneshot_read(handle, channel, &raw);
        if (ret != ESP_OK) {
            return ret;
        }
        raw_sum += raw;
    }
    int raw_avg = raw_sum / SAMPLE_COUNT;

    int voltage_mV;
    esp_err_t ret = adc_cali_raw_to_voltage(cali_handle, raw_avg, &voltage_mV);
    if (ret != ESP_OK) {
        return ret;
    }

    float Vout = voltage_mV / 1000.0f;
    float Rntc = (R1 * Vout) / (3.3f - Vout);

    float T0 = 25.0f + 273.15f;
    float ln_R = logf(Rntc / NTC_R25);
    float T = 1.0f / (1.0f / T0 + ln_R / NTC_BETA);

    *out_temp = T - 273.15f;
    return ESP_OK;
}

static void temperature_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (adc_handle_1 == NULL || cali_handle_1 == NULL) {
            esp_err_t ret = init_adc1_internal();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ADC1 init retry failed: %s", esp_err_to_name(ret));
            }
        }
        if (adc_handle_2 == NULL || cali_handle_2 == NULL) {
            esp_err_t ret = init_adc2_internal();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ADC2 init retry failed: %s", esp_err_to_name(ret));
            }
        }

        if (adc_handle_1 && cali_handle_1) {
            float temp;
            if (read_temperature(adc_handle_1, cali_handle_1, ADC_CHANNEL_1, &temp) == ESP_OK) {
                portENTER_CRITICAL(&g_temp_mux);
                g_temp1 = temp;
                portEXIT_CRITICAL(&g_temp_mux);
            }
        }

        if (adc_handle_2 && cali_handle_2) {
            float temp;
            if (read_temperature(adc_handle_2, cali_handle_2, ADC_CHANNEL_2, &temp) == ESP_OK) {
                portENTER_CRITICAL(&g_temp_mux);
                g_temp2 = temp;
                portEXIT_CRITICAL(&g_temp_mux);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}
