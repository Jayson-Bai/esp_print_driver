#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdio.h>
#include <math.h>

//ADC头文件
#include "esp_adc/adc_oneshot.h"      // 单次采样ADC驱动
#include "esp_adc/adc_cali.h"         // ADC校准
#include "esp_adc/adc_cali_scheme.h"  // 校准方案

// 纤维ADC配置
#define ADC_UNIT_1            ADC_UNIT_1        // 使用ADC1
#define ADC_CHANNEL_1         ADC_CHANNEL_5     // ADC1通道5（ESP32-S3对应GPIO 6） //---GPIO引脚

// 树脂ADC配置
#define ADC_UNIT_2            ADC_UNIT_2        // 使用ADC2
#define ADC_CHANNEL_2        ADC_CHANNEL_0     // ADC2通道0（ESP32-S3对应GPIO 11） //---GPIO引脚

// ADC通用配置
#define ADC_ATTEN           ADC_ATTEN_DB_12   // 输入衰减
#define ADC_BITWIDTH        ADC_BITWIDTH_12   // 12位分辨率

// 通用NTC热敏电阻参数
#define NTC_R25             100000.0    // 25°C时的电阻值（100KΩ）
#define NTC_BETA            3950.0      // B值（3950K）

// 通用分压电阻参数
#define R1                  10000.0     // 分压电阻阻值（10KΩ）

// 采样
#define SAMPLE_COUNT        64          // 采样次数（平均滤波）

void init_adc1();
void init_adc2();
float get_temperature1();
float get_temperature2();
void deinit_adc1();
void deinit_adc2();
void temperature_task_start(void);



#endif // TEMPERATURE_H
