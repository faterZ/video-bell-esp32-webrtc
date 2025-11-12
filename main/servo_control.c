/* 舵机控制模块实现 */

#include "servo_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERVO";

// 当前云台位置记录
static float current_yaw = SERVO_YAW_CENTER;
static float current_pitch = SERVO_PITCH_CENTER;
static bool servo_initialized = false;

/**
 * @brief 初始化舵机控制系统
 */
esp_err_t servo_init(void)
{
    if (servo_initialized) {
        ESP_LOGW(TAG, "Servo already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎬 Initializing servo control...");
    ESP_LOGI(TAG, "========================================");
    
    // 配置PWM定时器（50Hz）
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,  // 使用TIMER_1避免与USB冲突
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to config LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ LEDC timer configured (50Hz)");
    
    // 配置水平舵机（YAW）通道
    ledc_channel_config_t yaw_channel = {
        .gpio_num = SERVO_YAW_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    
    ret = ledc_channel_config(&yaw_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to config YAW channel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ YAW servo configured (GPIO%d)", SERVO_YAW_GPIO);
    
    // 配置俯仰舵机（PITCH）通道
    ledc_channel_config_t pitch_channel = {
        .gpio_num = SERVO_PITCH_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    
    ret = ledc_channel_config(&pitch_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to config PITCH channel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ PITCH servo configured (GPIO%d)", SERVO_PITCH_GPIO);
    
    // 初始化到中心位置
    servo_reset_gimbal();
    
    servo_initialized = true;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ Servo control initialized!");
    ESP_LOGI(TAG, "   YAW: %.1f°, PITCH: %.1f°", current_yaw, current_pitch);
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}

/**
 * @brief 设置单个舵机角度
 */
esp_err_t servo_set_angle(ledc_channel_t channel, float angle)
{
    if (!servo_initialized) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_FAIL;
    }
    
    // 限制角度范围
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 计算脉宽（us）
    uint32_t pulse_width_us = SERVO_MIN_PULSE_US + 
        (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    
    // 转换为占空比（14位分辨率）
    uint32_t duty = (pulse_width_us * ((1 << LEDC_TIMER_14_BIT) - 1)) / SERVO_TIMEBASE_US;
    
    // 设置PWM
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGD(TAG, "Servo CH%d: %.1f° (pulse=%luus, duty=%lu)", 
             channel, angle, pulse_width_us, duty);
    
    return ESP_OK;
}

/**
 * @brief 设置云台位置
 */
esp_err_t servo_set_gimbal_position(float yaw, float pitch)
{
    // 限制角度范围
    if (yaw < SERVO_YAW_MIN_ANGLE) yaw = SERVO_YAW_MIN_ANGLE;
    if (yaw > SERVO_YAW_MAX_ANGLE) yaw = SERVO_YAW_MAX_ANGLE;
    if (pitch < SERVO_PITCH_MIN_ANGLE) pitch = SERVO_PITCH_MIN_ANGLE;
    if (pitch > SERVO_PITCH_MAX_ANGLE) pitch = SERVO_PITCH_MAX_ANGLE;
    
    esp_err_t ret = servo_set_angle(LEDC_CHANNEL_0, yaw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = servo_set_angle(LEDC_CHANNEL_1, pitch);
    if (ret != ESP_OK) {
        return ret;
    }
    
    current_yaw = yaw;
    current_pitch = pitch;
    
    ESP_LOGI(TAG, "🎯 Gimbal: YAW=%.1f° PITCH=%.1f°", yaw, pitch);
    
    return ESP_OK;
}

/**
 * @brief 云台复位到中心位置
 */
esp_err_t servo_reset_gimbal(void)
{
    ESP_LOGI(TAG, "🔄 Resetting gimbal to center...");
    return servo_set_gimbal_position(SERVO_YAW_CENTER, SERVO_PITCH_CENTER);
}

/**
 * @brief 测试舵机功能
 */
void servo_test_scan(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🧪 Starting servo test scan...");
    ESP_LOGI(TAG, "========================================");
    
    // 水平扫描
    ESP_LOGI(TAG, "Testing YAW (horizontal) scan...");
    for (float yaw = SERVO_YAW_MIN_ANGLE; yaw <= SERVO_YAW_MAX_ANGLE; yaw += 10) {
        servo_set_gimbal_position(yaw, SERVO_PITCH_CENTER);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // 回中心
    servo_reset_gimbal();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 俯仰扫描
    ESP_LOGI(TAG, "Testing PITCH (vertical) scan...");
    for (float pitch = SERVO_PITCH_MIN_ANGLE; pitch <= SERVO_PITCH_MAX_ANGLE; pitch += 10) {
        servo_set_gimbal_position(SERVO_YAW_CENTER, pitch);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // 回中心
    servo_reset_gimbal();
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ Servo test completed!");
    ESP_LOGI(TAG, "========================================");
}
