/* 舵机控制模块 - 用于云台追踪
 * 支持双轴云台（水平YAW + 俯仰PITCH）
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/ledc.h"
#include "esp_err.h"

// 舵机GPIO配置（根据实际硬件调整）
// 注意：这里使用的GPIO需要根据实际板子的IO排针确认
#define SERVO_YAW_GPIO      4   // 水平舵机（需要根据实际IO排针GPIO调整）
#define SERVO_PITCH_GPIO    5   // 俯仰舵机（需要根据实际IO排针GPIO调整）

// SG90舵机PWM参数
#define SERVO_FREQ_HZ           50      // 50Hz (20ms周期)
#define SERVO_MIN_PULSE_US      500     // 0°对应0.5ms脉宽
#define SERVO_MAX_PULSE_US      2400    // 180°对应2.4ms脉宽
#define SERVO_TIMEBASE_US       20000   // 20ms

// 云台角度限制（防止机械碰撞）
#define SERVO_YAW_MIN_ANGLE     30.0f   // 水平最小角度
#define SERVO_YAW_MAX_ANGLE     150.0f  // 水平最大角度
#define SERVO_PITCH_MIN_ANGLE   45.0f   // 俯仰最小角度（防止过低）
#define SERVO_PITCH_MAX_ANGLE   135.0f  // 俯仰最大角度（防止过高）

// 云台初始位置（中心）
#define SERVO_YAW_CENTER        90.0f
#define SERVO_PITCH_CENTER      90.0f

/**
 * @brief 初始化舵机控制系统
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t servo_init(void);

/**
 * @brief 设置单个舵机角度
 * @param channel LEDC通道（LEDC_CHANNEL_0=YAW, LEDC_CHANNEL_1=PITCH）
 * @param angle 角度 0-180°
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t servo_set_angle(ledc_channel_t channel, float angle);

/**
 * @brief 设置云台位置（简化接口）
 * @param yaw 水平角度 0-180°（90°为中心）
 * @param pitch 俯仰角度 0-180°（90°为中心）
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t servo_set_gimbal_position(float yaw, float pitch);

/**
 * @brief 云台复位到中心位置
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t servo_reset_gimbal(void);

/**
 * @brief 测试舵机功能（水平和俯仰扫描）
 */
void servo_test_scan(void);

#ifdef __cplusplus
}
#endif
