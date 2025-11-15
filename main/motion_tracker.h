/* 运动物体追踪模块
 * 使用简单帧差法检测运动并控制云台追踪
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// 追踪器配置
typedef struct {
    uint16_t frame_width;       // 图像宽度
    uint16_t frame_height;      // 图像高度
    uint8_t motion_threshold;   // 运动检测阈值（0-255）
    float pid_kp;               // PID比例系数
    float pid_ki;               // PID积分系数
    float pid_kd;               // PID微分系数
    float max_speed;            // 最大云台移动速度（度/秒）
} tracker_config_t;

// 默认配置
#define DEFAULT_MOTION_THRESHOLD    30
#define DEFAULT_PID_KP              0.5f
#define DEFAULT_PID_KI              0.0f
#define DEFAULT_PID_KD              0.1f
#define DEFAULT_MAX_SPEED           20.0f

// 追踪目标信息
typedef struct {
    bool detected;              // 是否检测到目标
    int16_t center_x;           // 目标中心X坐标
    int16_t center_y;           // 目标中心Y坐标
    uint32_t area;              // 运动区域面积
    uint32_t timestamp;         // 检测时间戳
} motion_target_t;

/**
 * @brief 初始化运动追踪器
 * @param config 追踪器配置
 * @return ESP_OK 成功, ESP_FAIL 失败
 */
esp_err_t motion_tracker_init(tracker_config_t *config);

/**
 * @brief 处理新帧并检测运动
 * @param frame 图像帧数据（RGB888或灰度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param channels 通道数（1=灰度, 2=RGB565, 3=RGB888）
 * @param target 输出的目标信息
 * @return ESP_OK 成功
 */
esp_err_t motion_tracker_process_frame(uint8_t *frame, uint16_t width, 
                                      uint16_t height, uint8_t channels,
                                      motion_target_t *target);

/**
 * @brief 更新云台位置以跟踪目标
 * @param target 目标信息
 * @return ESP_OK 成功
 */
esp_err_t motion_tracker_update_gimbal(motion_target_t *target);

/**
 * @brief 启动追踪器
 * @return ESP_OK 成功
 */
esp_err_t motion_tracker_start(void);

/**
 * @brief 停止追踪器
 */
void motion_tracker_stop(void);

/**
 * @brief 检查追踪器是否运行
 * @return true 运行中, false 已停止
 */
bool motion_tracker_is_running(void);

/**
 * @brief 复位追踪器（清除历史帧和PID状态）
 */
void motion_tracker_reset(void);

#ifdef __cplusplus
}
#endif
