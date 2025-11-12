/* 运动物体追踪模块实现
 * 使用简单帧差法检测运动并控制云台追踪
 */

#include "motion_tracker.h"
#include "servo_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "TRACKER";

// 追踪器状态
typedef struct {
    tracker_config_t config;
    uint8_t *prev_frame;        // 前一帧（灰度）
    uint8_t *motion_mask;       // 运动掩码
    bool initialized;
    bool running;
    
    // PID控制器状态
    float error_yaw_prev;
    float error_pitch_prev;
    float integral_yaw;
    float integral_pitch;
    
    // 当前云台角度
    float current_yaw;
    float current_pitch;
} tracker_state_t;

static tracker_state_t tracker = {0};

/**
 * @brief 将RGB转换为灰度
 */
static inline uint8_t rgb_to_gray(uint8_t r, uint8_t g, uint8_t b)
{
    // 加权平均法：Gray = 0.299*R + 0.587*G + 0.114*B
    return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

/**
 * @brief 初始化运动追踪器
 */
esp_err_t motion_tracker_init(tracker_config_t *config)
{
    if (tracker.initialized) {
        ESP_LOGW(TAG, "Tracker already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎯 Initializing motion tracker...");
    ESP_LOGI(TAG, "========================================");
    
    // 保存配置
    memcpy(&tracker.config, config, sizeof(tracker_config_t));
    
    // 分配前一帧缓冲区（灰度）
    size_t frame_size = config->frame_width * config->frame_height;
    tracker.prev_frame = (uint8_t*)malloc(frame_size);
    if (!tracker.prev_frame) {
        ESP_LOGE(TAG, "❌ Failed to allocate prev_frame buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(tracker.prev_frame, 0, frame_size);
    
    // 分配运动掩码缓冲区
    tracker.motion_mask = (uint8_t*)malloc(frame_size);
    if (!tracker.motion_mask) {
        ESP_LOGE(TAG, "❌ Failed to allocate motion_mask buffer");
        free(tracker.prev_frame);
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化PID控制器
    tracker.error_yaw_prev = 0;
    tracker.error_pitch_prev = 0;
    tracker.integral_yaw = 0;
    tracker.integral_pitch = 0;
    
    // 初始化云台角度
    tracker.current_yaw = SERVO_YAW_CENTER;
    tracker.current_pitch = SERVO_PITCH_CENTER;
    
    tracker.initialized = true;
    tracker.running = false;
    
    ESP_LOGI(TAG, "✅ Motion tracker initialized");
    ESP_LOGI(TAG, "   Frame size: %dx%d", config->frame_width, config->frame_height);
    ESP_LOGI(TAG, "   Motion threshold: %d", config->motion_threshold);
    ESP_LOGI(TAG, "   PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
             config->pid_kp, config->pid_ki, config->pid_kd);
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}

/**
 * @brief 处理新帧并检测运动
 */
esp_err_t motion_tracker_process_frame(uint8_t *frame, uint16_t width, 
                                      uint16_t height, uint8_t channels,
                                      motion_target_t *target)
{
    if (!tracker.initialized || !tracker.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (width != tracker.config.frame_width || height != tracker.config.frame_height) {
        ESP_LOGE(TAG, "Frame size mismatch: expected %dx%d, got %dx%d",
                 tracker.config.frame_width, tracker.config.frame_height, width, height);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 转换为灰度（如果是彩色）
    uint8_t *gray_frame = NULL;
    if (channels == 3) {
        gray_frame = (uint8_t*)malloc(width * height);
        if (!gray_frame) {
            ESP_LOGE(TAG, "Failed to allocate gray_frame");
            return ESP_ERR_NO_MEM;
        }
        
        for (int i = 0; i < width * height; i++) {
            gray_frame[i] = rgb_to_gray(frame[i*3], frame[i*3+1], frame[i*3+2]);
        }
    } else {
        gray_frame = frame; // 已经是灰度
    }
    
    // 计算帧差并检测运动
    uint32_t motion_sum_x = 0;
    uint32_t motion_sum_y = 0;
    uint32_t motion_count = 0;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            
            // 计算像素差异
            int diff = abs(gray_frame[idx] - tracker.prev_frame[idx]);
            
            // 运动检测
            if (diff > tracker.config.motion_threshold) {
                tracker.motion_mask[idx] = 255;
                motion_sum_x += x;
                motion_sum_y += y;
                motion_count++;
            } else {
                tracker.motion_mask[idx] = 0;
            }
        }
    }
    
    // 更新前一帧
    memcpy(tracker.prev_frame, gray_frame, width * height);
    
    // 释放临时缓冲区
    if (channels == 3) {
        free(gray_frame);
    }
    
    // 计算运动中心
    if (motion_count > 100) { // 至少100个像素才认为是有效运动
        target->detected = true;
        target->center_x = motion_sum_x / motion_count;
        target->center_y = motion_sum_y / motion_count;
        target->area = motion_count;
        target->timestamp = xTaskGetTickCount();
        
        ESP_LOGD(TAG, "🎯 Motion detected at (%d, %d), area=%lu", 
                 target->center_x, target->center_y, target->area);
    } else {
        target->detected = false;
    }
    
    return ESP_OK;
}

/**
 * @brief PID控制器计算输出
 */
static float pid_compute(float error, float *error_prev, float *integral, 
                        float kp, float ki, float kd, float dt)
{
    // 比例项
    float p_term = kp * error;
    
    // 积分项（防止积分饱和）
    *integral += error * dt;
    if (*integral > 50.0f) *integral = 50.0f;
    if (*integral < -50.0f) *integral = -50.0f;
    float i_term = ki * (*integral);
    
    // 微分项
    float derivative = (error - *error_prev) / dt;
    float d_term = kd * derivative;
    
    *error_prev = error;
    
    return p_term + i_term + d_term;
}

/**
 * @brief 更新云台位置以跟踪目标
 */
esp_err_t motion_tracker_update_gimbal(motion_target_t *target)
{
    if (!tracker.initialized || !tracker.running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!target->detected) {
        // 没有目标，保持当前位置
        return ESP_OK;
    }
    
    // 计算目标偏移（像素坐标转换为误差）
    float center_x = tracker.config.frame_width / 2.0f;
    float center_y = tracker.config.frame_height / 2.0f;
    
    float error_x = (target->center_x - center_x) / center_x; // 归一化到[-1, 1]
    float error_y = (target->center_y - center_y) / center_y;
    
    // PID控制计算（假设dt=0.1秒）
    float dt = 0.1f;
    float control_yaw = pid_compute(error_x, &tracker.error_yaw_prev, &tracker.integral_yaw,
                                   tracker.config.pid_kp, tracker.config.pid_ki, 
                                   tracker.config.pid_kd, dt);
    float control_pitch = pid_compute(error_y, &tracker.error_pitch_prev, &tracker.integral_pitch,
                                     tracker.config.pid_kp, tracker.config.pid_ki,
                                     tracker.config.pid_kd, dt);
    
    // 限制速度
    if (control_yaw > tracker.config.max_speed) control_yaw = tracker.config.max_speed;
    if (control_yaw < -tracker.config.max_speed) control_yaw = -tracker.config.max_speed;
    if (control_pitch > tracker.config.max_speed) control_pitch = tracker.config.max_speed;
    if (control_pitch < -tracker.config.max_speed) control_pitch = -tracker.config.max_speed;
    
    // 更新云台角度
    tracker.current_yaw += control_yaw * dt;
    tracker.current_pitch -= control_pitch * dt; // Y轴方向相反
    
    // 限制角度范围
    if (tracker.current_yaw < SERVO_YAW_MIN_ANGLE) tracker.current_yaw = SERVO_YAW_MIN_ANGLE;
    if (tracker.current_yaw > SERVO_YAW_MAX_ANGLE) tracker.current_yaw = SERVO_YAW_MAX_ANGLE;
    if (tracker.current_pitch < SERVO_PITCH_MIN_ANGLE) tracker.current_pitch = SERVO_PITCH_MIN_ANGLE;
    if (tracker.current_pitch > SERVO_PITCH_MAX_ANGLE) tracker.current_pitch = SERVO_PITCH_MAX_ANGLE;
    
    // 控制舵机
    esp_err_t ret = servo_set_gimbal_position(tracker.current_yaw, tracker.current_pitch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update gimbal position");
        return ret;
    }
    
    ESP_LOGI(TAG, "🎯 Tracking: error=(%.2f, %.2f), gimbal=(%.1f°, %.1f°)", 
             error_x, error_y, tracker.current_yaw, tracker.current_pitch);
    
    return ESP_OK;
}

/**
 * @brief 启动追踪器
 */
esp_err_t motion_tracker_start(void)
{
    if (!tracker.initialized) {
        ESP_LOGE(TAG, "Tracker not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (tracker.running) {
        ESP_LOGW(TAG, "Tracker already running");
        return ESP_OK;
    }
    
    tracker.running = true;
    ESP_LOGI(TAG, "▶️  Motion tracker started");
    
    return ESP_OK;
}

/**
 * @brief 停止追踪器
 */
void motion_tracker_stop(void)
{
    if (tracker.running) {
        tracker.running = false;
        ESP_LOGI(TAG, "⏸️  Motion tracker stopped");
    }
}

/**
 * @brief 检查追踪器是否运行
 */
bool motion_tracker_is_running(void)
{
    return tracker.running;
}

/**
 * @brief 复位追踪器
 */
void motion_tracker_reset(void)
{
    if (!tracker.initialized) return;
    
    ESP_LOGI(TAG, "🔄 Resetting tracker...");
    
    // 清除前一帧和运动掩码
    size_t frame_size = tracker.config.frame_width * tracker.config.frame_height;
    memset(tracker.prev_frame, 0, frame_size);
    memset(tracker.motion_mask, 0, frame_size);
    
    // 重置PID状态
    tracker.error_yaw_prev = 0;
    tracker.error_pitch_prev = 0;
    tracker.integral_yaw = 0;
    tracker.integral_pitch = 0;
    
    // 重置云台到中心
    tracker.current_yaw = SERVO_YAW_CENTER;
    tracker.current_pitch = SERVO_PITCH_CENTER;
    servo_reset_gimbal();
    
    ESP_LOGI(TAG, "✅ Tracker reset completed");
}
