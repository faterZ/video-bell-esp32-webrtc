/**
 * 正点原子音频设备适配层头文件
 */

#ifndef AUDIO_DEVICE_H
#define AUDIO_DEVICE_H

#include "esp_err.h"
#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化音频设备
 * 
 * @return ESP_OK 成功
 * @return ESP_FAIL 失败
 */
esp_err_t audio_device_init(void);

/**
 * @brief 获取播放设备句柄
 * 
 * @return esp_codec_dev_handle_t 播放设备句柄
 */
esp_codec_dev_handle_t audio_get_playback_handle(void);

/**
 * @brief 获取录音设备句柄
 * 
 * @return esp_codec_dev_handle_t 录音设备句柄
 */
esp_codec_dev_handle_t audio_get_record_handle(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_DEVICE_H
