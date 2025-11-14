/**
 * ES8311 Audio Codec Driver
 * 简化版ES8311驱动,仅支持基本的播放功能
 */

#ifndef __ES8311_H__
#define __ES8311_H__

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ES8311 I2C地址
#define ES8311_ADDR 0x18  // 7-bit地址

/**
 * @brief 初始化ES8311 Codec
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_init(void);

/**
 * @brief 配置ES8311为DAC模式(播放)
 * @param sample_rate 采样率(8000, 16000, 44100, 48000等)
 * @param bits_per_sample 位宽(16, 24, 32)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_config_dac(uint32_t sample_rate, uint8_t bits_per_sample);

/**
 * @brief 设置DAC音量
 * @param volume 音量(0-100)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_set_voice_volume(uint8_t volume);

/**
 * @brief 控制DAC静音
 * @param mute true=静音, false=取消静音
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_set_voice_mute(bool mute);

/**
 * @brief 配置ES8311为ADC模式(录音)
 * @param sample_rate 采样率(8000, 16000, 44100, 48000等)
 * @param bits_per_sample 位宽(16, 24, 32)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_config_adc(uint32_t sample_rate, uint8_t bits_per_sample);

/**
 * @brief 设置ADC输入增益
 * @param gain_db 增益(0-24 dB, 步进3dB)
 * @return ESP_OK 成功, 其他值失败
 */
esp_err_t es8311_codec_set_mic_gain(uint8_t gain_db);

#ifdef __cplusplus
}
#endif

#endif /* __ES8311_H__ */
