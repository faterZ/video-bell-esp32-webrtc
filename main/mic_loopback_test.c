/**
 * 麦克风回音测试
 * 将麦克风录到的声音直接通过扬声器播放
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "codec_init.h"
#include "esp_codec_dev.h"
#include "esp_bit_defs.h"

static const char *TAG = "MIC_LOOPBACK";

#define BUFFER_SIZE 2048  // 缓冲区大小(字节)
#define LOOPBACK_MAX_ATTENUATION_SHIFT 2    // 动态衰减允许的最大位移
#define LOOPBACK_SATURATION_LOWER 10000      // 触发轻度衰减的阈值
#define LOOPBACK_SATURATION_UPPER 18000      // 触发重度衰减的阈值
#define LOOPBACK_TARGET_GAIN_DB 22.0f        // 回环测试时的麦克风增益
#define LOOPBACK_RESTORE_GAIN_DB 40.0f       // 回环结束后恢复增益
#define LOOPBACK_PLAYBACK_VOL 65             // 回环测试时的扬声器音量(0-100)
#define LOOPBACK_DELAY_FRAMES 4              // 播放前缓存的帧数,用于打破啸叫

static uint8_t *rx_buffer = NULL;  // 接收缓冲区(麦克风数据)
static uint8_t *tx_buffer = NULL;  // 发送缓冲区(扬声器数据)
static uint8_t *delay_buffer = NULL;  // 延时缓存队列
static bool loopback_running = false;
static esp_codec_dev_handle_t s_loopback_record = NULL;
static esp_codec_dev_handle_t s_loopback_playback = NULL;
static bool s_record_open = false;
static bool s_playback_open = false;
static size_t delay_write_idx = 0;
static size_t delay_read_idx = 0;
static size_t delay_count = 0;

/**
 * @brief 麦克风回音测试任务
 */
static void mic_loopback_task(void *arg)
{
    size_t bytes_read = 0;
    size_t bytes_written = 0;
    uint32_t loop_count = 0;
    uint32_t read_count = 0;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎤 Starting Microphone Loopback Test");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Speak into the microphone...");
    ESP_LOGI(TAG, "You should hear your voice from the speaker!");
    ESP_LOGI(TAG, "========================================");
    
    while (loopback_running) {
        if (s_loopback_record == NULL || s_loopback_playback == NULL) {
            ESP_LOGE(TAG, "Loopback handles invalid");
            break;
        }

        // 1. 从麦克风读取数据
        if (esp_codec_dev_read(s_loopback_record, rx_buffer, BUFFER_SIZE) != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "Failed to read from record device");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        bytes_read = BUFFER_SIZE;
        
        if (bytes_read > 0) {
            read_count++;

            int16_t *samples = (int16_t *)rx_buffer;
            int num_samples = bytes_read / sizeof(int16_t);

            // 2. 分析麦克风数据(仅前10次)
            if (read_count <= 10) {
                int16_t max_val = 0, min_val = 0;

                for (int i = 0; i < num_samples; i++) {
                    if (samples[i] > max_val) max_val = samples[i];
                    if (samples[i] < min_val) min_val = samples[i];
                }

                ESP_LOGI(TAG, "🎤 Read #%lu: Samples=%d, Range=[%d, %d]", 
                         read_count, num_samples, min_val, max_val);

                // 打印前8个样本
                if (read_count <= 3) {
                    ESP_LOGI(TAG, "   First 8 samples: %d %d %d %d %d %d %d %d",
                             samples[0], samples[1], samples[2], samples[3],
                             samples[4], samples[5], samples[6], samples[7]);
                }

                // 检查是否全0
                if (max_val == 0 && min_val == 0) {
                    ESP_LOGW(TAG, "   ⚠️  WARNING: Microphone data is all ZERO!");
                }
            }

            // 3. 延时缓冲：先尝试取出旧帧
            bool have_play_frame = false;
            if (delay_count == LOOPBACK_DELAY_FRAMES) {
                memcpy(tx_buffer, delay_buffer + (delay_read_idx * BUFFER_SIZE), bytes_read);
                delay_read_idx = (delay_read_idx + 1) % LOOPBACK_DELAY_FRAMES;
                delay_count--;
                have_play_frame = true;
            }

            // 将当前帧写入延时缓存
            memcpy(delay_buffer + (delay_write_idx * BUFFER_SIZE), rx_buffer, bytes_read);
            delay_write_idx = (delay_write_idx + 1) % LOOPBACK_DELAY_FRAMES;
            if (delay_count < LOOPBACK_DELAY_FRAMES) {
                delay_count++;
            }

            if (!have_play_frame) {
                if (read_count <= LOOPBACK_DELAY_FRAMES) {
                    ESP_LOGI(TAG, "⏳ Priming delay buffer (%u/%u frames)", (unsigned)delay_count, LOOPBACK_DELAY_FRAMES);
                }
                continue;
            }

            loop_count++;

            int16_t *play_samples = (int16_t *)tx_buffer;
            int16_t peak = 0;
            for (int i = 0; i < num_samples; ++i) {
                int16_t abs_val = (play_samples[i] >= 0) ? play_samples[i] : (int16_t)(-play_samples[i]);
                if (abs_val > peak) {
                    peak = abs_val;
                }
            }

            int attenuation_shift = 0;
            if (peak > LOOPBACK_SATURATION_UPPER) {
                attenuation_shift = 2;
            } else if (peak > LOOPBACK_SATURATION_LOWER) {
                attenuation_shift = 1;
            }
            if (attenuation_shift > LOOPBACK_MAX_ATTENUATION_SHIFT) {
                attenuation_shift = LOOPBACK_MAX_ATTENUATION_SHIFT;
            }

            if (attenuation_shift > 0) {
                const int attenuation_div = 1 << attenuation_shift;
                for (int i = 0; i < num_samples; ++i) {
                    play_samples[i] = (int16_t)(play_samples[i] / attenuation_div);
                }
                if (loop_count <= 5) {
                    ESP_LOGI(TAG, "   Applied attenuation shift=%d (peak=%d)", attenuation_shift, peak);
                }
            } else if (loop_count <= 3) {
                ESP_LOGI(TAG, "   No attenuation applied (peak=%d)", peak);
            }
            bytes_written = bytes_read;

            // 4. 发送到扬声器
            if (esp_codec_dev_write(s_loopback_playback, tx_buffer, bytes_written) != ESP_CODEC_DEV_OK) {
                ESP_LOGE(TAG, "Failed to write to playback device");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            
            if (loop_count <= 5) {
                ESP_LOGI(TAG, "🔊 Loop #%lu: Wrote %u bytes to speaker", 
                         loop_count, bytes_written);
            }
            
        } else {
            ESP_LOGE(TAG, "Failed to read from microphone!");
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // 短暂延迟,避免过载
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Microphone Loopback Test Stopped");
    ESP_LOGI(TAG, "Total loops: %lu", loop_count);
    ESP_LOGI(TAG, "========================================");
    
    vTaskDelete(NULL);
}

/**
 * @brief 启动麦克风回音测试
 */
void mic_loopback_start(void)
{
    if (loopback_running) {
        ESP_LOGW(TAG, "Loopback test already running!");
        return;
    }
    
    // 分配缓冲区
    rx_buffer = (uint8_t *)malloc(BUFFER_SIZE);
    tx_buffer = (uint8_t *)malloc(BUFFER_SIZE);
    delay_buffer = (uint8_t *)malloc(BUFFER_SIZE * LOOPBACK_DELAY_FRAMES);
    
    if (rx_buffer == NULL || tx_buffer == NULL || delay_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffers!");
        if (rx_buffer) free(rx_buffer);
        if (tx_buffer) free(tx_buffer);
        if (delay_buffer) free(delay_buffer);
        return;
    }
    
    memset(rx_buffer, 0, BUFFER_SIZE);
    memset(tx_buffer, 0, BUFFER_SIZE);
    memset(delay_buffer, 0, BUFFER_SIZE * LOOPBACK_DELAY_FRAMES);
    delay_write_idx = 0;
    delay_read_idx = 0;
    delay_count = 0;
    
    s_loopback_record = get_record_handle();
    s_loopback_playback = get_playback_handle();
    if (s_loopback_record == NULL || s_loopback_playback == NULL) {
        ESP_LOGE(TAG, "Codec handles not ready");
        free(rx_buffer);
        free(tx_buffer);
        rx_buffer = tx_buffer = NULL;
        return;
    }

    esp_codec_dev_sample_info_t record_fs = {
        .sample_rate = 24000,
        .channel = 1,
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
        .bits_per_sample = 16,
        .mclk_multiple = 0,
    };
    esp_codec_dev_sample_info_t playback_fs = {
        .sample_rate = 24000,
        .channel = 1,
        .channel_mask = 0,
        .bits_per_sample = 16,
        .mclk_multiple = 0,
    };

    int ret = esp_codec_dev_open(s_loopback_record, &record_fs);
    if (ret == ESP_CODEC_DEV_OK) {
        s_record_open = true;
    } else if (ret != ESP_CODEC_DEV_WRONG_STATE) {
        ESP_LOGE(TAG, "Failed to open record dev: %d", ret);
        goto cleanup;
    }
    int gain_ret = esp_codec_dev_set_in_channel_gain(s_loopback_record, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), LOOPBACK_TARGET_GAIN_DB);
    if (gain_ret != ESP_CODEC_DEV_OK) {
        gain_ret = esp_codec_dev_set_in_gain(s_loopback_record, LOOPBACK_TARGET_GAIN_DB);
        if (gain_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "Failed to set input gain: %d", gain_ret);
        }
    }

    ret = esp_codec_dev_open(s_loopback_playback, &playback_fs);
    if (ret == ESP_CODEC_DEV_OK) {
        s_playback_open = true;
    } else if (ret != ESP_CODEC_DEV_WRONG_STATE) {
        ESP_LOGE(TAG, "Failed to open playback dev: %d", ret);
        goto cleanup;
    }
    esp_codec_dev_set_out_vol(s_loopback_playback, LOOPBACK_PLAYBACK_VOL);
    esp_codec_dev_set_out_mute(s_loopback_playback, false);

    ESP_LOGI(TAG, "Loopback buffers allocated (%d bytes capture)", BUFFER_SIZE);

    loopback_running = true;

    // 创建回音测试任务
    xTaskCreate(mic_loopback_task, "mic_loopback", 4096, NULL, 5, NULL);
    return;

cleanup:
    if (rx_buffer) {
        free(rx_buffer);
        rx_buffer = NULL;
    }
    if (tx_buffer) {
        free(tx_buffer);
        tx_buffer = NULL;
    }
    loopback_running = false;
}

/**
 * @brief 停止麦克风回音测试
 */
void mic_loopback_stop(void)
{
    if (!loopback_running) {
        ESP_LOGW(TAG, "Loopback test not running!");
        return;
    }
    
    loopback_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待任务结束
    
    if (s_loopback_record && s_record_open) {
        int restore_ret = esp_codec_dev_set_in_channel_gain(s_loopback_record, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), LOOPBACK_RESTORE_GAIN_DB);
        if (restore_ret != ESP_CODEC_DEV_OK) {
            esp_codec_dev_set_in_gain(s_loopback_record, LOOPBACK_RESTORE_GAIN_DB);
        }
        esp_codec_dev_close(s_loopback_record);
        s_record_open = false;
    }
    if (s_playback_open && s_loopback_playback) {
        esp_codec_dev_close(s_loopback_playback);
        s_playback_open = false;
    }

    // 释放缓冲区
    if (rx_buffer) {
        free(rx_buffer);
        rx_buffer = NULL;
    }
    if (tx_buffer) {
        free(tx_buffer);
        tx_buffer = NULL;
    }
    if (delay_buffer) {
        free(delay_buffer);
        delay_buffer = NULL;
    }
    
    ESP_LOGI(TAG, "Loopback test stopped and buffers freed");
}
