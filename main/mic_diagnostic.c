/**
 * 麦克风诊断工具 - 简化版
 * 直接测试当前配置，不重新配置 I2S（避免死锁）
 */

#include <limits.h>
#include <stdlib.h>

#include "mic_diagnostic.h"
#include "codec_init.h"
#include "esp_codec_dev.h"
#include "esp_log.h"
#include "esp_bit_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MIC_DIAG";

/**
 * 运行完整的麦克风诊断 - 简化版
 * 只测试当前配置，不重新配置 I2S
 */
void mic_diagnostic_run(void) {
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   🔬 MICROPHONE DIAGNOSTIC TEST       ║");
    ESP_LOGI(TAG, "║   Testing current I2S configuration   ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TAG, "📝 Using esp_codec_dev read path");

    esp_codec_dev_handle_t record_dev = get_record_handle();
    if (record_dev == NULL) {
        ESP_LOGE(TAG, "❌ Record handle not ready");
        return;
    }

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 1,
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
        .bits_per_sample = 16,
        .mclk_multiple = 0,
    };
    bool record_opened_here = false;
    int ret = esp_codec_dev_open(record_dev, &fs);
    if (ret == ESP_CODEC_DEV_OK) {
        record_opened_here = true;
        int gain_ret = esp_codec_dev_set_in_gain(record_dev, 40.0f);
        if (gain_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG, "⚠️  Failed to raise input gain: %d", gain_ret);
        }
    } else if (ret != ESP_CODEC_DEV_WRONG_STATE) {
        ESP_LOGE(TAG, "❌ Failed to open record dev: %d", ret);
        return;
    }
    
    // 读取多次数据并分析
    uint8_t *buffer = (uint8_t *)malloc(2048);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "❌ Failed to allocate buffer!");
        return;
    }
    
    int total_samples = 0;
    int non_zero_samples = 0;
    int16_t max_val = INT16_MIN;
    int16_t min_val = INT16_MAX;
    
    ESP_LOGI(TAG, "🎤 Reading microphone data (10 samples)...");
    
    for (int loop = 0; loop < 10; loop++) {
        ESP_LOGI(TAG, "   📖 Reading sample #%d...", loop + 1);
        
        int ret = esp_codec_dev_read(record_dev, buffer, 2048);
        if (ret != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "   ❌ Read failed: %d", ret);
            continue;
        }
        
        int16_t *samples = (int16_t *)buffer;
        int num_samples = 2048 / 2;
        int loop_non_zero = 0;
        int16_t loop_max = INT16_MIN;
        int16_t loop_min = INT16_MAX;

        for (int j = 0; j < num_samples; j++) {
            total_samples++;
            int16_t value = samples[j];

            if (value != 0) {
                non_zero_samples++;
                loop_non_zero++;
                if (value > max_val) max_val = value;
                if (value < min_val) min_val = value;
                if (value > loop_max) loop_max = value;
                if (value < loop_min) loop_min = value;
            }
        }

        int16_t log_min = loop_non_zero ? loop_min : 0;
        int16_t log_max = loop_non_zero ? loop_max : 0;
        ESP_LOGI(TAG, "   ✅ Sample #%d: range=[%d, %d]",
                 loop + 1, log_min, log_max);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    free(buffer);
    
    float non_zero_percent = total_samples > 0 ? (float)non_zero_samples / total_samples * 100.0f : 0;
    int16_t summary_min = non_zero_samples ? min_val : 0;
    int16_t summary_max = non_zero_samples ? max_val : 0;
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   📊 DIAGNOSTIC RESULTS               ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📈 Overall Statistics:");
    ESP_LOGI(TAG, "   Total samples: %d", total_samples);
    ESP_LOGI(TAG, "   Non-zero samples: %d (%.1f%%)", non_zero_samples, non_zero_percent);
    ESP_LOGI(TAG, "   Data range: [%d, %d]", summary_min, summary_max);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "📊 Channel:");
    ESP_LOGI(TAG, "   Non-zero: %d (%.1f%%)", non_zero_samples, non_zero_percent);
    ESP_LOGI(TAG, "   Range: [%d, %d]", summary_min, summary_max);
    ESP_LOGI(TAG, "");
    
    if (non_zero_samples == 0) {
        ESP_LOGE(TAG, "❌ CRITICAL: NO DATA FROM MICROPHONE!");
        ESP_LOGE(TAG, "   Possible issues:");
        ESP_LOGE(TAG, "   1. GPIO47 not connected to MSM261");
        ESP_LOGE(TAG, "   2. MSM261 not powered");
        ESP_LOGE(TAG, "   3. I2S clock not working");
        ESP_LOGE(TAG, "   4. Wrong I2S configuration");
    } else if (non_zero_percent < 1) {
        ESP_LOGW(TAG, "⚠️  VERY WEAK SIGNAL (%.1f%%)", non_zero_percent);
        ESP_LOGW(TAG, "   Microphone might be working but signal is too weak");
    }
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "✅ Diagnostic complete!");
    ESP_LOGI(TAG, "");

    if (record_opened_here) {
        esp_codec_dev_close(record_dev);
    }
}
