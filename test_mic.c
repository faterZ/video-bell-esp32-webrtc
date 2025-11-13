/**
 * 麦克风硬件测试程序
 * 用途: 验证正点原子 ESP32-S3 BOX 的麦克风 MSM261S4030H0R 是否正常工作
 * 
 * 编译方法:
 * 1. 将此文件复制到 main/ 目录,替换 main.c
 * 2. idf.py build flash monitor
 * 
 * 预期结果:
 * - 如果麦克风正常: 对着麦克风说话,串口应该打印非零数据
 * - 如果麦克风异常: 数据全 0,range=[0, 0]
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char *TAG = "MIC_TEST";

// I2S 配置 (与官方录音样例一致)
#define I2S_NUM             I2S_NUM_0
#define I2S_BCK_IO          GPIO_NUM_21
#define I2S_WS_IO           GPIO_NUM_13
#define I2S_DO_IO           GPIO_NUM_14  // 扬声器 (不用)
#define I2S_DI_IO           GPIO_NUM_47  // 麦克风
#define SAMPLE_RATE         16000
#define BITS_PER_SAMPLE     I2S_DATA_BIT_WIDTH_16BIT
#define BUFFER_SIZE         1024

i2s_chan_handle_t rx_handle = NULL;

/**
 * 初始化 I2S (录音模式)
 */
esp_err_t mic_test_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎤 Microphone Hardware Test");
    ESP_LOGI(TAG, "========================================");
    
    // 1. 创建 I2S 通道
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));
    ESP_LOGI(TAG, "✅ I2S channel created");
    
    // 2. 配置 I2S 标准模式
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = 256,
        },
        .slot_cfg = {
            .data_bit_width = BITS_PER_SAMPLE,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = BITS_PER_SAMPLE,
            .ws_pol = false,        // 与官方样例一致
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = GPIO_NUM_NC,    // 只录音,不播放
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_LOGI(TAG, "✅ I2S configured: %d Hz, 16-bit, GPIO47", SAMPLE_RATE);
    
    // 3. 启用 I2S 通道
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    ESP_LOGI(TAG, "✅ I2S channel enabled");
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎤 Please speak into the microphone...");
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}

/**
 * 麦克风测试任务
 */
void mic_test_task(void *arg)
{
    uint8_t *buffer = malloc(BUFFER_SIZE);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        vTaskDelete(NULL);
        return;
    }
    
    size_t bytes_read = 0;
    int read_count = 0;
    
    while (1) {
        // 读取麦克风数据
        esp_err_t ret = i2s_channel_read(rx_handle, buffer, BUFFER_SIZE, &bytes_read, 1000);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ I2S read failed: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        read_count++;
        
        // 分析数据
        int16_t *samples = (int16_t *)buffer;
        int num_samples = bytes_read / 2;
        int16_t max_val = 0, min_val = 0;
        int32_t sum = 0;
        
        for (int i = 0; i < num_samples; i++) {
            if (samples[i] > max_val) max_val = samples[i];
            if (samples[i] < min_val) min_val = samples[i];
            sum += abs(samples[i]);
        }
        
        int16_t avg = sum / num_samples;
        
        // 打印结果
        ESP_LOGI(TAG, "📊 Read #%d: bytes=%u, samples=%d, range=[%d, %d], avg=%d",
                 read_count, bytes_read, num_samples, min_val, max_val, avg);
        
        // 前10次打印详细数据
        if (read_count <= 10) {
            ESP_LOGI(TAG, "   First 8 samples: %d %d %d %d %d %d %d %d",
                     samples[0], samples[1], samples[2], samples[3],
                     samples[4], samples[5], samples[6], samples[7]);
        }
        
        // 警告:数据全0
        if (max_val == 0 && min_val == 0) {
            ESP_LOGW(TAG, "⚠️  WARNING: All samples are ZERO! Microphone may not be working.");
        }
        
        // 每秒读取一次
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    free(buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // 初始化麦克风
    mic_test_init();
    
    // 创建测试任务
    xTaskCreate(mic_test_task, "mic_test", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "✅ Microphone test started!");
    ESP_LOGI(TAG, "   Expected: Non-zero data when speaking");
    ESP_LOGI(TAG, "   Problem: All zeros = hardware issue");
}
