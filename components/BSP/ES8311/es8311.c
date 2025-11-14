/**
 * ES8311 Audio Codec Driver
 * 简化版ES8311驱动,仅支持基本的播放功能
 * 参考: ESP-IDF esp_codec_dev 组件中的 ES8311 驱动
 */

#include "es8311.h"
#include "myiic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <string.h>

static const char *TAG = "ES8311";

// I2C设备句柄
static i2c_master_dev_handle_t es8311_handle = NULL;

// ES8311寄存器地址
#define ES8311_REG00_RESET          0x00  // Reset control
#define ES8311_REG01_CLK_MANAGER    0x01  // Clock Manager
#define ES8311_REG02_CLK_MANAGER    0x02  // Clock Manager
#define ES8311_REG03_CLK_MANAGER    0x03  // Clock Manager
#define ES8311_REG04_CLK_MANAGER    0x04  // Clock Manager
#define ES8311_REG05_SYSTEM_CTRL1   0x05  // System Control 1
#define ES8311_REG06_SYSTEM_CTRL2   0x06  // System Control 2
#define ES8311_REG07_SYSTEM_CTRL3   0x07  // System Control 3
#define ES8311_REG08_SYSTEM_CTRL4   0x08  // System Control 4
#define ES8311_REG09_SDP_IN         0x09  // Serial Data Port Input
#define ES8311_REG0A_SDP_OUT        0x0A  // Serial Data Port Output
#define ES8311_REG0B_SYSTEM_CTRL5   0x0B  // System Control 5
#define ES8311_REG0C_SYSTEM_CTRL6   0x0C  // System Control 6
#define ES8311_REG0D_BCLK_DIVIDER   0x0D  // BCLK Divider
#define ES8311_REG0E_LRCK_DIVIDER_H 0x0E  // LRCK Divider High
#define ES8311_REG0F_LRCK_DIVIDER_L 0x0F  // LRCK Divider Low
#define ES8311_REG10_MULTI_I2S      0x10  // Multi I2S
#define ES8311_REG11_CLK_MANAGER    0x11  // Clock Manager
#define ES8311_REG12_ANALOG_SYS1    0x12  // Analog System 1
#define ES8311_REG13_ANALOG_SYS2    0x13  // Analog System 2
#define ES8311_REG14_ANALOG_SYS3    0x14  // Analog System 3
#define ES8311_REG15_ANALOG_LP      0x15  // Analog Low Power
#define ES8311_REG16_ANALOG_SYS4    0x16  // Analog System 4
#define ES8311_REG17_ANALOG_SYS5    0x17  // Analog System 5
#define ES8311_REG18_ADC_PGA        0x18  // ADC PGA Gain
#define ES8311_REG19_ADC_CTRL1      0x19  // ADC Control 1
#define ES8311_REG1A_ADC_CTRL2      0x1A  // ADC Control 2
#define ES8311_REG1B_ADC_CTRL3      0x1B  // ADC Control 3
#define ES8311_REG1C_ADC_CTRL4      0x1C  // ADC Control 4
#define ES8311_REG31_DAC_CONTROL1   0x31  // DAC Control 1
#define ES8311_REG32_DAC_CONTROL2   0x32  // DAC Control 2
#define ES8311_REG33_DAC_CONTROL3   0x33  // DAC Control 3
#define ES8311_REG34_DAC_CONTROL4   0x34  // DAC Control 4
#define ES8311_REG35_DAC_CONTROL5   0x35  // DAC Control 5
#define ES8311_REG37_DAC_RAMPRATE   0x37  // DAC Ramp Rate

/**
 * @brief 写入ES8311寄存器
 */
static esp_err_t es8311_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(es8311_handle, write_buf, sizeof(write_buf), -1);
}

/**
 * @brief 读取ES8311寄存器
 */
static esp_err_t es8311_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(es8311_handle, &reg, 1, value, 1, -1);
}

/**
 * @brief 初始化ES8311 Codec
 */
esp_err_t es8311_codec_init(void)
{
    esp_err_t ret;
    uint8_t chip_id;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎵 Initializing ES8311 Audio Codec");
    ESP_LOGI(TAG, "========================================");
    
    // 0. 添加I2C设备
    extern i2c_master_bus_handle_t bus_handle;  // 从myiic获取总线句柄
    if (es8311_handle == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ES8311_ADDR,
            .scl_speed_hz = 400000,
        };
        ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &es8311_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Failed to add ES8311 I2C device: %s", esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGI(TAG, "✅ ES8311 I2C device added (addr=0x%02X, speed=400kHz)", ES8311_ADDR);
    }
    
    // 🔥 关键修复：延迟一下，确保 I2C 总线稳定
    ESP_LOGI(TAG, "⏳ Waiting for I2C bus to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 1. 检测芯片ID（带重试）
    ESP_LOGI(TAG, "🔍 Reading ES8311 chip ID (register 0xFD)...");
    int retry = 0;
    while (retry < 3) {
        ret = es8311_read_reg(0xFD, &chip_id);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ ES8311 detected, Chip ID: 0x%02X (expected: 0x83)", chip_id);
            break;
        } else {
            ESP_LOGW(TAG, "⚠️ Failed to read chip ID (attempt %d/3): %s", 
                     retry + 1, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            retry++;
        }
    }
    
    if (retry >= 3) {
        ESP_LOGE(TAG, "❌ Failed to detect ES8311 after 3 attempts!");
        ESP_LOGE(TAG, "   Possible reasons:");
        ESP_LOGE(TAG, "   - ES8311 not connected or powered");
        ESP_LOGE(TAG, "   - I2C address incorrect (current: 0x%02X)", ES8311_ADDR);
        ESP_LOGE(TAG, "   - I2C bus problem");
        return ESP_FAIL;
    }
    
    // 2. 复位芯片
    ESP_LOGI(TAG, "🔄 Resetting ES8311...");
    es8311_write_reg(ES8311_REG00_RESET, 0x1F);  // Reset all
    vTaskDelay(pdMS_TO_TICKS(10));
    es8311_write_reg(ES8311_REG00_RESET, 0x00);  // Clear reset
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 3. 配置时钟管理 (无MCLK,使用BCLK)
    ESP_LOGI(TAG, "⏱️  Configuring clock...");
    es8311_write_reg(ES8311_REG01_CLK_MANAGER, 0x3F);  // BCLK as clock source (no MCLK)
    es8311_write_reg(ES8311_REG02_CLK_MANAGER, 0x00);  // MCLK divider (not used)
    es8311_write_reg(ES8311_REG03_CLK_MANAGER, 0x10);  // ADC/DAC same clock source
    es8311_write_reg(ES8311_REG04_CLK_MANAGER, 0x00);  // LRCK divider
    es8311_write_reg(ES8311_REG05_SYSTEM_CTRL1, 0x00); // Normal state
    
    // 4. 配置系统控制
    ESP_LOGI(TAG, "⚙️  Configuring system...");
    es8311_write_reg(ES8311_REG06_SYSTEM_CTRL2, 0x00); // Normal state
    es8311_write_reg(ES8311_REG07_SYSTEM_CTRL3, 0x03); // Enable ADC + DAC (bit1=DAC, bit0=ADC)
    es8311_write_reg(ES8311_REG08_SYSTEM_CTRL4, 0x00); // Normal state
    
    // 5. 配置I2S接口(Slave模式, I2S标准格式, 16-bit)
    ESP_LOGI(TAG, "🔌 Configuring I2S interface...");
    es8311_write_reg(ES8311_REG09_SDP_IN, 0x0C);  // Slave mode, I2S format, 16-bit
    es8311_write_reg(ES8311_REG0A_SDP_OUT, 0x0C); // Slave mode, I2S format, 16-bit
    
    // 6. 配置模拟系统
    ESP_LOGI(TAG, "🎛️  Configuring analog system...");
    es8311_write_reg(ES8311_REG0B_SYSTEM_CTRL5, 0x00); // Normal power
    es8311_write_reg(ES8311_REG0C_SYSTEM_CTRL6, 0x00); // Normal state
    es8311_write_reg(ES8311_REG0D_BCLK_DIVIDER, 0x01); // BCLK divider
    
    es8311_write_reg(ES8311_REG12_ANALOG_SYS1, 0x00);  // Normal bias
    es8311_write_reg(ES8311_REG13_ANALOG_SYS2, 0x00);  // Normal state
    es8311_write_reg(ES8311_REG14_ANALOG_SYS3, 0x3A);  // Enable ADC+DAC analog (bit5=ADC, bit4=ADC bias, bit3:1=DAC)
    es8311_write_reg(ES8311_REG15_ANALOG_LP, 0x00);    // Normal power
    es8311_write_reg(ES8311_REG16_ANALOG_SYS4, 0x10);  // ADC input from MIC1, enable PGA
    es8311_write_reg(ES8311_REG17_ANALOG_SYS5, 0xFF);  // Enable ADC+DAC output (bit7=ADC to DSM, bit6:0=DAC)
    
    // 7. 配置DAC
    ESP_LOGI(TAG, "🔊 Configuring DAC...");
    es8311_write_reg(ES8311_REG31_DAC_CONTROL1, 0x00); // Normal state
    es8311_write_reg(ES8311_REG32_DAC_CONTROL2, 0x00); // Unmute DAC
    es8311_write_reg(ES8311_REG33_DAC_CONTROL3, 0x00); // DAC volume (0dB)
    es8311_write_reg(ES8311_REG34_DAC_CONTROL4, 0x00); // Normal state
    es8311_write_reg(ES8311_REG35_DAC_CONTROL5, 0x00); // Normal state
    es8311_write_reg(ES8311_REG37_DAC_RAMPRATE, 0x00); // Normal ramp rate
    
    // 8. 上电DAC
    ESP_LOGI(TAG, "⚡ Powering up DAC...");
    es8311_write_reg(ES8311_REG0B_SYSTEM_CTRL5, 0x00); // Power up
    es8311_write_reg(ES8311_REG0C_SYSTEM_CTRL6, 0x00); // Power up
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ ES8311 initialized successfully!");
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}

/**
 * @brief 配置ES8311为DAC模式(播放)
 */
esp_err_t es8311_codec_config_dac(uint32_t sample_rate, uint8_t bits_per_sample)
{
    ESP_LOGI(TAG, "🔧 Configuring DAC: %lu Hz, %u-bit", sample_rate, bits_per_sample);
    
    // 根据位宽配置I2S接口
    uint8_t sdp_fmt;
    switch (bits_per_sample) {
        case 16:
            sdp_fmt = 0x0C;  // I2S, 16-bit
            break;
        case 24:
            sdp_fmt = 0x1C;  // I2S, 24-bit
            break;
        case 32:
            sdp_fmt = 0x3C;  // I2S, 32-bit
            break;
        default:
            ESP_LOGE(TAG, "Unsupported bit width: %u", bits_per_sample);
            return ESP_ERR_INVALID_ARG;
    }
    
    es8311_write_reg(ES8311_REG09_SDP_IN, sdp_fmt);
    es8311_write_reg(ES8311_REG0A_SDP_OUT, sdp_fmt);
    
    ESP_LOGI(TAG, "✅ DAC configured");
    return ESP_OK;
}

/**
 * @brief 设置DAC音量
 */
esp_err_t es8311_codec_set_voice_volume(uint8_t volume)
{
    if (volume > 100) {
        volume = 100;
    }
    
    // ES8311音量寄存器: 0x00 = 0dB, 0xFF = -95.5dB
    // 线性映射: 100% -> 0x00, 0% -> 0xBF (约-96dB)
    uint8_t reg_value = (uint8_t)((100 - volume) * 0xBF / 100);
    
    ESP_LOGI(TAG, "🔊 Setting volume to %u%% (reg=0x%02X)", volume, reg_value);
    return es8311_write_reg(ES8311_REG33_DAC_CONTROL3, reg_value);
}

/**
 * @brief 控制DAC静音
 */
esp_err_t es8311_codec_set_voice_mute(bool mute)
{
    ESP_LOGI(TAG, "🔇 %s DAC", mute ? "Muting" : "Unmuting");
    
    if (mute) {
        return es8311_write_reg(ES8311_REG32_DAC_CONTROL2, 0x03);  // Mute L+R
    } else {
        return es8311_write_reg(ES8311_REG32_DAC_CONTROL2, 0x00);  // Unmute
    }
}

/**
 * @brief 配置ES8311为ADC模式(录音)
 */
esp_err_t es8311_codec_config_adc(uint32_t sample_rate, uint8_t bits_per_sample)
{
    ESP_LOGI(TAG, "🎤 Configuring ADC: %lu Hz, %u-bit", sample_rate, bits_per_sample);
    
    // ADC 的模拟部分已经在 es8311_codec_init() 中启用
    // 这里只需要配置数字控制寄存器
    
    // 1. 配置ADC控制寄存器
    es8311_write_reg(ES8311_REG19_ADC_CTRL1, 0x00);  // Normal operation
    es8311_write_reg(ES8311_REG1A_ADC_CTRL2, 0x00);  // Normal operation
    es8311_write_reg(ES8311_REG1B_ADC_CTRL3, 0x00);  // Normal operation
    es8311_write_reg(ES8311_REG1C_ADC_CTRL4, 0x00);  // Normal operation
    
    // 2. 设置默认增益 (12dB)
    es8311_codec_set_mic_gain(12);
    
    ESP_LOGI(TAG, "✅ ADC configured");
    return ESP_OK;
}

/**
 * @brief 设置ADC输入增益
 */
esp_err_t es8311_codec_set_mic_gain(uint8_t gain_db)
{
    // ES8311 ADC PGA增益范围: 0-24dB, 步进3dB
    // REG18: 0x00 = 0dB, 0x08 = 24dB
    if (gain_db > 24) {
        gain_db = 24;
    }
    
    uint8_t reg_value = gain_db / 3;  // 3dB per step
    ESP_LOGI(TAG, "🎤 Setting microphone gain to %u dB (reg=0x%02X)", gain_db, reg_value);
    return es8311_write_reg(ES8311_REG18_ADC_PGA, reg_value);
}
