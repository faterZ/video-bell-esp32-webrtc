/**
 * 正点原子音频设备适配层
 * 
 * 将正点原子的I2S驱动（myi2s.c）封装为 esp_codec_dev 接口
 */

#include "audio_device.h"
#include "myi2s.h"
#include "es8311.h"
#include "esp_log.h"
#include "esp_codec_dev.h"
#include "audio_codec_if.h"
#include "audio_codec_data_if.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "audio_device";

// ==================== Audio Codec Interface (codec control) ====================
typedef struct {
    audio_codec_if_t base;
    bool is_open;
    bool is_playback;  // true=playback, false=record
} alientek_codec_t;

static int codec_open(const audio_codec_if_t *h, void *cfg, int cfg_size)
{
    alientek_codec_t *codec = (alientek_codec_t *)h;
    // 🔥 不检查cfg,因为esp_codec_dev_open可能传NULL或sample_info
    // 硬件已经在board.c中初始化完成,这里只是标记状态
    codec->is_open = true;
    ESP_LOGI(TAG, "%s codec opened", codec->is_playback ? "Playback" : "Record");
    
    // 如果是播放设备,设置初始音量为100%
    if (codec->is_playback) {
        ESP_LOGI(TAG, "🔊 Setting initial playback volume to 100%%");
        es8311_codec_set_voice_volume(100);
    }
    
    return ESP_CODEC_DEV_OK;
}

static bool codec_is_open(const audio_codec_if_t *h)
{
    alientek_codec_t *codec = (alientek_codec_t *)h;
    // 🔥 返回实际的打开状态,而不是始终返回true
    // 这样WebRTC可以正确地打开/关闭设备
    return codec->is_open;
}

static int codec_enable(const audio_codec_if_t *h, bool enable)
{
    alientek_codec_t *codec = (alientek_codec_t *)h;
    ESP_LOGI(TAG, "%s codec %s", codec->is_playback ? "Playback" : "Record",
             enable ? "enabled" : "disabled");
    return ESP_CODEC_DEV_OK;
}

static int codec_set_fs(const audio_codec_if_t *h, esp_codec_dev_sample_info_t *fs)
{
    alientek_codec_t *codec = (alientek_codec_t *)h;
    ESP_LOGI(TAG, "%s codec set format: %" PRIu32 "Hz, %u-bit, %u-ch",
             codec->is_playback ? "Playback" : "Record",
             fs->sample_rate, fs->bits_per_sample, fs->channel);
    return ESP_CODEC_DEV_OK;
}

static int codec_mute(const audio_codec_if_t *h, bool mute)
{
    ESP_LOGI(TAG, "Codec %s", mute ? "muted" : "unmuted");
    es8311_codec_set_voice_mute(mute);
    return ESP_CODEC_DEV_OK;
}

static int codec_set_vol(const audio_codec_if_t *h, float db)
{
    // 将dB转换为0-100的百分比
    // WebRTC可能发送-96dB(静音)到0dB(最大)
    // 映射: -96dB -> 0%, 0dB -> 100%
    int volume = 0;
    if (db > -96.0f) {
        volume = (int)((db + 96.0f) * 100.0f / 96.0f);
        if (volume > 100) volume = 100;
        if (volume < 0) volume = 0;
    }
    
    ESP_LOGI(TAG, "🔊 Codec volume set to: %.1f dB -> %d%%", db, volume);
    es8311_codec_set_voice_volume((uint8_t)volume);
    return ESP_CODEC_DEV_OK;
}

static int codec_close(const audio_codec_if_t *h)
{
    alientek_codec_t *codec = (alientek_codec_t *)h;
    codec->is_open = false;
    ESP_LOGI(TAG, "%s codec closed", codec->is_playback ? "Playback" : "Record");
    return ESP_CODEC_DEV_OK;
}

// 可选函数(返回ESP_CODEC_DEV_NOT_SUPPORT表示不支持)
static int codec_set_mic_gain(const audio_codec_if_t *h, float db)
{
    ESP_LOGD(TAG, "Set mic gain: %.1f dB (not implemented)", db);
    return ESP_CODEC_DEV_OK;
}

static int codec_mute_mic(const audio_codec_if_t *h, bool mute)
{
    ESP_LOGD(TAG, "Mic %s (not implemented)", mute ? "muted" : "unmuted");
    return ESP_CODEC_DEV_OK;
}

static const audio_codec_if_t *new_codec_if(bool is_playback)
{
    alientek_codec_t *codec = (alientek_codec_t *)calloc(1, sizeof(alientek_codec_t));
    if (codec == NULL) {
        return NULL;
    }
    codec->base.open = codec_open;
    codec->base.is_open = codec_is_open;
    codec->base.enable = codec_enable;
    codec->base.set_fs = codec_set_fs;
    codec->base.mute = codec_mute;
    codec->base.set_vol = codec_set_vol;
    codec->base.set_mic_gain = codec_set_mic_gain;
    codec->base.set_mic_channel_gain = NULL;  // 不支持分通道增益
    codec->base.mute_mic = codec_mute_mic;
    codec->base.set_reg = NULL;  // 不支持寄存器访问
    codec->base.get_reg = NULL;
    codec->base.dump_reg = NULL;
    codec->base.close = codec_close;
    codec->is_playback = is_playback;
    
    // 🔥 模仿dummy_codec:在创建时就打开codec接口
    // 因为I2S硬件已经在board.c中初始化完成
    codec->base.open(&codec->base, NULL, 0);
    
    return &codec->base;
}

// ==================== Audio Data Interface (I2S read/write) ====================
typedef struct {
    audio_codec_data_if_t base;
    bool is_open;
    bool is_playback;
    uint32_t sample_rate;      // 记录当前采样率
    uint32_t bits_per_sample;  // 记录当前位宽
} alientek_data_if_t;

static int data_open(const audio_codec_data_if_t *h, void *cfg, int cfg_size)
{
    alientek_data_if_t *data_if = (alientek_data_if_t *)h;
    data_if->is_open = true;
    ESP_LOGI(TAG, "%s data interface opened", data_if->is_playback ? "Playback" : "Record");
    return ESP_CODEC_DEV_OK;
}

static bool data_is_open(const audio_codec_data_if_t *h)
{
    alientek_data_if_t *data_if = (alientek_data_if_t *)h;
    // 🔥 返回实际的打开状态,而不是始终返回true
    // 这样WebRTC可以正确地打开/关闭设备
    return data_if->is_open;
}

static int data_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t dev_type, bool enable)
{
    ESP_LOGI(TAG, "Data interface %s (type=%d)", enable ? "enabled" : "disabled", dev_type);
    // 🔥 注意：I2S已经在myi2s_init()中启动，保持运行状态
    // 不调用i2s_trx_start/stop，避免ESP_ERR_INVALID_STATE错误
    // WebRTC需要I2S持续运行以接收/发送音频数据
    return ESP_CODEC_DEV_OK;
}

// 🔥 全局I2S配置状态(录音和播放共享)
static uint32_t g_i2s_sample_rate = 0;
static uint32_t g_i2s_bits_per_sample = 0;

static int data_set_fmt(const audio_codec_data_if_t *h, esp_codec_dev_type_t dev_type,
                       esp_codec_dev_sample_info_t *fs)
{
    alientek_data_if_t *data_if = (alientek_data_if_t *)h;
    
    ESP_LOGI(TAG, "Data format request: %" PRIu32 "Hz, %u-bit, %u-ch (type=%d)",
             fs->sample_rate, fs->bits_per_sample, fs->channel, dev_type);
    
    uint32_t requested_rate = fs->sample_rate;
    if (requested_rate == 0) {
        requested_rate = g_i2s_sample_rate != 0 ? g_i2s_sample_rate : 16000;
        ESP_LOGW(TAG, "⚠️  Sample rate not specified, defaulting to %" PRIu32 " Hz", requested_rate);
    }

    if (g_i2s_sample_rate != 0 && g_i2s_sample_rate != requested_rate) {
        ESP_LOGW(TAG, "⚠️  Reconfiguring shared I2S sample rate from %" PRIu32 " Hz to %" PRIu32 " Hz",
                 g_i2s_sample_rate, requested_rate);
    }

    // 只有在采样率或位宽改变时才重新配置I2S，避免不必要的停顿
    if (g_i2s_sample_rate != requested_rate || g_i2s_bits_per_sample != fs->bits_per_sample) {
        ESP_LOGI(TAG, "🔄 Reconfiguring I2S: %" PRIu32 "Hz, %" PRIu32 "-bit -> %" PRIu32 "Hz, %u-bit",
                 g_i2s_sample_rate, g_i2s_bits_per_sample, requested_rate, fs->bits_per_sample);
        i2s_set_samplerate_bits_sample(requested_rate, fs->bits_per_sample);
        g_i2s_sample_rate = requested_rate;
        g_i2s_bits_per_sample = fs->bits_per_sample;
        ESP_LOGI(TAG, "✅ I2S configured to %" PRIu32 " Hz, %u-bit",
                 requested_rate, fs->bits_per_sample);
    } else {
        ESP_LOGI(TAG, "✅ I2S already at %" PRIu32 " Hz, %u-bit (no reconfiguration needed)",
                 requested_rate, fs->bits_per_sample);
    }

    data_if->sample_rate = requested_rate;
    data_if->bits_per_sample = fs->bits_per_sample;
    
    return ESP_CODEC_DEV_OK;
}

static int data_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    size_t bytes_read = i2s_rx_read(data, size);
    
    // 🔥 详细调试:记录每次读取
    static int read_debug_count = 0;
    read_debug_count++;
    
    // 每次都打印简要信息
    if (read_debug_count <= 20 || (read_debug_count % 100) == 0) {
        ESP_LOGI(TAG, "� [ESP32→Browser] data_read #%d: requested=%d, bytes_read=%u", 
                 read_debug_count, size, bytes_read);
    }
    
    //前5次打印详细数据
    if (read_debug_count <= 10 && bytes_read > 0) {
        int16_t *samples = (int16_t *)data;
        int num_samples = bytes_read / 2;
        int16_t max_val = 0, min_val = 0;
        for (int i = 0; i < num_samples && i < 80; i++) {
            if (samples[i] > max_val) max_val = samples[i];
            if (samples[i] < min_val) min_val = samples[i];
        }
        ESP_LOGI(TAG, "   📊 MIC Data #%d: samples=%d, range=[%d, %d]",
                 read_debug_count, num_samples, min_val, max_val);
                 
        // 🔥 特别警告:如果数据全0
        if (max_val == 0 && min_val == 0) {
            ESP_LOGW(TAG, "   ⚠️  WARNING: Microphone data is all ZERO! Hardware may not be working.");
        }
    }
    
    // 🔥 即使数据全0,也返回读取的字节数
    // 让 WebRTC 决定如何处理静音数据
    if (bytes_read == 0) {
        if (read_debug_count <= 10) {
            ESP_LOGW(TAG, "❌ I2S RX timeout, no data read, returning ESP_FAIL");
        }
        return ESP_FAIL;
    }
    
    // ✅ 返回实际读取的字节数(即使全0)
    return (int)bytes_read;
}

static int data_write(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    // 【日志】检查写入数据
    static int write_debug_count = 0;
    write_debug_count++;
    
    if (write_debug_count <= 5) {
        int16_t *samples = (int16_t *)data;
        int num_samples = size / 2;
        int16_t max_val = 0, min_val = 0;
        for (int i = 0; i < num_samples && i < 100; i++) {
            if (samples[i] > max_val) max_val = samples[i];
            if (samples[i] < min_val) min_val = samples[i];
        }
        ESP_LOGI(TAG, "🔊 [TX Write #%d] requested=%d bytes, samples=%d, range=[%d, %d]", 
                 write_debug_count, size, num_samples, min_val, max_val);
    }
    
    size_t bytes_written = i2s_tx_write(data, size);
    
    if (write_debug_count <= 5) {
        ESP_LOGI(TAG, "✅ [TX Write #%d] bytes_written=%u", write_debug_count, bytes_written);
    }
    
    // 🔥 如果没有写入数据,返回错误(ESP_FAIL=-1)
    if (bytes_written == 0) {
        ESP_LOGW(TAG, "I2S TX timeout, no data written");
        return ESP_FAIL;
    }
    return (int)bytes_written;
}

static int data_close(const audio_codec_data_if_t *h)
{
    alientek_data_if_t *data_if = (alientek_data_if_t *)h;
    data_if->is_open = false;
    ESP_LOGI(TAG, "%s data interface closed", data_if->is_playback ? "Playback" : "Record");
    return ESP_CODEC_DEV_OK;
}

static const audio_codec_data_if_t *new_data_if(bool is_playback)
{
    alientek_data_if_t *data_if = (alientek_data_if_t *)calloc(1, sizeof(alientek_data_if_t));
    if (data_if == NULL) {
        return NULL;
    }
    data_if->base.open = data_open;
    data_if->base.is_open = data_is_open;
    data_if->base.enable = data_enable;
    data_if->base.set_fmt = data_set_fmt;
    data_if->base.read = data_read;
    data_if->base.write = data_write;
    data_if->base.close = data_close;
    data_if->is_playback = is_playback;
    
    // 🔥 模仿dummy_codec:在创建时就打开data接口
    // 因为I2S硬件已经在board.c中初始化完成
    data_if->base.open(&data_if->base, NULL, 0);
    
    data_if->sample_rate = 0;
    data_if->bits_per_sample = 0;
    return &data_if->base;
}

// ==================== Device Handles ====================
static esp_codec_dev_handle_t playback_handle = NULL;
static esp_codec_dev_handle_t record_handle = NULL;

esp_err_t audio_device_init(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🎵 Initializing ALIENTEK Audio Device");
    ESP_LOGI(TAG, "========================================");

    // 创建播放设备
    esp_codec_dev_cfg_t playback_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = new_codec_if(true),
        .data_if = new_data_if(true),
    };
    playback_handle = esp_codec_dev_new(&playback_cfg);
    if (playback_handle == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create playback device");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ Playback device created");
    
    // 打开播放设备
    ESP_LOGI(TAG, "🔓 Opening playback device...");
    esp_codec_dev_sample_info_t sample_info = {
        .sample_rate = 16000,
        .channel = 1,
        .bits_per_sample = 16,
    };
    int ret = esp_codec_dev_open(playback_handle, &sample_info);
    ESP_LOGI(TAG, "📊 esp_codec_dev_open returned: %d", ret);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "❌ Failed to open playback device (error code: %d)", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ Playback device opened");
    
    // 🔥 设置默认音量为100%,覆盖ESP-IDF默认的0%
    // ESP-IDF的esp_codec_dev在创建时默认volume=0,会在打开时应用-96dB(静音)
    // 必须在open后立即设置音量,才能覆盖默认值
    ret = esp_codec_dev_set_out_vol(playback_handle, 100);
    if (ret == ESP_CODEC_DEV_OK) {
        ESP_LOGI(TAG, "🔊 Playback volume set to 100%% (overriding ESP-IDF default 0%%)");
    } else {
        ESP_LOGW(TAG, "⚠️  Failed to set playback volume (ret=%d)", ret);
    }
    
    // 确保未静音
    esp_codec_dev_set_out_mute(playback_handle, false);
    ESP_LOGI(TAG, "🔊 Playback unmuted");

    // 创建录音设备
    esp_codec_dev_cfg_t record_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = new_codec_if(false),
        .data_if = new_data_if(false),
    };
    record_handle = esp_codec_dev_new(&record_cfg);
    if (record_handle == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create record device");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ Record device created");
    
    // 🔥 不要在初始化时打开录音设备!
    // WebRTC会在需要时调用esp_codec_dev_open(),并传入正确的采样率(8kHz)
    // 如果这里提前打开,WebRTC会认为设备已打开,跳过重新配置,导致采样率不匹配
    ESP_LOGI(TAG, "⏭️  Record device will be opened by WebRTC when needed");

    // 注意:I2S已经在board.c的myi2s_init()中启动,无需重复启动
    ESP_LOGI(TAG, "✅ I2S already started by board initialization");

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ Audio device initialization complete");
    ESP_LOGI(TAG, "========================================");
    return ESP_OK;
}

esp_codec_dev_handle_t audio_get_playback_handle(void)
{
    return playback_handle;
}

esp_codec_dev_handle_t audio_get_record_handle(void)
{
    return record_handle;
}
