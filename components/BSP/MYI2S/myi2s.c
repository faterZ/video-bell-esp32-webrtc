/**
 ****************************************************************************************************
 * @file        i2s.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       I2S驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32S3 BOX 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

 #include "myi2s.h"
 #include "esp_log.h"


 i2s_chan_handle_t tx_handle = NULL;     /* I2S发送通道句柄 */
 i2s_chan_handle_t rx_handle = NULL;     /* I2S接收通道句柄 */
 i2s_std_config_t my_std_cfg;            /* 标准模式配置结构体(TX和RX共享,参考小智AI) */
 /* 添加全局增益变量(可根据需求调整为参数或配置项) */
 #define AUDIO_GAIN_FACTOR       1.0f    /* 该增益最大范围不能超过3.0f */
 /* 定义录音增益因子 */
 #define RECORER_GAIN_FACTOR     1.0f
 
 int my_data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;  /* 采样数据位宽 */
 
 /* 🔥 新增: 回调函数指针 (参考小智 AI) */
 static i2s_data_ready_callback_t rx_data_ready_callback = NULL;
 static i2s_data_ready_callback_t tx_data_ready_callback = NULL;
 
 /* 🔥 新增: I2S RX 事件回调 (在中断上下文中调用) */
 static bool on_i2s_rx_ready(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
     if (rx_data_ready_callback) {
         return rx_data_ready_callback();
     }
     return false;
 }
 
 /* 🔥 新增: I2S TX 事件回调 (在中断上下文中调用) */
 static bool on_i2s_tx_ready(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) {
     if (tx_data_ready_callback) {
         return tx_data_ready_callback();
     }
     return false;
 }
 
 /*
 * @brief       初始化I2S
 * @param       无
 * @retval      ESP_OK:初始化成功;其他:失败
 */
 esp_err_t myi2s_init(void)
 {
     i2s_chan_config_t chan_cfg = {
         .id = I2S_NUM_0,
         .role = I2S_ROLE_MASTER,
         .dma_desc_num = 6,
         .dma_frame_num = 240,
         .auto_clear_after_cb = true,
         .auto_clear_before_cb = false,
         .intr_priority = 0,
     };
     ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
 
     /* ============================================
      * TX/RX 通道: 标准模式 (参考小智AI配置)
      * TX: ES8311 DAC 播放
      * RX: MSM261S4030H0R MEMS麦克风录音
      * ============================================ */
     i2s_std_config_t std_cfg = {
         .clk_cfg  = {
             .sample_rate_hz = I2S_SAMPLE_RATE,
             .clk_src        = I2S_CLK_SRC_DEFAULT,
             .mclk_multiple  = I2S_MCLK_MULTIPLE_256,        /* 与小智AI保持一致 */
         },
 
        .slot_cfg = {
            .data_bit_width = my_data_bit_width,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_STEREO,
            .slot_mask      = I2S_STD_SLOT_BOTH,
            .ws_width       = my_data_bit_width,
            .ws_pol         = false,
            .bit_shift      = true,
            .left_align     = true,
            .big_endian     = false,
            .bit_order_lsb  = false
        },
        
        .gpio_cfg = {
             .mclk = I2S_MCK_IO,
             .bclk = I2S_BCK_IO,
             .ws   = I2S_WS_IO,
             .dout = I2S_DO_IO,
             .din  = I2S_DI_IO,                              /* RX和TX共享配置,与小智AI一致 */
             .invert_flags = {
                 .mclk_inv = false,
                 .bclk_inv = false,
                 .ws_inv   = false,
             },
         },
     };
 

    my_std_cfg = std_cfg;
    
    /* TX 和 RX 通道: 立体声模式 (参考小智 AI 成功配置)
     * 小智 AI 验证: ATK_NoAudioCodecDuplex 使用立体声配置
     * TX: ES8311 DAC 播放
     * RX: MSM261 麦克风录音 (虽然是单麦克风,但配置为立体声)
     */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));  // RX 也使用立体声
    
    /* 🔥 关键修复: 先注册回调，再启用通道 (完全对齐小智 AI 的初始化顺序)
     * 小智 AI 的 AudioCodec::Start() 顺序:
     * 1. i2s_channel_register_event_callback(rx_handle_, &rx_callbacks, this);
     * 2. i2s_channel_register_event_callback(tx_handle_, &tx_callbacks, this);
     * 3. i2s_channel_enable(tx_handle_);
     * 4. i2s_channel_enable(rx_handle_);
     */
    ESP_LOGI("MYI2S", "📌 Registering I2S event callbacks...");
    i2s_event_callbacks_t rx_callbacks = {
        .on_recv = on_i2s_rx_ready,
        .on_recv_q_ovf = NULL,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &rx_callbacks, NULL));
    
    i2s_event_callbacks_t tx_callbacks = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = on_i2s_tx_ready,
        .on_send_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &tx_callbacks, NULL));
    
    /* 🔥 关键修复: 在 enable 之前预加载 TX DMA 缓冲区
     * I2S 标准模式需要先填充 TX DMA 缓冲区，时钟才会启动
     * 必须在 i2s_channel_enable() 之前调用 i2s_channel_preload_data()
     * 参考: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2s.html
     */
    ESP_LOGI("MYI2S", "⏰ Preloading TX DMA buffer (BEFORE enable)...");
    uint8_t silence[1440] = {0};  // 30ms @ 24kHz stereo 16-bit (6 DMA descriptors * 240 frames)
    size_t bytes_written;
    
    // 预加载多个 DMA 缓冲区以启动时钟
    for (int i = 0; i < 3; i++) {
        esp_err_t ret = i2s_channel_preload_data(tx_handle, silence, sizeof(silence), &bytes_written);
        if (ret == ESP_OK) {
            ESP_LOGI("MYI2S", "   ✅ Preload #%d: %d bytes", i+1, bytes_written);
        } else {
            ESP_LOGW("MYI2S", "   ⚠️  Preload #%d failed: %s", i+1, esp_err_to_name(ret));
            break;
        }
    }
    
    ESP_LOGI("MYI2S", "🚀 Enabling I2S channels...");
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));                         /* 启用TX通道 */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));                         /* 启用RX通道 */

    ESP_LOGI("MYI2S", "✅ I2S initialized: TX=Stereo, RX=Stereo (Xiaozhi AI config with callbacks)");
    return ESP_OK;
} /**
  * @brief       I2S TRX启动
  * @param       无
  * @retval      无
  */
 void i2s_trx_start(void)
 {
     // 🔥 容错处理:如果通道已启用,忽略错误
     esp_err_t err_tx = i2s_channel_enable(tx_handle);
     if (err_tx != ESP_OK && err_tx != ESP_ERR_INVALID_STATE) {
         ESP_ERROR_CHECK(err_tx);
     }
     
     esp_err_t err_rx = i2s_channel_enable(rx_handle);
     if (err_rx != ESP_OK && err_rx != ESP_ERR_INVALID_STATE) {
         ESP_ERROR_CHECK(err_rx);
     }
 }
 
/**
 * @brief       I2S TRX停止
 * @param       无
 * @retval      无
 */
void i2s_trx_stop(void)
{
    // 🔥 容错处理:如果通道未启用,忽略错误
    esp_err_t err_tx = i2s_channel_disable(tx_handle);
    if (err_tx != ESP_OK && err_tx != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err_tx);
    }
    
    esp_err_t err_rx = i2s_channel_disable(rx_handle);
    if (err_rx != ESP_OK && err_rx != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err_rx);
    }
} /**
  * @brief       I2S卸载
  * @param       无
  * @retval      无
  */
 void i2s_deinit(void)
 {
     ESP_ERROR_CHECK(i2s_del_channel(tx_handle));
     ESP_ERROR_CHECK(i2s_del_channel(rx_handle));
 }
 
/**
 * @brief       设置采样率和位宽
 * @param       sampleRate  :采样率
 * @param       bits_sample :位宽
 * @retval      无
 */
void i2s_set_samplerate_bits_sample(int samplerate, int bits_sample)
{
    // 🔥 容错:只有在I2S运行时才停止,否则跳过
    i2s_trx_stop();
    
    /* TX 和 RX 通道(都使用标准模式,参考小智AI) */
    my_std_cfg.slot_cfg.ws_width = bits_sample;
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(tx_handle, &my_std_cfg.slot_cfg));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(rx_handle, &my_std_cfg.slot_cfg));
    
    my_std_cfg.clk_cfg.sample_rate_hz = samplerate;
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(tx_handle, &my_std_cfg.clk_cfg));
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(rx_handle, &my_std_cfg.clk_cfg));
    
    // 🔥 重新启动I2S
    i2s_trx_start();
    
    ESP_LOGI("MYI2S", "✅ I2S reconfigured: %d Hz, %d-bit (TX & RX both Standard I2S)", samplerate, bits_sample);
} /**
  * @brief       I2S传输数据
  * @param       buffer: 数据存储区的首地址
  * @param       frame_size: 数据大小
  * @retval      发送的数据长度
  */
 size_t i2s_tx_write(uint8_t *buffer, uint32_t frame_size) 
 {
     // 🔥 调试:检查写入前的数据
     static int write_count = 0;
     write_count++;
     
     if (write_count <= 10 && my_data_bit_width == I2S_DATA_BIT_WIDTH_16BIT) {
         int16_t *samples = (int16_t *)buffer;
         int num_samples = frame_size / 2;
         int16_t max_val = 0, min_val = 0;
         for (int i = 0; i < num_samples && i < 100; i++) {
             if (samples[i] > max_val) max_val = samples[i];
             if (samples[i] < min_val) min_val = samples[i];
         }
         ESP_LOGI("MYI2S", "🔊 [TX Before Gain] #%d: size=%u, samples=%d, range=[%d, %d]", 
                  write_count, frame_size, num_samples, min_val, max_val);
     }
     
     /* 解析采样数据 */
     const int bytes_per_sample = my_data_bit_width / 8;
     const int samples_count = frame_size / bytes_per_sample;
 
     /* 根据位深处理数据 */
     switch (my_data_bit_width)
     {
         case I2S_DATA_BIT_WIDTH_16BIT:
         {
             int16_t *samples = (int16_t *)buffer;
 
             for (int i = 0; i < samples_count; i++)
             {
                 int32_t amplified = samples[i] * AUDIO_GAIN_FACTOR;
                 amplified = amplified > INT16_MAX ? INT16_MAX : amplified < INT16_MIN ? INT16_MIN : amplified;
                 samples[i] = (int16_t)amplified;
             }
 
             break;
         }
         
         case I2S_DATA_BIT_WIDTH_24BIT:
         {
             /* 24-bit数据通常存储在32位的高24位 */
             int32_t *samples = (int32_t *)buffer;
 
             for (int i = 0; i < samples_count; i++)
             {
                 /* 提取有效24位数据（带符号扩展） */
                 int32_t raw = samples[i] >> 8; 
                 int32_t amplified = raw * AUDIO_GAIN_FACTOR;
                 amplified = amplified > 0x7FFFFF ? 0x7FFFFF : amplified < -0x800000 ? -0x800000 : amplified;
                 samples[i] = amplified << 8; /* 存回高24位 */
             }
             break;
         }
         
         case I2S_DATA_BIT_WIDTH_32BIT:
         {
             int32_t *samples = (int32_t *)buffer;
 
             for (int i = 0; i < samples_count; i++)
             {
                 int64_t amplified = (int64_t)(samples[i] * AUDIO_GAIN_FACTOR);
                 amplified = amplified > INT32_MAX ? INT32_MAX : amplified < INT32_MIN ? INT32_MIN : amplified;
                 samples[i] = (int32_t)amplified;
             }
 
             break;
         }
     }
 
     /* 写入放大后的数据 */
     size_t bytes_written;
     esp_err_t err = i2s_channel_write(tx_handle, buffer, frame_size, &bytes_written, 1000);
     
     // 🔥 调试:检查 I2S 硬件写入结果
     if (write_count <= 10 || err != ESP_OK || bytes_written == 0) {
         ESP_LOGI("MYI2S", "📤 [TX I2S Write] #%d: requested=%u, written=%u, err=%s(0x%x)", 
                  write_count, frame_size, bytes_written, esp_err_to_name(err), err);
     }
     
     // 检查写入后的数据(增益后)
     if (write_count <= 5 && bytes_written > 0 && my_data_bit_width == I2S_DATA_BIT_WIDTH_16BIT) {
         int16_t *samples = (int16_t *)buffer;
         int num_samples = bytes_written / 2;
         int16_t max_val = 0, min_val = 0;
         for (int i = 0; i < num_samples && i < 100; i++) {
             if (samples[i] > max_val) max_val = samples[i];
             if (samples[i] < min_val) min_val = samples[i];
         }
         ESP_LOGI("MYI2S", "   🔊 [After Gain & Write] range=[%d, %d] (gain=%.1f)", 
                  min_val, max_val, AUDIO_GAIN_FACTOR);
     }
     
     if (err != ESP_OK) {
         ESP_LOGE("MYI2S", "❌ i2s_channel_write failed!");
         return 0;
     }
     
     return bytes_written;
 }
 
 /**
  * @brief       I2S读取数据
  * @param       buffer: 读取数据存储区的首地址
  * @param       frame_size: 读取数据大小
  * @retval      接收的数据长度
  */
size_t i2s_rx_read(uint8_t *buffer, uint32_t frame_size)
{
    size_t bytes_written = 0;
    
    // 🔥 使用和官方一样的方式:直接读取,不检查错误
    // 如果有错误会触发 ESP_ERROR_CHECK 的 panic
    ESP_ERROR_CHECK(i2s_channel_read(rx_handle, buffer, frame_size, &bytes_written, 1000));
    
    // 🔥 详细调试:记录每次读取的情况
    static int read_count = 0;
    read_count++;
    if (read_count <= 10) {
        ESP_LOGI("MYI2S", "📖 RX Read #%d: requested=%u, read=%u", 
                 read_count, frame_size, bytes_written);
    }
    
    // 🔥 调试:打印增益处理前的原始数据
    if (read_count <= 3 && bytes_written > 0 && my_data_bit_width == I2S_DATA_BIT_WIDTH_16BIT) {
        int16_t *samples = (int16_t *)buffer;
        int num_samples = bytes_written / 2;
        int16_t max_val = 0, min_val = 0;
        for (int i = 0; i < num_samples && i < 80; i++) {
            if (samples[i] > max_val) max_val = samples[i];
            if (samples[i] < min_val) min_val = samples[i];
        }
        ESP_LOGI("MYI2S", "🎤 RAW I2S Data #%d: samples=%d, range=[%d, %d] (BEFORE gain)", 
                 read_count, num_samples, min_val, max_val);
        // 打印前8个样本的十六进制值
        ESP_LOGI("MYI2S", "   First 8 samples (hex): %04x %04x %04x %04x %04x %04x %04x %04x",
                 samples[0], samples[1], samples[2], samples[3],
                 samples[4], samples[5], samples[6], samples[7]);
    }
    
    /* 根据位深处理数据 */
    switch (my_data_bit_width)
    {
        case I2S_DATA_BIT_WIDTH_16BIT:
        {
            for (size_t i = 0; i < bytes_written / sizeof(int16_t); i++)
            {
                int16_t sample = ((int16_t *)buffer)[i];
                int32_t scaled_sample = sample * RECORER_GAIN_FACTOR;                 /* 防止溢出 */
                 if (scaled_sample > INT16_MAX)
                 {
                     scaled_sample = INT16_MAX;
                 }
                 else if (scaled_sample < INT16_MIN)
                 {
                     scaled_sample = INT16_MIN;
                 }
     
                 ((int16_t *)buffer)[i] = (int16_t)scaled_sample;
             }
             break;
         }
         
         case I2S_DATA_BIT_WIDTH_24BIT:
         {
             for (size_t i = 0; i < bytes_written / 3; i++)
             {
                 /* 24位样本需要特殊处理，因为它们不是标准的整数大小 */
                 int32_t sample = ((int8_t *)buffer)[3 * i] | ((int8_t *)buffer)[3 * i + 1] << 8 | ((int8_t *)buffer)[3 * i + 2] << 16;
     
                 if (sample & 0x800000)
                 { 
                     /* 如果是负数，扩展符号位 */
                     sample |= 0xFF000000;
                 }
                 int64_t scaled_sample = sample * RECORER_GAIN_FACTOR;
     
                 /* 防止溢出 */
                 if (scaled_sample > INT32_MAX)
                 {
                     scaled_sample = INT32_MAX;
                 }
                 else if (scaled_sample < INT32_MIN)
                 {
                     scaled_sample = INT32_MIN;
                 }
     
                 /* 将处理后的样本写回缓冲区 */
                 ((int8_t *)buffer)[3 * i] = scaled_sample & 0xFF;
                 ((int8_t *)buffer)[3 * i + 1] = (scaled_sample >> 8) & 0xFF;
                 ((int8_t *)buffer)[3 * i + 2] = (scaled_sample >> 16) & 0xFF;
             }
             break;
         }
         
         case I2S_DATA_BIT_WIDTH_32BIT:
         {
             for (size_t i = 0; i < bytes_written / sizeof(int32_t); i++)
             {
                 int32_t sample = ((int32_t *)buffer)[i];
                 int64_t scaled_sample = sample * RECORER_GAIN_FACTOR;
     
                 /* 防止溢出 */
                 if (scaled_sample > INT32_MAX)
                 {
                     scaled_sample = INT32_MAX;
                 }
                 else if (scaled_sample < INT32_MIN)
                 {
                     scaled_sample = INT32_MIN;
                 }
     
                 ((int32_t *)buffer)[i] = (int32_t)scaled_sample;
             }
             break;
         }
     }
 
     return bytes_written;
 }

/**
 * @brief       注册 RX 数据就绪回调
 * @param       callback: 回调函数指针
 * @retval      无
 */
void i2s_register_rx_callback(i2s_data_ready_callback_t callback)
{
    rx_data_ready_callback = callback;
    ESP_LOGI("MYI2S", "✅ RX callback registered: %p", callback);
}

/**
 * @brief       注册 TX 数据就绪回调
 * @param       callback: 回调函数指针
 * @retval      无
 */
void i2s_register_tx_callback(i2s_data_ready_callback_t callback)
{
    tx_data_ready_callback = callback;
    ESP_LOGI("MYI2S", "✅ TX callback registered: %p", callback);
}