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


 i2s_chan_handle_t tx_handle = NULL;     /* I2S发送通道句柄 */
 i2s_chan_handle_t rx_handle = NULL;     /* I2S接收通道句柄 */
 i2s_std_config_t my_std_cfg;            /* 标准模式配置结构体 */
 /* 添加全局增益变量（可根据需求调整为参数或配置项） */
 #define AUDIO_GAIN_FACTOR       1.0f    /* 该增益最大范围不能超过3.0f */
 /* 定义录音增益因子 */
 #define RECORER_GAIN_FACTOR     1.0f
 
 int my_data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;  /* 采样数据位宽 */
 
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
 
     i2s_std_config_t std_cfg = {    /* 标准通信模式配置 */
         .clk_cfg  = {               /* 时钟配置 可用I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE)宏函数辅助配置 */
             .sample_rate_hz = I2S_SAMPLE_RATE,              /* I2S采样率 */
             .clk_src        = I2S_CLK_SRC_DEFAULT,          /* I2S时钟源 */
             .mclk_multiple  = I2S_MCLK_MULTIPLE,            /* I2S主时钟MCLK相对于采样率的倍数(默认256) */
         },
 
         .slot_cfg = {               /* 声道配置,可用I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)宏函数辅助配置(支持16位宽采样数据) */
             .data_bit_width = my_data_bit_width,            /* 声道支持16位宽的采样数据 */
             .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,      /* 通道位宽 */
             .slot_mode      = I2S_SLOT_MODE_STEREO,         /* 立体声 */
             .slot_mask      = I2S_STD_SLOT_BOTH,            /* 启用通道 */
             .ws_width       = my_data_bit_width,            /* WS信号位宽 */
             .ws_pol         = false,                        /* WS信号极性 */
             .bit_shift      = true,                         /* 位移位(Philips模式下配置) */
             .left_align     = true,                         /* 左对齐 */
             .big_endian     = false,                        /* 小端模式 */
             .bit_order_lsb  = false                         /* MSB */
         }, 
         
         .gpio_cfg = {                                       /* 引脚配置 */
             .mclk = I2S_MCK_IO,                             /* 主时钟线 */
             .bclk = I2S_BCK_IO,                             /* 位时钟线 */
             .ws   = I2S_WS_IO,                              /* 字(声道)选择线 */
             .dout = I2S_DO_IO,                              /* 串行数据输出线 */
             .din  = I2S_DI_IO,                              /* 串行数据输入线 */
             .invert_flags = {                               /* 引脚翻转(不反相) */
                 .mclk_inv = false,
                 .bclk_inv = false,
                 .ws_inv   = false,
             },
         },
     };
 
     my_std_cfg = std_cfg;
 
     ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));    /* 初始化TX通道 */
     ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));    /* 初始化RX通道 */
     ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));                     /* 启用TX通道 */
     ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));                     /* 启用RX通道 */
 
     return ESP_OK;
 }
 
 /**
  * @brief       I2S TRX启动
  * @param       无
  * @retval      无
  */
 void i2s_trx_start(void)
 {
     ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
     ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
 }
 
 /**
  * @brief       I2S TRX停止
  * @param       无
  * @retval      无
  */
 void i2s_trx_stop(void)
 {
     ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
     ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
 }
 
 /**
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
     i2s_trx_stop();
     /* 如果需要更新声道或时钟配置,需要在更新前先禁用通道 */
     my_std_cfg.slot_cfg.ws_width = bits_sample;        /* 位宽 */
     ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(tx_handle, &my_std_cfg.slot_cfg));
     my_std_cfg.clk_cfg.sample_rate_hz = samplerate;    /* 设置采样率 */
     ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(tx_handle, &my_std_cfg.clk_cfg));
 }
 
 /**
  * @brief       I2S传输数据
  * @param       buffer: 数据存储区的首地址
  * @param       frame_size: 数据大小
  * @retval      发送的数据长度
  */
 size_t i2s_tx_write(uint8_t *buffer, uint32_t frame_size) 
 {
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
     ESP_ERROR_CHECK(i2s_channel_write(tx_handle, buffer, frame_size, &bytes_written, 1000));
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
     size_t bytes_written;
     ESP_ERROR_CHECK(i2s_channel_read(rx_handle, buffer, frame_size, &bytes_written, 1000));
     /* 根据位深处理数据 */
     switch (my_data_bit_width)
     {
         case I2S_DATA_BIT_WIDTH_16BIT:
         {
             for (size_t i = 0; i < bytes_written / sizeof(int16_t); i++)
             {
                 int16_t sample = ((int16_t *)buffer)[i];
                 int32_t scaled_sample = sample * RECORER_GAIN_FACTOR;
     
                 /* 防止溢出 */
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