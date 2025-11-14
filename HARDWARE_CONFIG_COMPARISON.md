# 硬件配置对比：你的项目 vs 小智 AI

## ✅ GPIO 引脚配置（完全一致）

| 功能 | 小智 AI | 你的配置 | 状态 |
|------|---------|----------|------|
| **I2S WS (左右声道时钟)** | GPIO_NUM_13 | GPIO_NUM_13 | ✅ 相同 |
| **I2S BCLK (位时钟)** | GPIO_NUM_21 | GPIO_NUM_21 | ✅ 相同 |
| **I2S DIN (麦克风输入)** | GPIO_NUM_47 | GPIO_NUM_47 | ✅ 相同 |
| **I2S DOUT (扬声器输出)** | GPIO_NUM_14 | GPIO_NUM_14 | ✅ 相同 |
| **I2S MCLK (主时钟)** | GPIO_NUM_NC | GPIO_NUM_NC | ✅ 相同 |
| **I2C SDA** | GPIO_NUM_48 | GPIO_NUM_48 | ✅ 相同 |
| **I2C SCL** | GPIO_NUM_45 | GPIO_NUM_45 | ✅ 相同 |

---

## ✅ I2S 配置（已完全对齐）

| 配置项 | 小智 AI | 你的配置（修改后） | 状态 |
|--------|---------|-------------------|------|
| **采样率** | 24000 Hz | **24000 Hz** ✅ | ✅ 已修改 |
| **数据位宽** | 16-bit | 16-bit | ✅ 相同 |
| **声道模式** | STEREO | STEREO | ✅ 相同 |
| **声道掩码** | BOTH | BOTH | ✅ 相同 |
| **MCLK 倍数** | 256 | 256 | ✅ 相同 |
| **DMA 描述符** | 6 | 6 | ✅ 相同 |
| **DMA 帧数** | 240 | 240 | ✅ 相同 |

---

## 🎯 小智 AI 使用的音频路径

### NS4168 模式（你的硬件）

小智 AI 使用 `ATK_NoAudioCodecDuplex` 类：

```cpp
// 小智 AI 配置
i2s_std_config_t std_cfg = {
    .clk_cfg = {
        .sample_rate_hz = 24000,              // ✅ 24kHz
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
        .slot_mode = I2S_SLOT_MODE_STEREO,    // ✅ 立体声
        .slot_mask = I2S_STD_SLOT_BOTH,       // ✅ 左右声道
        .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
        .ws_pol = false,
        .bit_shift = true,
        .left_align = true,
        .big_endian = false,
        .bit_order_lsb = false
    },
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = GPIO_NUM_21,
        .ws = GPIO_NUM_13,
        .dout = GPIO_NUM_14,
        .din = GPIO_NUM_47,
    }
};
// TX 和 RX 使用相同配置！
ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
```

---

## 📋 修改记录

### 1. 采样率修改（最关键）
- **之前**: 44.1kHz
- **修改后**: 24kHz（对齐小智 AI）
- **文件**: `components/BSP/MYI2S/myi2s.h`

### 2. ES8311 DAC/ADC 配置
- **之前**: 16kHz
- **修改后**: 24kHz
- **文件**: `main/board.c`

### 3. I2S 声道模式
- **之前**: 单声道左声道
- **修改后**: 立体声，左右声道（STEREO + BOTH）
- **文件**: `components/BSP/MYI2S/myi2s.c`

---

## 🔬 诊断工具

已添加麦克风诊断工具（`mic_diagnostic.c`），会自动测试：
1. STEREO + BOTH（小智 AI 默认）
2. MONO + LEFT（MSM261 L/R=GND）
3. MONO + RIGHT（MSM261 L/R=VDD）
4. STEREO + LEFT only
5. STEREO + RIGHT only

启动后自动运行诊断，找出哪个配置有数据。

---

## ⚠️ 可能的问题

如果麦克风仍然无数据，可能原因：
1. **MSM261 L/R 引脚状态**：需要确认是接地（左声道）还是接 VDD（右声道）
2. **I2S 时钟问题**：检查 GPIO47 是否有时钟信号
3. **硬件连接**：GPIO47 可能没有正确连接到 MSM261
4. **MSM261 电源**：麦克风可能没有正确供电

---

## 🎯 下一步

1. **编译烧录**，观察诊断输出
2. **检查哪个配置有非零数据**
3. **根据诊断结果调整配置**
4. **如果所有配置都是 0，需要检查硬件连接**

---

## 📚 参考文件

- 小智 AI 配置：`example/xiaozhi_ai_demo/xiaozhi-esp32s3_box/xiaozhi-esp32s3_box/main/boards/atk-dnesp32s3-box/config.h`
- 小智 AI 音频驱动：`example/xiaozhi_ai_demo/.../main/audio_codecs/no_audio_codec.cc`
- 你的 I2S 配置：`components/BSP/MYI2S/myi2s.c`
- 你的硬件初始化：`main/board.c`
