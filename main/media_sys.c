/* Media system

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "codec_init.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "codec_board.h"
#endif
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "esp_video_init.h"
#endif
#if CONFIG_IDF_TARGET_ESP32S3
#include "usb_stream.h"
#include "jpeg_decoder.h"
#include "ppbuffer.h"
#include "lcd.h"
#endif
#include "av_render.h"
#include "av_render_default.h"
#include "common.h"
#include "esp_log.h"
#include "settings.h"
#include "media_lib_os.h"
#include "esp_timer.h"
#include "encoder/esp_audio_enc_default.h"
#include "encoder/esp_video_enc_default.h"
#include "decoder/esp_video_dec_default.h"
#include "decoder/esp_audio_dec_default.h"
#include "esp_capture_defaults.h"
#include "esp_capture_sink.h"
#include "servo_control.h"
#include "motion_tracker.h"
#include "esp_bit_defs.h"

#define TAG "MEDIA_SYS"

#define RET_ON_NULL(ptr, v) do {                                \
    if (ptr == NULL) {                                          \
        ESP_LOGE(TAG, "Memory allocate fail on %d", __LINE__);  \
        return v;                                               \
    }                                                           \
} while (0)

typedef struct {
    esp_capture_sink_handle_t   capture_handle;
    esp_capture_video_src_if_t *vid_src;
    esp_capture_audio_src_if_t *aud_src;
} capture_system_t;

typedef struct {
    audio_render_handle_t audio_render;
    video_render_handle_t video_render;
    av_render_handle_t    player;
} player_system_t;

static capture_system_t capture_sys;
static player_system_t  player_sys;

static bool           music_playing  = false;
static bool           music_stopping = false;
static const uint8_t *music_to_play;
static int            music_size;
static int            music_duration;

#if CONFIG_IDF_TARGET_ESP32S3
// USB摄像头相关全局变量
#define DEMO_UVC_XFER_BUFFER_SIZE (88 * 1024)  // 双缓冲
static uint8_t *jpg_frame_buf1 = NULL;
static uint8_t *jpg_frame_buf2 = NULL;
static uint8_t *xfer_buffer_a = NULL;
static uint8_t *xfer_buffer_b = NULL;
static uint8_t *frame_buffer = NULL;
static PingPongBuffer_t *ppbuffer_handle = NULL;
static uint16_t current_width = 0;
static uint16_t current_height = 0;
static bool if_ppbuffer_init = false;
static TaskHandle_t s_usb_display_task = NULL;
static bool s_usb_camera_initialized = false;
static bool s_usb_state_cb_registered = false;
static int s_lcd_width = 0;
static int s_lcd_height = 0;
static bool s_preview_first_frame_seen = false;
static void ensure_lcd_dimensions(void);
typedef struct {
    int64_t timestamp_us;
    char    message[80];
} preview_event_t;

#define PREVIEW_EVENT_MAX 32
static preview_event_t s_preview_events[PREVIEW_EVENT_MAX];
static size_t s_preview_event_count = 0;
static size_t s_preview_event_next = 0;

static void record_preview_event(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    preview_event_t *evt = &s_preview_events[s_preview_event_next];
    vsnprintf(evt->message, sizeof(evt->message), fmt, args);
    va_end(args);
    evt->timestamp_us = esp_timer_get_time();

    s_preview_event_next = (s_preview_event_next + 1) % PREVIEW_EVENT_MAX;
    if (s_preview_event_count < PREVIEW_EVENT_MAX) {
        s_preview_event_count++;
    }
}

#if CONFIG_IDF_TARGET_ESP32S3
static bool s_lcd_dims_logged = false;

static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

#define FONT_CHAR_WIDTH   6
#define FONT_CHAR_HEIGHT  8

static const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x10,0x08,0x08,0x10,0x08}, {0x78,0x46,0x41,0x46,0x78},
    {0x3C,0x26,0x23,0x26,0x3C}, {0x1C,0x36,0x6B,0x36,0x1C}, {0x08,0x1C,0x2A,0x08,0x08}, {0x08,0x08,0x2A,0x1C,0x08},
    {0x1E,0x21,0x21,0x12,0x3C}, {0x3C,0x12,0x21,0x21,0x1E}, {0x3C,0x32,0x29,0x26,0x3C}, {0x1E,0x21,0x21,0x21,0x12},
    {0x12,0x21,0x21,0x21,0x1E}, {0x00,0x2A,0x55,0x55,0x3E}, {0x3E,0x55,0x55,0x2A,0x00}, {0x22,0x14,0x08,0x14,0x22},
    {0x02,0x15,0x15,0x15,0x0E}, {0x0E,0x15,0x15,0x15,0x02}, {0x2A,0x1C,0x3E,0x1C,0x2A}, {0x7F,0x1C,0x1C,0x1C,0x1C},
    {0x08,0x3E,0x08,0x3E,0x08}, {0x14,0x14,0x14,0x14,0x14}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x08,0x0C,0x0A,0x08},
    {0x08,0x0A,0x0C,0x08,0x00}, {0x78,0x40,0x40,0x40,0x40}, {0x7C,0x7C,0x7C,0x7C,0x7C}, {0x00,0x00,0x00,0x00,0x00},
};

static void fill_rect(esp_lcd_panel_handle_t panel, int x0, int y0, int x1, int y1, uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > s_lcd_width) x1 = s_lcd_width;
    if (y1 > s_lcd_height) y1 = s_lcd_height;
    int width = x1 - x0;
    int height = y1 - y0;
    if (width <= 0 || height <= 0) {
        return;
    }

    const int chunk = 32;
    uint16_t line_buf[chunk];
    for (int i = 0; i < chunk; ++i) {
        line_buf[i] = color;
    }

    for (int y = y0; y < y1; ++y) {
        int remaining = width;
        int xs = x0;
        while (remaining > 0) {
            int draw = remaining < chunk ? remaining : chunk;
            esp_lcd_panel_draw_bitmap(panel, xs, y, xs + draw, y + 1, line_buf);
            remaining -= draw;
            xs += draw;
        }
    }
}

static void draw_char(esp_lcd_panel_handle_t panel, int x, int y, char c, uint16_t fg, uint16_t bg)
{
    if (x + FONT_CHAR_WIDTH > s_lcd_width || y + FONT_CHAR_HEIGHT > s_lcd_height) {
        return;
    }
    if (c < 32 || c > 127) {
        c = '?';
    }
    const uint8_t *glyph = font5x7[c - 32];
    uint16_t pixels[FONT_CHAR_WIDTH * FONT_CHAR_HEIGHT];
    for (int row = 0; row < FONT_CHAR_HEIGHT; ++row) {
        for (int col = 0; col < FONT_CHAR_WIDTH; ++col) {
            bool on = false;
            if (col < 5) {
                on = (glyph[col] >> row) & 0x01;
            }
            pixels[row * FONT_CHAR_WIDTH + col] = on ? fg : bg;
        }
    }
    esp_lcd_panel_draw_bitmap(panel, x, y, x + FONT_CHAR_WIDTH, y + FONT_CHAR_HEIGHT, pixels);
}

static void draw_text_line(esp_lcd_panel_handle_t panel, int x, int y, const char *text, uint16_t fg, uint16_t bg)
{
    int cursor = x;
    for (const char *p = text; *p != '\0' && cursor + FONT_CHAR_WIDTH <= s_lcd_width; ++p) {
        draw_char(panel, cursor, y, *p, fg, bg);
        cursor += FONT_CHAR_WIDTH;
    }
}

static void lcd_show_init_message(void)
{
    esp_lcd_panel_handle_t panel = panel_handle;
    if (panel == NULL) {
        ESP_LOGW(TAG, "LCD panel handle unavailable, skip init message");
        return;
    }

    ensure_lcd_dimensions();

    const uint16_t bg = rgb565(8, 8, 16);
    const uint16_t accent = rgb565(0, 180, 255);
    const uint16_t ready = rgb565(0, 200, 80);
    const uint16_t text = rgb565(200, 200, 200);

    fill_rect(panel, 0, 0, s_lcd_width, s_lcd_height, bg);

    const char *line1 = "ESP32-S3-BOX";
    const char *line2 = "LCD INIT COMPLETE";
    const char *line3 = "Ready to use";

    int line1_x = (s_lcd_width - (int)strlen(line1) * FONT_CHAR_WIDTH) / 2;
    int line2_x = (s_lcd_width - (int)strlen(line2) * FONT_CHAR_WIDTH) / 2;
    int line3_x = (s_lcd_width - (int)strlen(line3) * FONT_CHAR_WIDTH) / 2;

    if (line1_x < 0) {
        line1_x = 0;
    }
    if (line2_x < 0) {
        line2_x = 0;
    }
    if (line3_x < 0) {
        line3_x = 0;
    }

    int center_y = s_lcd_height / 2;
    int line_spacing = FONT_CHAR_HEIGHT + 6;

    draw_text_line(panel, line1_x, center_y - line_spacing, line1, accent, bg);
    draw_text_line(panel, line2_x, center_y, line2, ready, bg);
    draw_text_line(panel, line3_x, center_y + line_spacing, line3, text, bg);

    record_preview_event("LCD init message displayed");
}

static void render_preview_status_screen(esp_lcd_panel_handle_t panel)
{
    ensure_lcd_dimensions();
    fill_rect(panel, 0, 0, s_lcd_width, s_lcd_height, rgb565(0, 0, 0));

    const uint16_t header_fg = rgb565(0, 255, 0);
    const uint16_t text_fg = rgb565(200, 200, 200);
    const uint16_t text_bg = rgb565(0, 0, 0);

    draw_text_line(panel, 4, 4, "USB PREVIEW STATUS", header_fg, text_bg);

    size_t count = s_preview_event_count;
    if (count == 0) {
        draw_text_line(panel, 4, 4 + FONT_CHAR_HEIGHT + 4, "No events yet", text_fg, text_bg);
        return;
    }

    size_t start = (s_preview_event_next + PREVIEW_EVENT_MAX - count) % PREVIEW_EVENT_MAX;
    int y = 4 + FONT_CHAR_HEIGHT + 4;
    int lines_avail = (s_lcd_height - y) / (FONT_CHAR_HEIGHT + 2);
    if (lines_avail <= 0) {
        lines_avail = 1;
    }

    if ((size_t)lines_avail < count) {
        start = (start + count - lines_avail) % PREVIEW_EVENT_MAX;
        count = lines_avail;
    }

    for (size_t i = 0; i < count; ++i) {
        size_t idx = (start + i) % PREVIEW_EVENT_MAX;
        const preview_event_t *evt = &s_preview_events[idx];
        uint64_t ms = evt->timestamp_us / 1000ULL;
        char line[128];
        snprintf(line, sizeof(line), "%5llu.%03llu %s",
                 (unsigned long long)(ms / 1000ULL),
                 (unsigned long long)(ms % 1000ULL),
                 evt->message);
        draw_text_line(panel, 4, y, line, text_fg, text_bg);
        y += FONT_CHAR_HEIGHT + 2;
    }
}
#endif // CONFIG_IDF_TARGET_ESP32S3

void media_sys_show_lcd_ready_message(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    lcd_show_init_message();
#else
    (void)0;
#endif
}

static void ensure_lcd_dimensions(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    if (s_lcd_width != 0 && s_lcd_height != 0) {
        return;
    }

    if (lcd_dev.width > 0 && lcd_dev.height > 0) {
        s_lcd_width = lcd_dev.width;
        s_lcd_height = lcd_dev.height;
    } else {
        s_lcd_width = 320;
        s_lcd_height = 240;
    }
#else
    if (s_lcd_width != 0 && s_lcd_height != 0) {
        return;
    }

    lcd_cfg_t lcd_cfg = {0};
    if (get_lcd_cfg(&lcd_cfg) == 0 && lcd_cfg.width > 0 && lcd_cfg.height > 0) {
        s_lcd_width = lcd_cfg.width;
        s_lcd_height = lcd_cfg.height;
    } else {
        s_lcd_width = 320;
        s_lcd_height = 240;
    }
#endif
#if CONFIG_IDF_TARGET_ESP32S3
    if (!s_lcd_dims_logged) {
        record_preview_event("LCD size %dx%d", s_lcd_width, s_lcd_height);
        s_lcd_dims_logged = true;
    }
#endif
}

/**
 * @brief JPEG解码一张图片
 */
static int esp_jpeg_decoder_one_picture(uint8_t *input_buf, size_t len, uint8_t *output_buf)
{
    esp_err_t ret = ESP_OK;
    
    // JPEG解码配置
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)input_buf,
        .indata_size = len,
        .outbuf = (uint8_t *)(output_buf),
        .outbuf_size = current_width * current_height * sizeof(uint16_t),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 0,
        }
    };
    
    // JPEG解码
    esp_jpeg_image_output_t outimg;
    ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "JPEG decoded: %dpx x %dpx", outimg.width, outimg.height);
    }
    
    return ret;
}

/**
 * @brief 自适应JPG帧缓冲器
 */
static void adaptive_jpg_frame_buffer(size_t length)
{
    if (jpg_frame_buf1 != NULL) {
        free(jpg_frame_buf1);
    }
    if (jpg_frame_buf2 != NULL) {
        free(jpg_frame_buf2);
    }
    
    // 申请PSRAM内存
    jpg_frame_buf1 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf1 != NULL);
    jpg_frame_buf2 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf2 != NULL);
    
    // 创建PingPong缓冲区
    ESP_ERROR_CHECK(ppbuffer_create(ppbuffer_handle, jpg_frame_buf2, jpg_frame_buf1));
    if_ppbuffer_init = true;
    ESP_LOGI(TAG, "PingPong buffer created, size: %zu bytes", length);
}

/**
 * @brief USB摄像头帧回调函数
 */
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    static int frame_count = 0;
    static int64_t last_print_time = 0;
    
    // 检测分辨率变化
    if (current_width != frame->width || current_height != frame->height) {
        current_width = frame->width;
        current_height = frame->height;
        ESP_LOGI(TAG, "📐 Resolution changed: %dx%d", current_width, current_height);
        adaptive_jpg_frame_buffer(current_width * current_height * 2);
    }
    
    static void *jpeg_buffer = NULL;
    // 获取可写缓冲区
    if (ppbuffer_get_write_buf(ppbuffer_handle, &jpeg_buffer) != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  Failed to get write buffer (buffer full?)");
        return;
    }
    
    assert(jpeg_buffer != NULL);
    
    // JPEG解码
    if (esp_jpeg_decoder_one_picture((uint8_t *)frame->data, frame->data_bytes, jpeg_buffer) == ESP_OK) {
        // 通知缓冲区写完成
        ppbuffer_set_write_done(ppbuffer_handle);

        if (!s_preview_first_frame_seen) {
            s_preview_first_frame_seen = true;
            record_preview_event("First video frame decoded (%dx%d)", current_width, current_height);
        }
        
        // 【运动追踪】处理解码后的帧
        if (motion_tracker_is_running()) {
            motion_target_t target = {0};
            // 处理解码后的RGB565图像（内部自动转换灰度）
            if (motion_tracker_process_frame(jpeg_buffer, current_width, current_height, 2, &target) == ESP_OK) {
                // 如果检测到运动，更新云台位置
                if (target.detected) {
                    motion_tracker_update_gimbal(&target);
                }
            }
        }
        
        // 统计帧率（每秒打印一次）
        frame_count++;
        int64_t now = esp_timer_get_time();
        if (now - last_print_time > 1000000) {  // 1秒
            ESP_LOGI(TAG, "📹 Receiving frames: %d fps, JPEG size: %d bytes", 
                     frame_count, frame->data_bytes);
            frame_count = 0;
            last_print_time = now;
        }
    } else {
        ESP_LOGE(TAG, "❌ JPEG decode failed for frame size %d", frame->data_bytes);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void usb_display_task(void *arg)
{
    esp_lcd_panel_handle_t panel = panel_handle;
    if (panel == NULL) {
        ESP_LOGE(TAG, "LCD panel handle not available, stop display task");
        vTaskDelete(NULL);
        return;
    }

    ensure_lcd_dimensions();
    record_preview_event("LCD panel ready (%dx%d)", s_lcd_width, s_lcd_height);
    render_preview_status_screen(panel);

    uint16_t *lcd_buffer = NULL;
    int frame_count = 0;
    int fps = 0;
    int64_t last_report = esp_timer_get_time();
    int64_t last_status_render = esp_timer_get_time();

    while (1) {
        if (!if_ppbuffer_init) {
            int64_t now = esp_timer_get_time();
            if (now - last_status_render > 300000) {
                render_preview_status_screen(panel);
                last_status_render = now;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ppbuffer_get_read_buf(ppbuffer_handle, (void **)&lcd_buffer) == ESP_OK) {
            int draw_width = current_width;
            int draw_height = current_height;

            if (draw_width > 0 && draw_height > 0 && lcd_buffer != NULL) {
                if (draw_width > s_lcd_width || draw_height > s_lcd_height) {
                    ESP_LOGW(TAG, "Skip frame %dx%d larger than LCD %dx%d", draw_width, draw_height, s_lcd_width, s_lcd_height);
                    record_preview_event("Skip frame %dx%d (LCD %dx%d)", draw_width, draw_height, s_lcd_width, s_lcd_height);
                } else {
                    int x_start = (s_lcd_width - draw_width) / 2;
                    int y_start = (s_lcd_height - draw_height) / 2;
                    esp_lcd_panel_draw_bitmap(panel,
                                             x_start,
                                             y_start,
                                             x_start + draw_width,
                                             y_start + draw_height,
                                             lcd_buffer);
                }
            }

            ppbuffer_set_read_done(ppbuffer_handle);

            if (++frame_count == 30) {
                int64_t now = esp_timer_get_time();
                fps = (int)((int64_t)frame_count * 1000000 / (now - last_report));
                last_report = now;
                frame_count = 0;
                ESP_LOGI(TAG, "LCD preview fps: %d", fps);
                record_preview_event("Preview fps %d", fps);
            }
        } else {
            int64_t now = esp_timer_get_time();
            if (!s_preview_first_frame_seen && now - last_status_render > 300000) {
                render_preview_status_screen(panel);
                last_status_render = now;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

/**
 * @brief USB数据流状态回调函数
 */
static void usb_stream_state_changed_cb(usb_stream_state_t event, void *arg)
{
    switch(event) {
        case STREAM_CONNECTED:
            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "✅ USB CAMERA CONNECTED!");
            ESP_LOGI(TAG, "========================================");
            record_preview_event("Camera connected");
            s_preview_first_frame_seen = false;

            ensure_lcd_dimensions();

            size_t frame_list_num = 0;
            uvc_frame_size_list_get(NULL, &frame_list_num, NULL);
            if (frame_list_num > 0) {
                uvc_frame_size_t *frame_list = (uvc_frame_size_t *)malloc(frame_list_num * sizeof(uvc_frame_size_t));
                if (frame_list) {
                    uvc_frame_size_list_get(frame_list, NULL, NULL);
                    size_t selected = frame_list_num;  // invalid index
                    uint32_t best_area = 0;
                    for (size_t i = 0; i < frame_list_num; ++i) {
                        uint32_t w = frame_list[i].width;
                        uint32_t h = frame_list[i].height;
                        ESP_LOGI(TAG, "  candidate[%d]: %ux%u", (int)i, (unsigned)w, (unsigned)h);
                        if (w <= s_lcd_width && h <= s_lcd_height) {
                            uint32_t area = w * h;
                            if (area > best_area) {
                                best_area = area;
                                selected = i;
                            }
                        }
                    }
                    if (selected == frame_list_num) {
                        ESP_LOGW(TAG, "No resolution fits LCD %ux%u, fallback to camera default", s_lcd_width, s_lcd_height);
                        record_preview_event("No matching camera resolution (LCD %dx%d)", s_lcd_width, s_lcd_height);
                    } else {
                        uint32_t target_w = frame_list[selected].width;
                        uint32_t target_h = frame_list[selected].height;
                        ESP_LOGI(TAG, "Selecting %ux%u @ %dfps", (unsigned)target_w, (unsigned)target_h, VIDEO_FPS);
                        record_preview_event("Request camera %ux%u@%dfps", (unsigned)target_w, (unsigned)target_h, VIDEO_FPS);
                        esp_err_t ret = uvc_frame_size_reset(target_w, target_h, FPS2INTERVAL(VIDEO_FPS));
                        if (ret != ESP_OK) {
                            ESP_LOGW(TAG, "uvc_frame_size_reset failed: %s", esp_err_to_name(ret));
                            record_preview_event("uvc_frame_size_reset failed: %s", esp_err_to_name(ret));
                        }
                    }
                    free(frame_list);
                }
            } else {
                ESP_LOGW(TAG, "Camera did not report resolution list");
            }

            ESP_LOGI(TAG, "💾 Memory status:");
            ESP_LOGI(TAG, "  Heap free: %lu bytes", (unsigned long)esp_get_free_heap_size());
            ESP_LOGI(TAG, "  PSRAM free: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

            if_ppbuffer_init = false;
            current_width = 0;
            current_height = 0;

            if (usb_streaming_control(STREAM_UVC, CTRL_RESUME, NULL) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to resume USB stream");
                record_preview_event("USB stream resume failed");
            }
            break;
            
        case STREAM_DISCONNECTED:
            ESP_LOGW(TAG, "========================================");
            ESP_LOGW(TAG, "⚠️  USB CAMERA DISCONNECTED!");
            ESP_LOGW(TAG, "========================================");
            record_preview_event("Camera disconnected");
            if_ppbuffer_init = false;
            current_width = 0;
            current_height = 0;
            s_preview_first_frame_seen = false;
            break;
            
        default:
            ESP_LOGD(TAG, "USB stream state: %d", event);
            break;
    }
}

static esp_err_t usb_camera_ensure_initialized(void)
{
#if !CONFIG_IDF_TARGET_ESP32S3
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_usb_camera_initialized) {
        return ESP_OK;
    }

    esp_err_t ret = ESP_OK;
    record_preview_event("Allocating USB buffers");

    if (xfer_buffer_a == NULL) {
        xfer_buffer_a = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (xfer_buffer_b == NULL) {
        xfer_buffer_b = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (frame_buffer == NULL) {
        frame_buffer = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (ppbuffer_handle == NULL) {
        ppbuffer_handle = (PingPongBuffer_t *)malloc(sizeof(PingPongBuffer_t));
    }

    if (xfer_buffer_a == NULL || xfer_buffer_b == NULL || frame_buffer == NULL || ppbuffer_handle == NULL) {
        ESP_LOGE(TAG, "USB camera buffer allocation failed");
        ret = ESP_ERR_NO_MEM;
        record_preview_event("USB buffer allocation failed");
        goto fail;
    }

    uvc_config_t uvc_config = {
        .frame_interval = FPS2INTERVAL(VIDEO_FPS),
        .xfer_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .xfer_buffer_a = xfer_buffer_a,
        .xfer_buffer_b = xfer_buffer_b,
        .frame_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .frame_buffer = frame_buffer,
        .frame_cb = &camera_frame_cb,
        .frame_cb_arg = NULL,
        .frame_width = FRAME_RESOLUTION_ANY,
        .frame_height = FRAME_RESOLUTION_ANY,
        .flags = FLAG_UVC_SUSPEND_AFTER_START,
    };

    ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uvc_streaming_config failed: %s", esp_err_to_name(ret));
        record_preview_event("uvc_streaming_config failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    record_preview_event("USB streaming configured");
    record_preview_event("Waiting for camera connection");

    if (!s_usb_state_cb_registered) {
        ret = usb_streaming_state_register(&usb_stream_state_changed_cb, NULL);
        if (ret == ESP_OK) {
            s_usb_state_cb_registered = true;
            record_preview_event("USB state callback registered");
        } else if (ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "usb_streaming_state_register failed: %s", esp_err_to_name(ret));
            record_preview_event("usb_streaming_state_register failed: %s", esp_err_to_name(ret));
            goto fail;
        }
    }

    ret = usb_streaming_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "usb_streaming_start returned %s", esp_err_to_name(ret));
        record_preview_event("usb_streaming_start returned %s", esp_err_to_name(ret));
    }

    if (s_usb_display_task == NULL) {
        if (xTaskCreate(usb_display_task, "usb_display", 4096, NULL, 5, &s_usb_display_task) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create USB display task");
            record_preview_event("Create display task failed");
            ret = ESP_FAIL;
            goto fail;
        }
    }

    s_usb_camera_initialized = true;
    record_preview_event("USB preview pipeline ready");
    ESP_LOGI(TAG, "USB camera preview pipeline ready");
    return ESP_OK;

fail:
    if (s_usb_display_task) {
        vTaskDelete(s_usb_display_task);
        s_usb_display_task = NULL;
    }
    if_ppbuffer_init = false;
    s_usb_camera_initialized = false;
    s_preview_first_frame_seen = false;
    return ret;
#endif
}

/**
 * @brief 创建USB摄像头视频源 (ESP32-S3专用)
 * 
 * 为ESP32-S3外接USB摄像头创建视频捕获源
 * 
 * @return esp_capture_video_src_if_t* 成功返回视频源接口指针，失败返回NULL
 */
static esp_capture_video_src_if_t *create_usb_video_source(void)
{
    esp_err_t ret = usb_camera_ensure_initialized();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB camera initialization failed with error 0x%x", ret);
        return NULL;
    }

    ESP_LOGI(TAG, "USB camera initialized, awaiting capture integration ...");
    // TODO: Implement esp_capture video source bridge for UVC stream
    return NULL;
}
#endif

/**
 * @brief 创建视频捕获源
 * 
 * 根据硬件平台选择合适的视频源:
 * - ESP32-S3: 优先USB摄像头，失败则回退到DVP摄像头
 * - ESP32-P4: 支持MIPI CSI或DVP摄像头
 * 
 * @return esp_capture_video_src_if_t* 成功返回视频源接口指针，失败返回NULL
 */
static esp_capture_video_src_if_t *create_video_source(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    // For ESP32-S3, the USB source is now the primary video source
    return create_usb_video_source();
#elif CONFIG_IDF_TARGET_ESP32P4
    camera_cfg_t cam_pin_cfg = {};
    int ret = get_camera_cfg(&cam_pin_cfg);
    if (ret != 0) {
        return NULL;
    }
    esp_video_init_csi_config_t csi_config = { 0 };
    esp_video_init_dvp_config_t dvp_config = { 0 };
    esp_video_init_config_t cam_config = { 0 };
    if (cam_pin_cfg.type == CAMERA_TYPE_MIPI) {
        csi_config.sccb_config.i2c_handle = get_i2c_bus_handle(0);
        csi_config.sccb_config.freq = 100000;
        csi_config.reset_pin = cam_pin_cfg.reset;
        csi_config.pwdn_pin = cam_pin_cfg.pwr;
        ESP_LOGI(TAG, "Use i2c handle %p", csi_config.sccb_config.i2c_handle);
        cam_config.csi = &csi_config;
    } else if (cam_pin_cfg.type == CAMERA_TYPE_DVP) {
        dvp_config.reset_pin = cam_pin_cfg.reset;
        dvp_config.pwdn_pin = cam_pin_cfg.pwr;
        dvp_config.dvp_pin.data_width = CAM_CTLR_DATA_WIDTH_8;
        dvp_config.dvp_pin.data_io[0] = cam_pin_cfg.data[0];
        dvp_config.dvp_pin.data_io[1] = cam_pin_cfg.data[1];
        dvp_config.dvp_pin.data_io[2] = cam_pin_cfg.data[2];
        dvp_config.dvp_pin.data_io[3] = cam_pin_cfg.data[3];
        dvp_config.dvp_pin.data_io[4] = cam_pin_cfg.data[4];
        dvp_config.dvp_pin.data_io[5] = cam_pin_cfg.data[5];
        dvp_config.dvp_pin.data_io[6] = cam_pin_cfg.data[6];
        dvp_config.dvp_pin.data_io[7] = cam_pin_cfg.data[7];
        dvp_config.dvp_pin.vsync_io = cam_pin_cfg.vsync;
        dvp_config.dvp_pin.pclk_io = cam_pin_cfg.pclk;
        dvp_config.dvp_pin.xclk_io = cam_pin_cfg.xclk;
        dvp_config.dvp_pin.de_io = cam_pin_cfg.de;
        dvp_config.xclk_freq = 20000000;
        cam_config.dvp = &dvp_config;
    }
    ret = esp_video_init(&cam_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", ret);
        return NULL;
    }
    esp_capture_video_v4l2_src_cfg_t v4l2_cfg = {
        .dev_name = "/dev/video0",
        .buf_count = 2,
    };
    return esp_capture_new_video_v4l2_src(&v4l2_cfg);
#else
    return NULL;
#endif
}

/**
 * @brief 构建音视频捕获系统
 * 
 * 创建并初始化视频源和音频源，将它们整合到捕获系统中
 * 
 * @return int 成功返回0，失败返回-1
 */
static int build_capture_system(void)
{
    // For ESP32-S3, use USB camera as video source
#if CONFIG_IDF_TARGET_ESP32S3
    capture_sys.vid_src = create_usb_video_source();
    if (capture_sys.vid_src == NULL) {
        ESP_LOGW(TAG, "Failed to create USB video source, continuing with audio only.");
    }
#else
    // For other targets, use the original logic
    capture_sys.vid_src = create_video_source();
#endif

    esp_capture_audio_dev_src_cfg_t codec_cfg = {
        .record_handle = get_record_handle(),
    };
    capture_sys.aud_src = esp_capture_new_audio_dev_src(&codec_cfg);
    RET_ON_NULL(capture_sys.aud_src, -1);

    esp_capture_cfg_t cfg = {
        .sync_mode = ESP_CAPTURE_SYNC_MODE_AUDIO,
        .audio_src = capture_sys.aud_src,
        .video_src = capture_sys.vid_src, // Use the created video source
    };
    esp_capture_open(&cfg, &capture_sys.capture_handle);
    return 0;
}

/**
 * @brief 构建音视频播放系统
 * 
 * 创建音频渲染器(I2S)、视频渲染器(LCD)以及AV播放器
 * 
 * @return int 成功返回0，失败返回-1
 */
static int build_player_system()
{
    // 创建I2S音频渲染器
    i2s_render_cfg_t i2s_cfg = {
        .fixed_clock = true,
        .play_handle = get_playback_handle(),
    };
    player_sys.audio_render = av_render_alloc_i2s_render(&i2s_cfg);
    if (player_sys.audio_render == NULL) {
        ESP_LOGE(TAG, "Fail to create audio render");
        return -1;
    }
    
    // 创建LCD视频渲染器
    lcd_render_cfg_t lcd_cfg = {
    #if CONFIG_IDF_TARGET_ESP32S3
        .lcd_handle = panel_handle,
    #else
        .lcd_handle = NULL,
    #endif
    };
    player_sys.video_render = av_render_alloc_lcd_render(&lcd_cfg);

    if (player_sys.video_render == NULL) {
        ESP_LOGE(TAG, "Fail to create video render");
        // 允许没有视频显示
    }
    
    // 创建AV播放器，整合音视频渲染器
    av_render_cfg_t render_cfg = {
        .audio_render = player_sys.audio_render,
        .video_render = player_sys.video_render,
        .audio_raw_fifo_size = 4096,
        .audio_render_fifo_size = 6 * 1024,
        .video_raw_fifo_size = 500 * 1024,
        .allow_drop_data = false,  // 不允许丢帧
        //.video_render_fifo_size = 4*1024,
    };
    player_sys.player = av_render_open(&render_cfg);
    if (player_sys.player == NULL) {
        ESP_LOGE(TAG, "Fail to create player");
        return -1;
    }
    return 0;
}

/**
 * @brief 初始化媒体系统
 * 
 * 注册默认的音视频编解码器，构建捕获和播放系统
 * 这是媒体系统的总入口函数
 * 
 * @return int 成功返回0，失败返回-1
 */
int media_sys_buildup(void)
{
    // 注册默认的音视频编解码器
    esp_video_enc_register_default();  // 注册视频编码器
    esp_audio_enc_register_default();  // 注册音频编码器
    esp_video_dec_register_default();  // 注册视频解码器
    esp_audio_dec_register_default();  // 注册音频解码器
    
    // 构建捕获系统（摄像头+麦克风）
    build_capture_system();
    // 构建播放系统（LCD+扬声器）
    build_player_system();
    return 0;
}

/**
 * @brief 获取媒体提供者接口
 * 
 * 供WebRTC使用，提供捕获和播放系统的句柄
 * 
 * @param provide 输出参数，填充捕获和播放句柄
 * @return int 成功返回0
 */
int media_sys_get_provider(esp_webrtc_media_provider_t *provide)
{
    provide->capture = capture_sys.capture_handle;
    provide->player = player_sys.player;
    return 0;
}

esp_err_t media_sys_start_usb_preview(void)
{
#if CONFIG_IDF_TARGET_ESP32S3
    if (panel_handle == NULL) {
        ESP_LOGW(TAG, "LCD handle unavailable, skip USB preview");
        record_preview_event("LCD handle unavailable, preview skipped");
        return ESP_ERR_INVALID_STATE;
    }
    s_preview_event_count = 0;
    s_preview_event_next = 0;
    s_preview_first_frame_seen = false;
    s_lcd_dims_logged = false;
    record_preview_event("Starting USB preview init");
    return usb_camera_ensure_initialized();
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

void media_sys_dump_usb_preview_events(void)
{
    if (s_preview_event_count == 0) {
        ESP_LOGI(TAG, "No USB preview events recorded");
        return;
    }

    size_t count = s_preview_event_count;
    size_t start = (s_preview_event_next + PREVIEW_EVENT_MAX - count) % PREVIEW_EVENT_MAX;
    for (size_t i = 0; i < count; ++i) {
        size_t idx = (start + i) % PREVIEW_EVENT_MAX;
        const preview_event_t *evt = &s_preview_events[idx];
        uint64_t ms = evt->timestamp_us / 1000ULL;
        ESP_LOGI(TAG, "[preview][%llu.%03llu s] %s",
                 (unsigned long long)(ms / 1000ULL),
                 (unsigned long long)(ms % 1000ULL),
                 evt->message);
    }
}

int test_capture_to_player(void)
{
    esp_capture_sink_cfg_t sink_cfg = {
        .audio_info = {
            .format_id = ESP_CAPTURE_FMT_ID_G711A,
            .sample_rate = 8000,
            .channel = 1,
            .bits_per_sample = 16,
        },
        .video_info = { .format_id = ESP_CAPTURE_FMT_ID_MJPEG, .width = VIDEO_WIDTH, .height = VIDEO_HEIGHT, .fps = 20 },
    };
    // Create capture
    esp_capture_sink_handle_t capture_path = NULL;
    esp_capture_sink_setup(capture_sys.capture_handle, 0, &sink_cfg, &capture_path);
    esp_capture_sink_enable(capture_path, ESP_CAPTURE_RUN_MODE_ALWAYS);
    // Create player
    av_render_audio_info_t render_aud_info = {
        .codec = AV_RENDER_AUDIO_CODEC_G711A,
        .sample_rate = 8000,
        .channel = 1,
    };
    av_render_add_audio_stream(player_sys.player, &render_aud_info);

    av_render_video_info_t render_vid_info = {
        .codec = AV_RENDER_VIDEO_CODEC_MJPEG,
    };
    av_render_add_video_stream(player_sys.player, &render_vid_info);
    uint32_t start_time = (uint32_t)(esp_timer_get_time() / 1000);
    esp_capture_start(capture_sys.capture_handle);
    while ((uint32_t)(esp_timer_get_time() / 1000) < start_time + 2000) {
        media_lib_thread_sleep(30);
        esp_capture_stream_frame_t frame = {
            .stream_type = ESP_CAPTURE_STREAM_TYPE_AUDIO,
        };
        while (esp_capture_sink_acquire_frame(capture_path, &frame, true) == ESP_CAPTURE_ERR_OK) {
            av_render_audio_data_t audio_data = {
                .data = frame.data,
                .size = frame.size,
                .pts = frame.pts,
            };
            av_render_add_audio_data(player_sys.player, &audio_data);
            esp_capture_sink_release_frame(capture_path, &frame);
        }
        frame.stream_type = ESP_CAPTURE_STREAM_TYPE_VIDEO;
        while (esp_capture_sink_acquire_frame(capture_path, &frame, true) == ESP_CAPTURE_ERR_OK) {
            av_render_video_data_t video_data = {
                .data = frame.data,
                .size = frame.size,
                .pts = frame.pts,
            };
            av_render_add_video_data(player_sys.player, &video_data);
            esp_capture_sink_release_frame(capture_path, &frame);
        }
    }
    esp_capture_stop(capture_sys.capture_handle);
    av_render_reset(player_sys.player);
    return 0;
}

static void music_play_thread(void *arg)
{
    // Suppose all music is AAC
    av_render_audio_info_t render_aud_info = {
        .codec = AV_RENDER_AUDIO_CODEC_AAC,
    };
    av_render_add_audio_stream(player_sys.player, &render_aud_info);
    int music_pos = 0;
    while (!music_stopping && music_duration >= 0) {
        uint32_t start_time = esp_timer_get_time() / 1000;
        int send_size = music_size - music_pos;
        const uint8_t *adts_header = music_to_play + music_pos;
        if (adts_header[0] != 0xFF) {
            send_size = 0;
        } else {
            int frame_size = ((adts_header[3] & 0x03) << 11) | (adts_header[4] << 3) | (adts_header[5] >> 5);
            if (frame_size < send_size) {
                send_size = frame_size;
            }
        }
        if (send_size) {
            av_render_audio_data_t audio_data = {
                .data = (uint8_t *)adts_header,
                .size = send_size,
            };
            int ret = av_render_add_audio_data(player_sys.player, &audio_data);
            if (ret != 0) {
                break;
            }
            music_pos += send_size;
        }
        if (music_pos >= music_size || send_size == 0) {
            music_pos = 0;
            // Play one loop only
            if (music_duration == 0) {
                av_render_fifo_stat_t stat = { 0 };
                while (!music_stopping) {
                    av_render_get_audio_fifo_level(player_sys.player, &stat);
                    if (stat.data_size > 0) {
                        media_lib_thread_sleep(50);
                        continue;
                    }
                    break;
                }
                break;
            }
        }
        uint32_t end_time = esp_timer_get_time() / 1000;
        if (music_duration) {
            music_duration -= end_time - start_time;
        }
    }
    av_render_reset(player_sys.player);
    music_stopping = false;
    music_playing = false;
    media_lib_thread_destroy(NULL);
}

int play_music(const uint8_t *data, int size, int duration)
{
    if (music_playing) {
        ESP_LOGE(TAG, "Music is playing, stop automatically");
        stop_music();
    }
    music_playing = true;
    music_to_play = data;
    music_size = size;
    music_duration = duration;
    media_lib_thread_handle_t thread;
    int ret = media_lib_thread_create_from_scheduler(&thread, "music_player", music_play_thread, NULL);
    if (ret != 0) {
        music_playing = false;
        ESP_LOGE(TAG, "Fail to create music_player thread");
        return ret;
    }
    return 0;
}

int stop_music()
{
    if (music_playing) {
        music_stopping = true;
        while (music_stopping) {
            media_lib_thread_sleep(20);
        }
    }
    return 0;
}
