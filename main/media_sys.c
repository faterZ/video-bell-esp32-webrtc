/* Media system

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "codec_init.h"
#include "codec_board.h"
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
        
        // 【运动追踪】处理解码后的帧
        if (motion_tracker_is_running()) {
            motion_target_t target = {0};
            // 处理解码后的RGB888图像
            if (motion_tracker_process_frame(jpeg_buffer, current_width, current_height, 3, &target) == ESP_OK) {
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
            
            // 获取支持的分辨率列表
            size_t frame_list_num = 0;
            uvc_frame_size_list_get(NULL, &frame_list_num, NULL);
            
            if (frame_list_num > 0) {
                ESP_LOGI(TAG, "📋 UVC: Found %u supported resolutions:", frame_list_num);
                uvc_frame_size_t *frame_list = (uvc_frame_size_t *)malloc(frame_list_num * sizeof(uvc_frame_size_t));
                if (frame_list) {
                    uvc_frame_size_list_get(frame_list, NULL, NULL);
                    // 打印支持的分辨率
                    bool found_720p = false;
                    for (size_t i = 0; i < frame_list_num; i++) {
                        ESP_LOGI(TAG, "  [%d] %dx%d", i,
                                frame_list[i].width, frame_list[i].height);
                        if (frame_list[i].width == 1280 && frame_list[i].height == 720) {
                            ESP_LOGI(TAG, "     ✅ 720p SUPPORTED");
                            found_720p = true;
                        }
                    }
                    if (!found_720p) {
                        ESP_LOGW(TAG, "⚠️  720p not found in supported list, may use default resolution");
                    }
                    free(frame_list);
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for resolution list");
                }
            } else {
                ESP_LOGW(TAG, "No resolution list available");
            }
            
            ESP_LOGI(TAG, "💾 Memory status:");
            ESP_LOGI(TAG, "  Heap free: %lu bytes", (unsigned long)esp_get_free_heap_size());
            ESP_LOGI(TAG, "  PSRAM free: %lu bytes", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            break;
            
        case STREAM_DISCONNECTED:
            ESP_LOGW(TAG, "========================================");
            ESP_LOGW(TAG, "⚠️  USB CAMERA DISCONNECTED!");
            ESP_LOGW(TAG, "========================================");
            break;
            
        default:
            ESP_LOGD(TAG, "USB stream state: %d", event);
            break;
    }
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
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "🚀 Initializing USB camera...");
    ESP_LOGI(TAG, "========================================");
    
    // 分配USB传输缓冲区
    ESP_LOGI(TAG, "📦 Allocating USB buffers (%d KB each)...", DEMO_UVC_XFER_BUFFER_SIZE / 1024);
    xfer_buffer_a = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    xfer_buffer_b = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    frame_buffer = (uint8_t *)heap_caps_aligned_alloc(16, DEMO_UVC_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    
    if (!xfer_buffer_a || !xfer_buffer_b || !frame_buffer) {
        ESP_LOGE(TAG, "❌ Failed to allocate USB buffers");
        ESP_LOGE(TAG, "   xfer_a: %s, xfer_b: %s, frame: %s",
                 xfer_buffer_a ? "OK" : "FAIL",
                 xfer_buffer_b ? "OK" : "FAIL",
                 frame_buffer ? "OK" : "FAIL");
        goto error;
    }
    ESP_LOGI(TAG, "✅ USB buffers allocated");
    
    // 创建PingPong缓冲区句柄
    ESP_LOGI(TAG, "📦 Creating PingPong buffer...");
    ppbuffer_handle = (PingPongBuffer_t *)malloc(sizeof(PingPongBuffer_t));
    if (!ppbuffer_handle) {
        ESP_LOGE(TAG, "❌ Failed to create ppbuffer handle");
        goto error;
    }
    ESP_LOGI(TAG, "✅ PingPong buffer handle created");
    
    // 配置UVC流
    ESP_LOGI(TAG, "⚙️  Configuring UVC stream (1280x720 @ 15fps)...");
    uvc_config_t uvc_config = {
        .frame_interval = FRAME_INTERVAL_FPS_15,  // 15fps
        .xfer_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .xfer_buffer_a = xfer_buffer_a,
        .xfer_buffer_b = xfer_buffer_b,
        .frame_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .frame_buffer = frame_buffer,
        .frame_cb = &camera_frame_cb,
        .frame_cb_arg = NULL,
        .frame_width = 1280,   // 从settings.h读取
        .frame_height = 720,
        .flags = FLAG_UVC_SUSPEND_AFTER_START,
    };
    
    esp_err_t ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ UVC streaming config failed: %s (0x%x)", 
                 esp_err_to_name(ret), ret);
        goto error;
    }
    ESP_LOGI(TAG, "✅ UVC streaming configured");
    
    // 注册USB流状态回调
    ESP_LOGI(TAG, "🔔 Registering USB state callback...");
    usb_streaming_state_register(&usb_stream_state_changed_cb, NULL);
    ESP_LOGI(TAG, "✅ USB state callback registered");
    
    // 启动USB流（非阻塞模式，在后台等待摄像头插入）
    ESP_LOGI(TAG, "▶️  Starting USB streaming in background...");
    ESP_LOGI(TAG, "   USB camera will be detected automatically when plugged in");
    ret = usb_streaming_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️  USB streaming start returned: %s (0x%x)", 
                 esp_err_to_name(ret), ret);
        ESP_LOGW(TAG, "   This is normal if no camera is connected yet");
        ESP_LOGW(TAG, "   Camera will be initialized when you plug it in");
    } else {
        ESP_LOGI(TAG, "✅ USB streaming started successfully!");
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ USB camera system ready!");
    ESP_LOGI(TAG, "   Plug in camera to start streaming");
    ESP_LOGI(TAG, "========================================");
    
    // TODO: 创建esp_capture接口适配层
    // 目前返回NULL，后续对接
    return NULL;
    
error:
    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "❌ USB camera initialization FAILED");
    ESP_LOGE(TAG, "========================================");
    if (xfer_buffer_a) free(xfer_buffer_a);
    if (xfer_buffer_b) free(xfer_buffer_b);
    if (frame_buffer) free(frame_buffer);
    if (ppbuffer_handle) free(ppbuffer_handle);
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
    // ESP32-S3优先尝试USB摄像头
    esp_capture_video_src_if_t *usb_src = create_usb_video_source();
    if (usb_src != NULL) {
        ESP_LOGI(TAG, "Using USB camera as video source");
        return usb_src;
    }
    ESP_LOGI(TAG, "USB camera not available, falling back to DVP camera");
#endif

    camera_cfg_t cam_pin_cfg = {};
    int ret = get_camera_cfg(&cam_pin_cfg);
    if (ret != 0) {
        return NULL;
    }
#if CONFIG_IDF_TARGET_ESP32P4
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
#endif

#if CONFIG_IDF_TARGET_ESP32S3
    if (cam_pin_cfg.type == CAMERA_TYPE_DVP) {
        esp_capture_video_dvp_src_cfg_t dvp_config = { 0 };
        dvp_config.buf_count = 2;
        dvp_config.reset_pin = cam_pin_cfg.reset;
        dvp_config.pwr_pin = cam_pin_cfg.pwr;
        dvp_config.data[0] = cam_pin_cfg.data[0];
        dvp_config.data[1] = cam_pin_cfg.data[1];
        dvp_config.data[2] = cam_pin_cfg.data[2];
        dvp_config.data[3] = cam_pin_cfg.data[3];
        dvp_config.data[4] = cam_pin_cfg.data[4];
        dvp_config.data[5] = cam_pin_cfg.data[5];
        dvp_config.data[6] = cam_pin_cfg.data[6];
        dvp_config.data[7] = cam_pin_cfg.data[7];
        dvp_config.vsync_pin = cam_pin_cfg.vsync;
        dvp_config.href_pin = cam_pin_cfg.href;
        dvp_config.pclk_pin = cam_pin_cfg.pclk;
        dvp_config.xclk_pin = cam_pin_cfg.xclk;
        dvp_config.xclk_freq = 20000000;
        return esp_capture_new_video_dvp_src(&dvp_config);
    }
#endif
    return NULL;
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
    // 创建视频源
    capture_sys.vid_src = create_video_source();
    RET_ON_NULL(capture_sys.vid_src, -1);

    // 创建音频源（从codec设备）
    esp_capture_audio_dev_src_cfg_t codec_cfg = {
        .record_handle = get_record_handle(),
    };
    capture_sys.aud_src = esp_capture_new_audio_dev_src(&codec_cfg);
    RET_ON_NULL(capture_sys.aud_src, -1);

    // 创建捕获系统，音视频同步模式
    esp_capture_cfg_t cfg = {
        .sync_mode = ESP_CAPTURE_SYNC_MODE_AUDIO,  // 以音频为同步基准
        .audio_src = capture_sys.aud_src,
        .video_src = capture_sys.vid_src,
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
        .lcd_handle = board_get_lcd_handle(),
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
