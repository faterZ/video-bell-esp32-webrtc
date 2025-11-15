/* Door Bell Demo

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_webrtc.h"
#include "media_lib_adapter.h"
#include "media_lib_os.h"
#include "esp_timer.h"
#include "webrtc_utils_time.h"
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "settings.h"
#include "common.h"
#include "esp_capture.h"
#include "servo_control.h"
#include "ppbuffer.h"
#include "usb_stream.h"
#include "jpeg_decoder.h"
#include "motion_tracker.h"
#include "codec_init.h"
#include "esp_bit_defs.h"
#include "xl9555.h"
#include "led.h"
#include "lcd.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"
#include "nvs.h"

bool webrtc_is_active(void);
void webrtc_trigger_ring(void);
bool webrtc_is_peer_connected(void);
bool webrtc_is_ringing(void);

static const char *TAG = "Webrtc_Test";
static bool s_webrtc_auto_started = false;
static TaskHandle_t s_auto_ring_task_handle = NULL;
static volatile bool s_auto_ring_stop = false;

#define ENABLE_AUTO_WEBRTC_JOIN 1

#define AUTO_RING_INITIAL_DELAY_MS 30000
#define AUTO_RING_RETRY_DELAY_MS   15000
#define AUTO_RING_MAX_ATTEMPTS     5

// USB Camera variables (from 29_usb_camera example)
#define DEMO_UVC_XFER_BUFFER_SIZE (88 * 1024)
#define BIT0_FRAME_START (0x01 << 0)
#define DEMO_KEY_RESOLUTION "resolution"
static EventGroupHandle_t s_evt_handle = NULL;
static uint8_t *jpg_frame_buf1 = NULL;
static uint8_t *jpg_frame_buf2 = NULL;
static uint8_t *xfer_buffer_a = NULL;
static uint8_t *xfer_buffer_b = NULL;
static uint8_t *frame_buffer = NULL;
static PingPongBuffer_t *ppbuffer_handle = NULL;
static uint16_t current_width = 0;
static uint16_t current_height = 0;
static bool if_ppbuffer_init = false;
static uint16_t lcd_log_cursor = 0;
static bool lcd_log_active = false;
static bool s_usb_device_ready = false;
static TaskHandle_t s_usb_wait_task = NULL;
extern uint32_t g_back_color;

static void lcd_log_reset(uint16_t bg_color)
{
    if (panel_handle == NULL) {
        return;
    }
    g_back_color = bg_color;
    lcd_fill(0, 0, lcd_dev.width, lcd_dev.height, bg_color);
    lcd_log_cursor = 0;
    lcd_log_active = true;
}

static void lcd_log_line(const char *fmt, ...)
{
    if (!lcd_log_active || panel_handle == NULL) {
        return;
    }
    const uint8_t font_size = 16;
    if (lcd_log_cursor + font_size > lcd_dev.height) {
        lcd_fill(0, 0, lcd_dev.width, lcd_dev.height, BLACK);
        lcd_log_cursor = 0;
    }

    char line[96];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    lcd_show_string(4,
                    lcd_log_cursor,
                    lcd_dev.width - 8,
                    font_size,
                    font_size,
                    line,
                    WHITE);
    lcd_log_cursor += font_size + 2;
}

static void usb_wait_screen_task(void *arg)
{
    int seconds = 0;
    while (!s_usb_device_ready) {
        lcd_log_line("Waiting for camera... (%ds)", seconds++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    lcd_log_line("Camera connected");
    lcd_log_active = false;
    s_usb_wait_task = NULL;
    vTaskDelete(NULL);
}

typedef struct {
    uint16_t width;
    uint16_t height;
} camera_frame_size_t;

typedef struct {
    camera_frame_size_t camera_frame_size;
    uvc_frame_size_t *camera_frame_list;
    size_t camera_frame_list_num;
    size_t camera_currect_frame_index;
} camera_resolution_info_t;

static camera_resolution_info_t camera_resolution_info = {0};

static void stop_auto_ring_task(void);

static void auto_ring_task(void *arg)
{
    const TickType_t check_interval = pdMS_TO_TICKS(1000);
    TickType_t waited = 0;
    const TickType_t initial_delay = pdMS_TO_TICKS(AUTO_RING_INITIAL_DELAY_MS);

    while (!s_auto_ring_stop && waited < initial_delay) {
        vTaskDelay(check_interval);
        waited += check_interval;
    }

    int attempt = 0;
    while (!s_auto_ring_stop && attempt < AUTO_RING_MAX_ATTEMPTS) {
        while (!s_auto_ring_stop && !webrtc_is_active()) {
            vTaskDelay(check_interval);
        }

        if (s_auto_ring_stop || !webrtc_is_active()) {
            break;
        }

        if (webrtc_is_peer_connected()) {
            break;
        }

        attempt++;
        ESP_LOGI(TAG, "🔔 Auto doorbell ring attempt #%d", attempt);
        webrtc_trigger_ring();

        TickType_t elapsed = 0;
        const TickType_t retry_window = pdMS_TO_TICKS(AUTO_RING_RETRY_DELAY_MS);
        while (!s_auto_ring_stop && !webrtc_is_peer_connected() && elapsed < retry_window) {
            vTaskDelay(check_interval);
            elapsed += check_interval;
        }
    }

    s_auto_ring_task_handle = NULL;
    s_auto_ring_stop = false;
    ESP_LOGI(TAG, "Auto doorbell task finished");
    vTaskDelete(NULL);
}

static void start_auto_ring_task(void)
{
    if (s_auto_ring_task_handle != NULL) {
        return;
    }
    s_auto_ring_stop = false;
    if (xTaskCreate(auto_ring_task, "auto_ring", 3072, NULL, 4, &s_auto_ring_task_handle) != pdPASS) {
        ESP_LOGW(TAG, "Failed to start auto doorbell task");
        s_auto_ring_task_handle = NULL;
    }
}

static void stop_auto_ring_task(void)
{
    if (s_auto_ring_task_handle == NULL) {
        return;
    }

    s_auto_ring_stop = true;
    const TickType_t wait_tick = pdMS_TO_TICKS(50);
    for (int i = 0; i < 60 && s_auto_ring_task_handle != NULL; ++i) {
        vTaskDelay(wait_tick);
    }

    if (s_auto_ring_task_handle != NULL) {
        vTaskDelete(s_auto_ring_task_handle);
        s_auto_ring_task_handle = NULL;
    }
    s_auto_ring_stop = false;
}

// 房间命令参数结构体：用于解析命令行输入的房间ID
static struct {
    struct arg_str *room_id;  // 房间ID参数（如 "w123456"）
    struct arg_end *end;      // 参数列表结束标记
} room_args;

// 完整房间URL缓存，格式: "https://server/join/room_id"
static char room_url[128];
static char s_auto_room_id[32] = "";

/**
 * @brief 异步任务宏：创建新线程执行指定代码块
 * @param name 任务名称（用于线程标识）
 * @param body 要在新线程中执行的代码
 * 
 * 用法示例: RUN_ASYNC(leave, { stop_webrtc(); });
 * 会创建名为 "leave" 的线程执行 stop_webrtc()，执行完自动销毁线程
 */
#define RUN_ASYNC(name, body)           \
    void run_async##name(void *arg)     \
    {                                   \
        body;                           \
        media_lib_thread_destroy(NULL); \
    }                                   \
    media_lib_thread_create_from_scheduler(NULL, #name, run_async##name, NULL);

// WebRTC信令服务器地址，默认使用Espressif官方服务器
char server_url[64] = "https://webrtc.espressif.com";

/**
 * @brief 加入WebRTC房间命令处理函数
 * @param argc 参数数量
 * @param argv 参数数组，argv[0]为命令名 "join"，argv[1]为房间ID
 * @return 0: 成功，1: 参数错误
 * 
 * 功能：
 * 1. 解析房间ID参数
 * 2. 首次调用时同步SNTP时间（WebRTC需要准确时间戳）
 * 3. 构造房间URL并启动WebRTC连接
 * 
 * 使用示例: join w123456
 */
static int join_room(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&room_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, room_args.end, argv[0]);
        return 1;
    }
    // SNTP时间同步：WebRTC需要准确时间戳，首次调用时同步
    static bool sntp_synced = false;
    if (sntp_synced == false) {
        if (0 == webrtc_utils_time_sync_init()) {
            sntp_synced = true;
        }
    }
    const char *room_id = room_args.room_id->sval[0];
    snprintf(room_url, sizeof(room_url), "%s/join/%s", server_url, room_id);
    ESP_LOGI(TAG, "Start to join in room %s", room_id);
    start_webrtc(room_url);
    return 0;
}

/**
 * @brief 离开WebRTC房间命令处理函数
 * 使用异步线程停止WebRTC会话，避免阻塞命令行
 */
static int leave_room(int argc, char **argv)
{
    RUN_ASYNC(leave, { stop_webrtc(); });
    return 0;
}

static int cmd_cli(int argc, char **argv)
{
    send_cmd(argc > 1 ? argv[1] : "ring");
    return 0;
}

static int assert_cli(int argc, char **argv)
{
    *(int *)0 = 0;
    return 0;
}

static int sys_cli(int argc, char **argv)
{
    sys_state_show();
    return 0;
}

static int wifi_cli(int argc, char **argv)
{
    if (argc < 1) {
        return -1;
    }
    char *ssid = argv[1];
    char *password = argc > 2 ? argv[2] : NULL;
    return network_connect_wifi(ssid, password);
}

static int server_cli(int argc, char **argv)
{
    int server_sel = argc > 1 ? atoi(argv[1]) : 0;
    if (server_sel == 0) {
        strcpy(server_url, "https://webrtc.espressif.com");
    } else {
        strcpy(server_url, "https://webrtc.espressif.cn");
    }
    ESP_LOGI(TAG, "Select server %s", server_url);
    return 0;
}

static int capture_to_player_cli(int argc, char **argv)
{
    return test_capture_to_player();
}

static int measure_cli(int argc, char **argv)
{
    void measure_enable(bool enable);
    void show_measure(void);
    measure_enable(true);
    media_lib_thread_sleep(1500);
    measure_enable(false);
    return 0;
}

static int tracker_cli(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGI(TAG, "Usage: tracker [start|stop|reset]");
        return -1;
    }
    
    if (strcmp(argv[1], "start") == 0) {
        motion_tracker_start();
        ESP_LOGI(TAG, "▶️  Motion tracker started");
    } else if (strcmp(argv[1], "stop") == 0) {
        motion_tracker_stop();
        ESP_LOGI(TAG, "⏸️  Motion tracker stopped");
    } else if (strcmp(argv[1], "reset") == 0) {
        motion_tracker_reset();
        ESP_LOGI(TAG, "🔄 Motion tracker reset");
    } else {
        ESP_LOGI(TAG, "Unknown command: %s", argv[1]);
        return -1;
    }
    return 0;
}

static int gimbal_cli(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGI(TAG, "Usage: gimbal [scan|reset|yaw <angle>|pitch <angle>]");
        return -1;
    }
    
    if (strcmp(argv[1], "scan") == 0) {
        servo_test_scan();
    } else if (strcmp(argv[1], "reset") == 0) {
        servo_reset_gimbal();
        ESP_LOGI(TAG, "🔄 Gimbal reset to center");
    } else if (strcmp(argv[1], "yaw") == 0 && argc >= 3) {
        float angle = atof(argv[2]);
        servo_set_angle(LEDC_CHANNEL_0, angle);
        ESP_LOGI(TAG, "Set YAW to %.1f°", angle);
    } else if (strcmp(argv[1], "pitch") == 0 && argc >= 3) {
        float angle = atof(argv[2]);
        servo_set_angle(LEDC_CHANNEL_1, angle);
        ESP_LOGI(TAG, "Set PITCH to %.1f°", angle);
    } else {
        ESP_LOGI(TAG, "Unknown gimbal command");
        return -1;
    }
    return 0;
}

static int init_console()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "esp>";
    repl_config.task_stack_size = 16 * 1024;  // 增加栈大小：10KB → 16KB
    repl_config.task_priority = 22;
    repl_config.max_cmdline_length = 1024;
    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif

    room_args.room_id = arg_str1(NULL, NULL, "<w123456>", "room name");
    room_args.end = arg_end(2);
    esp_console_cmd_t cmds[] = {
        {
            .command = "join",
            .help = "Please enter a room name.\r\n",
            .func = join_room,
            .argtable = &room_args,
        },
        {
            .command = "leave",
            .help = "Leave from room\n",
            .func = leave_room,
        },
        {
            .command = "cmd",
            .help = "Send command (ring etc)\n",
            .func = cmd_cli,
        },
        {
            .command = "i",
            .help = "Show system status\r\n",
            .func = sys_cli,
        },
        {
            .command = "assert",
            .help = "Assert system\r\n",
            .func = assert_cli,
        },
        {
            .command = "rec2play",
            .help = "Play capture content\n",
            .func = capture_to_player_cli,
        },
        {
            .command = "wifi",
            .help = "wifi ssid psw\r\n",
            .func = wifi_cli,
        },
        {
            .command = "m",
            .help = "measure system loading\r\n",
            .func = measure_cli,
        },
        {
            .command = "server",
            .help = "Select server\r\n",
            .func = server_cli,
        },
        {
            .command = "tracker",
            .help = "Motion tracker control: tracker [start|stop|reset]\r\n",
            .func = tracker_cli,
        },
        {
            .command = "gimbal",
            .help = "Gimbal control: gimbal [scan|reset|yaw <angle>|pitch <angle>]\r\n",
            .func = gimbal_cli,
        },
    };
    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    return 0;
}

static void thread_scheduler(const char *thread_name, media_lib_thread_cfg_t *schedule_cfg)
{
    if (strcmp(thread_name, "venc_0") == 0) {
        // 视频编码线程：H264硬件编码栈可减小，软件编码需更多栈空间
        schedule_cfg->priority = 10;
#if CONFIG_IDF_TARGET_ESP32S3
        schedule_cfg->stack_size = 20 * 1024;
#endif
    }
#ifdef WEBRTC_SUPPORT_OPUS
    else if (strcmp(thread_name, "aenc_0") == 0) {
        // 音频编码线程：OPUS编码器需要大量栈空间，G711可设小值
        schedule_cfg->stack_size = 40 * 1024;
        schedule_cfg->priority = 10;
        schedule_cfg->core_id = 1;  // 绑定核心1减少干扰
    }
#endif
    else if (strcmp(thread_name, "AUD_SRC") == 0) {
        // 音频采集线程：高优先级保证采样实时性
        schedule_cfg->priority = 15;
    } else if (strcmp(thread_name, "pc_task") == 0) {
        // PeerConnection主任务：处理信令和媒体流控制
        schedule_cfg->stack_size = 25 * 1024;
        schedule_cfg->priority = 18;
        schedule_cfg->core_id = 1;
    }
    if (strcmp(thread_name, "start") == 0) {
        // WebRTC启动线程：临时任务，栈空间可小
        schedule_cfg->stack_size = 6 * 1024;
    }
}

/**
 * @brief 采集模块线程调度适配器：将媒体库调度配置转换为采集模块调度配置
 * @param name 线程名称
 * @param schedule_cfg 采集模块的调度配置结构体
 * 
 * 作用：统一管理所有线程的调度策略，确保采集线程（如摄像头读取）也遵循调度规则
 */
static void capture_scheduler(const char *name, esp_capture_thread_schedule_cfg_t *schedule_cfg)
{
    media_lib_thread_cfg_t cfg = {
        .stack_size = schedule_cfg->stack_size,
        .priority = schedule_cfg->priority,
        .core_id = schedule_cfg->core_id,
    };
    schedule_cfg->stack_in_ext = true;  // 使用外部PSRAM存放栈，节省内部RAM
    thread_scheduler(name, &cfg);
    schedule_cfg->stack_size = cfg.stack_size;
    schedule_cfg->priority = cfg.priority;
    schedule_cfg->core_id = cfg.core_id;
}

/**
 * @brief Jpeg解码一张图片 (from USB camera example)
 */
static int esp_jpeg_decoder_one_picture(uint8_t *input_buf, size_t len, uint8_t *output_buf)
{
    const int lcd_width = lcd_dev.width ? lcd_dev.width : 320;
    const int lcd_height = lcd_dev.height ? lcd_dev.height : 240;

    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)input_buf,
        .indata_size = len,
        .outbuf = output_buf,
        .outbuf_size = lcd_width * lcd_height * sizeof(uint16_t),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 0,
        }
    };

    esp_jpeg_image_output_t outimg = {0};
    esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "JPEG decoded: %dx%d", outimg.width, outimg.height);
    }
    return ret;
}

/**
 * @brief 自适应JPG帧缓冲器 (from USB camera example)
 */
static void adaptive_jpg_frame_buffer(size_t length)
{
    if (jpg_frame_buf1 != NULL) {
        free(jpg_frame_buf1);
    }
    if (jpg_frame_buf2 != NULL) {
        free(jpg_frame_buf2);
    }
    
    jpg_frame_buf1 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf1 != NULL);
    jpg_frame_buf2 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf2 != NULL);
    
    ESP_ERROR_CHECK(ppbuffer_create(ppbuffer_handle, jpg_frame_buf2, jpg_frame_buf1));
    if_ppbuffer_init = true;
    ESP_LOGI(TAG, "USB camera buffers allocated: %zu bytes", length);
}

/**
 * @brief 摄像头帧回调函数 (from USB camera example)
 */
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    if (current_width != frame->width || current_height != frame->height) {
        current_width = frame->width;
        current_height = frame->height;
        adaptive_jpg_frame_buffer(current_width * current_height * 2);
    }

    static void *jpeg_buffer = NULL;
    ppbuffer_get_write_buf(ppbuffer_handle, &jpeg_buffer);
    assert(jpeg_buffer != NULL);
    
    esp_jpeg_decoder_one_picture((uint8_t *)frame->data, frame->data_bytes, jpeg_buffer);
    ppbuffer_set_write_done(ppbuffer_handle);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * @brief USB摄像头显示任务 (from USB camera example)
 */
static void usb_display_task(void *arg)
{
    uint16_t *lcd_buffer = NULL;
    int64_t count_start_time = 0;
    int frame_count = 0;
    int fps = 0;
    int x_start = 0;
    int y_start = 0;

    int wait_cycles = 0;
    while (panel_handle == NULL) {
        if (wait_cycles % 50 == 0) {
            ESP_LOGW(TAG, "Waiting for LCD panel handle...");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        ++wait_cycles;
    }

    while (!if_ppbuffer_init) {
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "USB camera display task started");

    while (1) {
        if (ppbuffer_get_read_buf(ppbuffer_handle, (void *)&lcd_buffer) == ESP_OK) {
            const int lcd_width = lcd_dev.width ? lcd_dev.width : current_width;
            const int lcd_height = lcd_dev.height ? lcd_dev.height : current_height;

            if (current_width <= lcd_width && current_height <= lcd_height) {
                x_start = (lcd_width - current_width) / 2;
                y_start = (lcd_height - current_height) / 2;
                esp_lcd_panel_draw_bitmap(panel_handle,
                                          x_start,
                                          y_start,
                                          x_start + current_width,
                                          y_start + current_height,
                                          lcd_buffer);
            }
            
            ppbuffer_set_read_done(ppbuffer_handle);

            if (count_start_time == 0) {
                count_start_time = esp_timer_get_time();
            }

            if (++frame_count == 20) {
                frame_count = 0;
                fps = 20 * 1000000 / (esp_timer_get_time() - count_start_time);
                count_start_time = esp_timer_get_time();
                ESP_LOGI(TAG, "USB camera fps: %d (%dx%d)", fps, current_width, current_height);
            }
        }
        vTaskDelay(1);
    }
}

static void usb_get_value_from_nvs(const char *key, void *value, size_t *size)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("memory", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_get_blob(handle, key, value, size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "%s not saved yet", key);
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS read %s failed: %s", key, esp_err_to_name(err));
    }

    nvs_close(handle);
}

static esp_err_t usb_set_value_to_nvs(const char *key, const void *value, size_t size)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("memory", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, key, value, size);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write %s failed: %s", key, esp_err_to_name(err));
    }

    nvs_close(handle);
    return err;
}

static size_t usb_camera_find_current_resolution(camera_frame_size_t *camera_frame_size)
{
    if (camera_resolution_info.camera_frame_list == NULL) {
        return (size_t)-1;
    }

    size_t index = (size_t)-1;
    for (size_t i = 0; i < camera_resolution_info.camera_frame_list_num; ++i) {
        const uvc_frame_size_t *frame = &camera_resolution_info.camera_frame_list[i];
        if (camera_frame_size->width >= frame->width && camera_frame_size->height >= frame->height) {
            camera_frame_size->width = frame->width;
            camera_frame_size->height = frame->height;
            index = i;
            break;
        }
        if (i == camera_resolution_info.camera_frame_list_num - 1) {
            camera_frame_size->width = frame->width;
            camera_frame_size->height = frame->height;
            index = i;
        }
    }

    if (index != (size_t)-1) {
        ESP_LOGI(TAG, "Current resolution is %dx%d", camera_frame_size->width, camera_frame_size->height);
    }

    return index;
}

/**
 * @brief USB数据流状态回调 (simplified from USB camera example)
 */
static void usb_stream_state_changed_cb(usb_stream_state_t event, void *arg)
{
    lcd_log_line("USB event: %d", event);
    switch (event) {
        case STREAM_CONNECTED: {
            s_usb_device_ready = true;
            lcd_log_line("USB event: STREAM_CONNECTED");
            size_t size = sizeof(camera_frame_size_t);
            usb_get_value_from_nvs(DEMO_KEY_RESOLUTION, &camera_resolution_info.camera_frame_size, &size);
            size_t frame_index = 0;
            uvc_frame_size_list_get(NULL, &camera_resolution_info.camera_frame_list_num, NULL);

            if (camera_resolution_info.camera_frame_list_num) {
                lcd_log_line("UVC: Found %d frame sizes", camera_resolution_info.camera_frame_list_num);
                ESP_LOGI(TAG, "UVC: get frame list size = %u, current = %u",
                         camera_resolution_info.camera_frame_list_num, frame_index);
                uvc_frame_size_t *_frame_list = (uvc_frame_size_t *)malloc(camera_resolution_info.camera_frame_list_num * sizeof(uvc_frame_size_t));

                camera_resolution_info.camera_frame_list = (uvc_frame_size_t *)realloc(camera_resolution_info.camera_frame_list,
                                                                                       camera_resolution_info.camera_frame_list_num * sizeof(uvc_frame_size_t));

                if (camera_resolution_info.camera_frame_list == NULL) {
                    ESP_LOGE(TAG, "camera_resolution_info.camera_frame_list realloc failed");
                    lcd_log_line("Error: Frame list alloc failed");
                }

                uvc_frame_size_list_get(_frame_list, NULL, NULL);

                for (size_t i = 0; i < camera_resolution_info.camera_frame_list_num; i++) {
                    if (_frame_list[i].width <= lcd_dev.width && _frame_list[i].height <= lcd_dev.height) {
                        camera_resolution_info.camera_frame_list[frame_index++] = _frame_list[i];
                        ESP_LOGI(TAG, "\tpick frame[%u] = %ux%u", (unsigned)i, _frame_list[i].width, _frame_list[i].height);
                    } else {
                        ESP_LOGI(TAG, "\tdrop frame[%u] = %ux%u", (unsigned)i, _frame_list[i].width, _frame_list[i].height);
                    }
                }
                camera_resolution_info.camera_frame_list_num = frame_index;
                lcd_log_line("UVC: Picked %d frames", frame_index);

                if (camera_resolution_info.camera_frame_size.width != 0 && camera_resolution_info.camera_frame_size.height != 0) {
                    camera_resolution_info.camera_currect_frame_index = usb_camera_find_current_resolution(&camera_resolution_info.camera_frame_size);
                } else {
                    camera_resolution_info.camera_currect_frame_index = 0;
                }

                if (camera_resolution_info.camera_currect_frame_index == (size_t)-1) {
                    ESP_LOGE(TAG, "find current resolution fail");
                    lcd_log_line("Error: Resolution not found");
                } else {
                    lcd_log_line("UVC: Setting resolution...");
                    ESP_ERROR_CHECK(uvc_frame_size_reset(
                        camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].width,
                        camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].height,
                        FPS2INTERVAL(30)));
                    camera_frame_size_t camera_frame_size = {
                        .width = camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].width,
                        .height = camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].height,
                    };
                    ESP_ERROR_CHECK(usb_set_value_to_nvs(DEMO_KEY_RESOLUTION, &camera_frame_size, sizeof(camera_frame_size_t)));
                    lcd_log_line("UVC: Resolution set");
                }

                if (_frame_list != NULL) {
                    free(_frame_list);
                }
                
                lcd_log_line("UVC: Resuming stream...");
                usb_streaming_control(STREAM_UVC, CTRL_RESUME, NULL);
                xEventGroupSetBits(s_evt_handle, BIT0_FRAME_START);
            } else {
                ESP_LOGW(TAG, "UVC: get frame list size = %u", camera_resolution_info.camera_frame_list_num);
                lcd_log_line("UVC: No frames found!");
            }
            ESP_LOGI(TAG, "Device connected");
            break;
        }
        case STREAM_DISCONNECTED:
            s_usb_device_ready = false;
            xEventGroupClearBits(s_evt_handle, BIT0_FRAME_START);
            lcd_log_line("USB event: STREAM_DISCONNECTED");
            ESP_LOGI(TAG, "Device disconnected");
            break;
        default:
            ESP_LOGE(TAG, "Unknown event");
            lcd_log_line("USB event: Unknown (%d)", event);
            break;
    }
}

static esp_err_t usb_stream_init(void)
{
    uvc_config_t uvc_config = {
        .frame_interval = FRAME_INTERVAL_FPS_30,
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

    esp_err_t ret = uvc_streaming_config(&uvc_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uvc streaming config failed");
    }
    return ret;
}

/**
 * @brief 根据设备MAC地址生成唯一房间ID
 * @return 房间ID字符串，格式: "esp_XXYYZZ"（后3字节MAC地址）
 * 
 * 用途：设备上电后自动使用MAC生成固定房间ID，无需手动输入
 * 示例：MAC 24:62:AB:12:34:56 → 房间ID "esp_123456"
 */
static char* gen_room_id_use_mac(void)
{
    static char room_mac[24];
    uint8_t mac[6];
    network_get_mac(mac);
    // 添加随机数避免房间冲突
    uint16_t random = esp_random() & 0xFFFF;
    snprintf(room_mac, sizeof(room_mac)-1, "esp_%02x%02x%02x_%04x", 
             mac[3], mac[4], mac[5], random);
    return room_mac;
}

/**
 * @brief 网络事件回调函数：处理Wi-Fi连接成功/断开事件
 * @param connected true: Wi-Fi已连接，false: Wi-Fi断开
 * @return 0
 * 
 * 功能：
 * - 连接成功：自动生成房间ID并加入WebRTC房间（门铃自动上线）
 * - 连接断开：停止WebRTC会话，释放资源
 */
static int network_event_handler(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "Wi-Fi Connected");
        if (ENABLE_AUTO_WEBRTC_JOIN && !s_webrtc_auto_started) {
            s_webrtc_auto_started = true;
            RUN_ASYNC(auto_webrtc_join, {
                vTaskDelay(pdMS_TO_TICKS(9000));
                const char *room = "my_doorbell_room"; // 使用固定房间ID
                strncpy(s_auto_room_id, room, sizeof(s_auto_room_id) - 1);
                s_auto_room_id[sizeof(s_auto_room_id) - 1] = '\0';
                snprintf(room_url, sizeof(room_url), "%s/join/%s", server_url, s_auto_room_id);
                ESP_LOGI(TAG, "🌐 Auto joining WebRTC room: %s", s_auto_room_id);
                ESP_LOGI(TAG, "   Open https://webrtc.espressif.com/#/doorbell and enter room ID: %s", s_auto_room_id);
                
                ESP_LOGI(TAG, "Joining WebRTC room: %s", s_auto_room_id);
                int ret = start_webrtc(room_url);
                if (ret != 0) {
                    ESP_LOGE(TAG, "Auto WebRTC start failed (%d)", ret);
                } else {
                    ESP_LOGI(TAG, "WebRTC Connected - Video Call Active");
                }
            });
        }
        start_auto_ring_task();
    } else {
        ESP_LOGI(TAG, "Wi-Fi Disconnected");
        stop_webrtc();
        s_webrtc_auto_started = false;
        stop_auto_ring_task();
    }
    return 0;
}

/**
 * @brief 应用程序主入口函数，系统启动后首先执行此函数
 * 负责初始化初始化硬件、媒体系统、网络等核心模块，并进入主循环处理状态查询
 */
void app_main(void)
{
    // 早期日志：验证是否进入app_main
    printf("\n\n========================================\n");
    printf("🚀 ENTERING APP_MAIN\n");
    printf("========================================\n\n");
    
    // 设置全局日志输出级别为ESP_LOG_INFO：高于INFO级别的日志（如DEBUG）不输出
    // 可减少冗余日志，只保留关键运行信息
    esp_log_level_set("*", ESP_LOG_INFO);

    // 添加默认媒体适配器：初始化音视频编解码、采集/播放的适配接口
    // 为后续媒体操作（如摄像头采集、音频编码）提供统一调用层
    media_lib_add_default_adapter();

    // 设置音视频采集线程的调度器：自定义采集线程的优先级、栈大小、绑定CPU核心
    // 确保采集线程（如摄像头帧读取、麦克风录音）高效运行，避免卡顿
    esp_capture_set_thread_scheduler(capture_scheduler);

    // 设置媒体库线程的调度回调函数：为所有媒体相关线程（如编码、解码线程）配置调度参数
    // 例如视频编码线程分配更高优先级，确保实时性
    media_lib_thread_set_schedule_cb(thread_scheduler);

    // 初始化硬件板卡：摄像头、按键、音频 codec 等外设
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📟 Initializing board peripherals...");
    ESP_LOGI(TAG, "========================================");

    led_init();
    init_board();
    ESP_LOGI(TAG, "✅ Board core peripherals ready");

    // 使用 USB Camera 示例的 LCD 驱动进行屏幕初始化
    ESP_LOGI(TAG, "🖥️  Initializing LCD display (USB camera reference)...");
    lcd_cfg_t lcd_cfg = {
        .notify_flush_ready = NULL,
        .user_ctx = NULL,
    };
    lcd_init(lcd_cfg);
    ESP_LOGI(TAG, "✅ LCD display initialized via BSP/LCD driver");
    lcd_log_reset(BLACK);
    lcd_log_line("LCD ready (%dx%d)", lcd_dev.width, lcd_dev.height);
    
    // 检查 I2C 和 IO 扩展芯片
    lcd_log_line("Checking I2C devices...");
    if (myiic_init() == ESP_OK) {
        lcd_log_line("I2C master OK");
    } else {
        lcd_log_line("I2C master FAILED");
    }
    if (xl9555_init() == ESP_OK) {
        lcd_log_line("XL9555 IO expander OK");
        // 尝试显式启用 USB 电源
        xl9555_pin_write(IO1_4, 1);
        lcd_log_line("USB Power Enabled");
    } else {
        lcd_log_line("XL9555 IO expander FAILED");
    }

    lcd_log_line("Waiting for USB camera...");

    // Initialize USB camera (following 29_usb_camera example)
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📷 Initializing USB Camera...");
    ESP_LOGI(TAG, "========================================");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(nvs_ret);
    }

    s_evt_handle = xEventGroupCreate();
    if (s_evt_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create event group for USB camera");
    } else {
        bool alloc_ok = true;
        xfer_buffer_a = malloc(DEMO_UVC_XFER_BUFFER_SIZE);
        xfer_buffer_b = malloc(DEMO_UVC_XFER_BUFFER_SIZE);
        frame_buffer = malloc(DEMO_UVC_XFER_BUFFER_SIZE);
        ppbuffer_handle = calloc(1, sizeof(PingPongBuffer_t));

        if (!xfer_buffer_a || !xfer_buffer_b || !frame_buffer || !ppbuffer_handle) {
            ESP_LOGE(TAG, "Failed to allocate USB camera buffers");
            alloc_ok = false;
        }

        if (!alloc_ok) {
            free(xfer_buffer_a);
            free(xfer_buffer_b);
            free(frame_buffer);
            free(ppbuffer_handle);
            xfer_buffer_a = xfer_buffer_b = frame_buffer = NULL;
            ppbuffer_handle = NULL;
        } else {
            BaseType_t task_ret = xTaskCreate(usb_display_task, "usb_display_task", 4 * 1024, NULL, 5, NULL);
            if (task_ret != pdPASS) {
                ESP_LOGE(TAG, "Failed to create USB display task");
                lcd_log_line("Error: Display task failed");
            } else {
                lcd_log_line("USB: Initing stream...");
                ESP_ERROR_CHECK(usb_stream_init());
                lcd_log_line("USB: Registering callback...");
                ESP_ERROR_CHECK(usb_streaming_state_register(&usb_stream_state_changed_cb, NULL));
                lcd_log_line("USB: Starting stream...");
                ESP_ERROR_CHECK(usb_streaming_start());
                lcd_log_line("USB stream started");
                s_usb_device_ready = false;
                if (s_usb_wait_task == NULL && lcd_log_active) {
                    xTaskCreate(usb_wait_screen_task, "usb_wait_screen", 2048, NULL, 4, &s_usb_wait_task);
                }
                lcd_log_line("USB: Waiting for connection...");
                ESP_ERROR_CHECK(usb_streaming_connect_wait(portMAX_DELAY));
                ESP_LOGI(TAG, "✅ USB camera system ready");
            }
        }
    }

    // 构建媒体系统：启用音频系统（纯音频模式，暂无摄像头）
    ESP_LOGI(TAG, "🎬 Building media system (audio-only mode)...");
    media_sys_buildup();
    ESP_LOGI(TAG, "✅ Media system ready");

    // 初始化舵机云台系统（不立即扫描，通过命令手动测试）
    ESP_LOGI(TAG, "🎮 Initializing servo gimbal...");
    if (servo_init() == ESP_OK) {
        ESP_LOGI(TAG, "✅ Servo gimbal initialized (use 'gimbal scan' to test)");
    } else {
        ESP_LOGW(TAG, "⚠️  Servo gimbal init skipped (no hardware)");
    }

    // 初始化运动追踪器（不立即启动，通过命令手动启动）
    ESP_LOGI(TAG, "🎯 Initializing motion tracker...");
    tracker_config_t tracker_cfg = {
        .frame_width = 640,          // USB摄像头默认分辨率（根据实际调整）
        .frame_height = 480,
        .motion_threshold = DEFAULT_MOTION_THRESHOLD,
        .pid_kp = DEFAULT_PID_KP,
        .pid_ki = DEFAULT_PID_KI,
        .pid_kd = DEFAULT_PID_KD,
        .max_speed = DEFAULT_MAX_SPEED,
    };
    if (motion_tracker_init(&tracker_cfg) == ESP_OK) {
        ESP_LOGI(TAG, "✅ Motion tracker initialized (use 'tracker start' to enable)");
    } else {
        ESP_LOGW(TAG, "⚠️  Motion tracker init skipped");
    }
    ESP_LOGI(TAG, "========================================");

    // 自动硬件测试（通过WebRTC验证）
    ESP_LOGI(TAG, "🧪 Hardware ready for WebRTC testing");
    ESP_LOGI(TAG, "   Use browser to test microphone and speaker");
    ESP_LOGI(TAG, "========================================");

    // 初始化命令行控制台：注册"join"、"wifi"等交互命令，启动串口控制台服务
    // 用于调试和手动控制设备（如输入指令连接Wi-Fi、加入房间）
    init_console();

    // 初始化网络：连接预设的Wi-Fi（WIFI_SSID和WIFI_PASSWORD在settings.h中定义）
    // 并注册网络事件回调（连接成功/断开时触发对应处理逻辑）
    network_init(WIFI_SSID, WIFI_PASSWORD, network_event_handler);

    // 主循环：定期查询WebRTC状态，保持程序运行
    // 注意：控制台在独立的REPL任务中运行，不会被这个循环阻塞
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "✅ System ready! Type 'help' for commands");
    ESP_LOGI(TAG, "========================================");
    
    while (1) {
        // 休眠5秒：降低CPU占用，给控制台更多响应时间
        media_lib_thread_sleep(5000);

        // 查询WebRTC当前状态（静默模式，减少日志输出）
        query_webrtc();
    }
}
