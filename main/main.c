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
#include "lcd_test.h"
#include "servo_control.h"
#include "motion_tracker.h"
#include "mic_loopback_test.h"
#include "mic_diagnostic.h"
#include "codec_init.h"
#include "esp_codec_dev.h"
#include "esp_bit_defs.h"

bool webrtc_is_active(void);
void webrtc_trigger_ring(void);
bool webrtc_is_peer_connected(void);
bool webrtc_is_ringing(void);

static const char *TAG = "Webrtc_Test";
static const char *TAG_MIC = "MicMonitor";
static bool s_mic_monitor_started = false;
static esp_codec_dev_handle_t s_mic_monitor_record = NULL;
static bool s_mic_monitor_record_open = false;
static bool s_loopback_auto_started = false;
static bool s_webrtc_auto_started = false;
static TaskHandle_t s_mic_monitor_task_handle = NULL;
static volatile bool s_mic_monitor_stop_requested = false;
static TaskHandle_t s_auto_ring_task_handle = NULL;
static volatile bool s_auto_ring_stop = false;

#define ENABLE_AUTO_LOOPBACK 0
#define ENABLE_AUTO_WEBRTC_JOIN 1

#define AUTO_RING_INITIAL_DELAY_MS 30000
#define AUTO_RING_RETRY_DELAY_MS   15000
#define AUTO_RING_MAX_ATTEMPTS     5

static void stop_auto_ring_task(void);

static bool mic_monitor_prepare_record(void)
{
    if (s_mic_monitor_record == NULL) {
        s_mic_monitor_record = get_record_handle();
        if (s_mic_monitor_record == NULL) {
            ESP_LOGW(TAG_MIC, "Record handle not ready");
            return false;
        }
    }

    if (!s_mic_monitor_record_open) {
        esp_codec_dev_sample_info_t fs = {
            .sample_rate = 16000,
            .channel = 1,
            .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
            .bits_per_sample = 16,
            .mclk_multiple = 0,
        };
        int ret = esp_codec_dev_open(s_mic_monitor_record, &fs);
        if (ret != ESP_CODEC_DEV_OK && ret != ESP_CODEC_DEV_WRONG_STATE) {
            ESP_LOGW(TAG_MIC, "Failed to open record device (%d)", ret);
            return false;
        }
        if (ret == ESP_CODEC_DEV_OK || !s_mic_monitor_record_open) {
            int gain_ret = esp_codec_dev_set_in_gain(s_mic_monitor_record, 40.0f);
            if (gain_ret != ESP_CODEC_DEV_OK) {
                ESP_LOGW(TAG_MIC, "Failed to set input gain (%d)", gain_ret);
            }
        }
        s_mic_monitor_record_open = true;
    }

    return true;
}

static void mic_monitor_task(void *arg)
{
    int16_t sample_buffer[128];  // 128 samples = 256 bytes @16-bit

    while (!s_mic_monitor_stop_requested) {
        if (!mic_monitor_prepare_record()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int read_ret = esp_codec_dev_read(s_mic_monitor_record, sample_buffer, sizeof(sample_buffer));
        if (read_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGW(TAG_MIC, "esp_codec_dev_read failed (%d)", read_ret);
            s_mic_monitor_record_open = false;
            esp_codec_dev_close(s_mic_monitor_record);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        size_t sample_count = sizeof(sample_buffer) / sizeof(int16_t);

        int16_t peak_max = -32768;
        int16_t peak_min = 32767;

        for (size_t i = 0; i < sample_count; ++i) {
            if (sample_buffer[i] > peak_max) {
                peak_max = sample_buffer[i];
            }
            if (sample_buffer[i] < peak_min) {
                peak_min = sample_buffer[i];
            }
        }

        ESP_LOGI(TAG_MIC, "Samples=%u range=[%d, %d]", (unsigned)sample_count, peak_min, peak_max);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (s_mic_monitor_record && s_mic_monitor_record_open) {
        esp_codec_dev_close(s_mic_monitor_record);
        s_mic_monitor_record_open = false;
    }

    s_mic_monitor_started = false;
    s_mic_monitor_task_handle = NULL;
    s_mic_monitor_stop_requested = false;
    ESP_LOGI(TAG_MIC, "Mic monitor task stopped");
    vTaskDelete(NULL);
}

static void mic_monitor_start(void)
{
    if (s_mic_monitor_started) {
        return;
    }

    s_mic_monitor_stop_requested = false;
    if (xTaskCreate(mic_monitor_task, "mic_monitor", 3072, NULL, 4, &s_mic_monitor_task_handle) == pdPASS) {
        s_mic_monitor_started = true;
        ESP_LOGI(TAG_MIC, "Mic monitor task started (1 Hz)");
    } else {
        ESP_LOGW(TAG_MIC, "Failed to start mic monitor task");
    }
}

static void mic_monitor_stop(void)
{
    if (!s_mic_monitor_started) {
        return;
    }

    s_mic_monitor_stop_requested = true;

    // Wait for the task to cleanly exit
    const TickType_t wait_tick = pdMS_TO_TICKS(50);
    for (int retry = 0; retry < 40; ++retry) {
        if (!s_mic_monitor_started) {
            break;
        }
        vTaskDelay(wait_tick);
    }

    if (s_mic_monitor_started && s_mic_monitor_task_handle) {
        vTaskDelete(s_mic_monitor_task_handle);
        s_mic_monitor_task_handle = NULL;
        s_mic_monitor_started = false;
        s_mic_monitor_stop_requested = false;
    }

    if (s_mic_monitor_record && s_mic_monitor_record_open) {
        esp_codec_dev_close(s_mic_monitor_record);
        s_mic_monitor_record_open = false;
    }
}

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
    mic_monitor_stop();
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

static int test_cli(int argc, char **argv)
{
    if (argc < 2) {
        ESP_LOGI(TAG, "Usage: test [lcd|mic|speaker|all]");
        return -1;
    }
    
    if (strcmp(argv[1], "lcd") == 0) {
        ESP_LOGI(TAG, "🖥️  Testing LCD...");
        lcd_test_run();
    } else if (strcmp(argv[1], "mic") == 0) {
        ESP_LOGI(TAG, "🎤 Testing microphone (recording 3 seconds)...");
        ESP_LOGI(TAG, "Please speak now!");
        // TODO: 麦克风测试需要实现录音逻辑
        ESP_LOGW(TAG, "Microphone test not yet implemented");
    } else if (strcmp(argv[1], "speaker") == 0) {
        ESP_LOGI(TAG, "🔊 Testing speaker...");
        // TODO: 喇叭测试需要实现播放逻辑
        ESP_LOGW(TAG, "Speaker test not yet implemented");
    } else if (strcmp(argv[1], "all") == 0) {
        ESP_LOGI(TAG, "🧪 Running all hardware tests...");
        lcd_test_run();
        media_lib_thread_sleep(2000);
        ESP_LOGW(TAG, "Audio tests not yet implemented");
    } else {
        ESP_LOGI(TAG, "Unknown test: %s", argv[1]);
        return -1;
    }
    return 0;
}

/**
 * @brief 麦克风回音测试命令
 * 用法: loopback [start|stop]
 */
static int loopback_cli(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: loopback [start|stop]\n");
        return 1;
    }
    
    if (strcmp(argv[1], "start") == 0) {
        printf("Starting microphone loopback test...\n");
        printf("Speak into the microphone, you should hear your voice from speaker!\n");
        mic_loopback_start();
    } else if (strcmp(argv[1], "stop") == 0) {
        printf("Stopping microphone loopback test...\n");
        mic_loopback_stop();
    } else {
        printf("Unknown command: %s\n", argv[1]);
        printf("Usage: loopback [start|stop]\n");
        return 1;
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
        {
            .command = "test",
            .help = "Hardware test: test [lcd|mic|speaker|all]\r\n",
            .func = test_cli,
        },
        {
            .command = "loopback",
            .help = "Microphone loopback test: loopback [start|stop]\r\n",
            .func = loopback_cli,
        },
    };
    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    return 0;
}

/**
 * @brief 线程调度配置回调函数：为不同功能的线程设置优先级、栈大小、CPU核心绑定
 * @param thread_name 线程名称（由媒体库/WebRTC模块传入）
 * @param schedule_cfg 调度配置结构体（输入默认值，函数修改后输出）
 * 
 * 关键线程配置：
 * - venc_0: 视频编码线程（优先级10，栈20KB for ESP32-S3）
 * - aenc_0: 音频编码线程（OPUS需40KB栈，优先级10，绑定核心1）
 * - AUD_SRC: 音频采集线程（优先级15，保证实时性）
 * - pc_task: PeerConnection任务（25KB栈，优先级18，核心1）
 */
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
        // Wi-Fi连接成功，运行麦克风诊断测试
        printf("\n========================================\n");
        printf("Wi-Fi connected!\n");
        printf("Running Microphone Diagnostic...\n");
        printf("This will test all I2S configurations.\n");
        printf("========================================\n\n");
        
        // 3秒后自动运行诊断
        RUN_ASYNC(auto_diagnostic, {
            vTaskDelay(pdMS_TO_TICKS(3000));  // 等待3秒
            ESP_LOGI(TAG, "🔬 Starting microphone diagnostic...");
            mic_diagnostic_run();
            ESP_LOGI(TAG, "✅ Diagnostic complete! Check results above.");
        });

        RUN_ASYNC(auto_mic_monitor, {
            vTaskDelay(pdMS_TO_TICKS(5000));  // 等待诊断完成后再监测
            mic_monitor_start();
        });

        if (ENABLE_AUTO_LOOPBACK && !s_loopback_auto_started) {
            s_loopback_auto_started = true;
            RUN_ASYNC(auto_loopback_start, {
                vTaskDelay(pdMS_TO_TICKS(7000));  // 诊断完成后自动开始回环
                ESP_LOGI(TAG, "🔁 Starting microphone loopback test (auto)");
                mic_loopback_start();
            });
            RUN_ASYNC(auto_loopback_stop, {
                vTaskDelay(pdMS_TO_TICKS(13000));  // 播放一段时间后自动停止
                ESP_LOGI(TAG, "⏹️  Stopping microphone loopback test (auto)");
                mic_loopback_stop();
            });
        }

        if (ENABLE_AUTO_WEBRTC_JOIN && !s_webrtc_auto_started) {
            s_webrtc_auto_started = true;
            RUN_ASYNC(auto_webrtc_join, {
                vTaskDelay(pdMS_TO_TICKS(9000));
                const char *room = gen_room_id_use_mac();
                strncpy(s_auto_room_id, room, sizeof(s_auto_room_id) - 1);
                s_auto_room_id[sizeof(s_auto_room_id) - 1] = '\0';
                snprintf(room_url, sizeof(room_url), "%s/join/%s", server_url, s_auto_room_id);
                ESP_LOGI(TAG, "🌐 Auto joining WebRTC room: %s", s_auto_room_id);
                ESP_LOGI(TAG, "   Open https://webrtc.espressif.com/#/doorbell and enter room ID: %s", s_auto_room_id);
                mic_monitor_stop();
                int ret = start_webrtc(room_url);
                if (ret != 0) {
                    ESP_LOGE(TAG, "Auto WebRTC start failed (%d)", ret);
                }
            });
        }
        start_auto_ring_task();
    } else {
        // Wi-Fi断开，停止测试和WebRTC
        mic_loopback_stop();
        stop_webrtc();
        s_loopback_auto_started = false;
        s_webrtc_auto_started = false;
        stop_auto_ring_task();
        mic_monitor_stop();
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

    // 初始化硬件板卡：包括摄像头、按键、音频 codec、LCD等外设的初始化
    // 具体初始化逻辑在init_board()函数中实现（如引脚配置、设备上电等）
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "📟 Initializing board...");
    ESP_LOGI(TAG, "========================================");
    
    // init_board()会自动执行完整的4步初始化流程：
    // 1. myiic_init()  - I2C总线（GPIO48/45）
    // 2. xl9555_init() - IO扩展芯片
    // 3. xl9555_pin_write(SPK_CTRL_IO, 1) - 扬声器使能
    // 4. myi2s_init()  - I2S音频（GPIO21/13/14/47）
    init_board();
    ESP_LOGI(TAG, "✅ Board initialized");

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3: 初始化 LCD（暂时禁用，因为与codec_init的I2C冲突）
    // TODO: 需要统一I2C管理，让LCD复用codec_init的I2C总线
    ESP_LOGW(TAG, "⚠️  LCD initialization skipped (I2C conflict with codec_init)");
    // lcd_test_init();
    // ESP_LOGI(TAG, "✅ LCD initialized (use 'test lcd' to test)");
#endif

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
