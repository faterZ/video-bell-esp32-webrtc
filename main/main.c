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
#include <nvs_flash.h>
#include <sys/param.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_webrtc.h"
#include "media_lib_adapter.h"
#include "media_lib_os.h"
#include "esp_timer.h"
#include "webrtc_utils_time.h"
#include "esp_cpu.h"
#include "settings.h"
#include "common.h"
#include "esp_capture.h"
#include "lcd_test.h"

static const char *TAG = "Webrtc_Test";

// 房间命令参数结构体：用于解析命令行输入的房间ID
static struct {
    struct arg_str *room_id;  // 房间ID参数（如 "w123456"）
    struct arg_end *end;      // 参数列表结束标记
} room_args;

// 完整房间URL缓存，格式: "https://server/join/room_id"
static char room_url[128];

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

static int init_console()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "esp>";
    repl_config.task_stack_size = 10 * 1024;
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
    static char room_mac[16];
    uint8_t mac[6];
    network_get_mac(mac);
    snprintf(room_mac, sizeof(room_mac)-1, "esp_%02x%02x%02x", mac[3], mac[4], mac[5]);
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
        // Wi-Fi连接成功，异步启动WebRTC会话
        RUN_ASYNC(start, {
            char *room = gen_room_id_use_mac();
            snprintf(room_url, sizeof(room_url), "%s/join/%s", server_url, room);
            ESP_LOGI(TAG, "Start to join in room %s", room);
            if (start_webrtc(room_url) == 0) {
                ESP_LOGW(TAG, "Please use browser to join in %s on %s/doorbell", room, server_url);
            }
        });
    } else {
        // Wi-Fi断开，停止WebRTC
        stop_webrtc();
    }
    return 0;
}

/**
 * @brief 应用程序主入口函数，系统启动后首先执行此函数
 * 负责初始化初始化硬件、媒体系统、网络等核心模块，并进入主循环处理状态查询
 */
void app_main(void)
{
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
    init_board();

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3: 初始化并测试 LCD 屏幕
    ESP_LOGI(TAG, "Initializing LCD test for ESP32-S3 Box...");
    lcd_test_init();
    lcd_test_run();
    ESP_LOGI(TAG, "LCD test completed");
#endif

    // 构建媒体系统：初始化音视频采集链路（摄像头→编码）和播放链路（解码→扬声器）
    // 建立从硬件到WebRTC模块的媒体数据传输通道
    media_sys_buildup();

    // 初始化命令行控制台：注册"join"、"wifi"等交互命令，启动串口控制台服务
    // 用于调试和手动控制设备（如输入指令连接Wi-Fi、加入房间）
    init_console();

    // 初始化网络：连接预设的Wi-Fi（WIFI_SSID和WIFI_PASSWORD在settings.h中定义）
    // 并注册网络事件回调（连接成功/断开时触发对应处理逻辑）
    network_init(WIFI_SSID, WIFI_PASSWORD, network_event_handler);

    // 主循环：定期查询WebRTC状态，保持程序运行
    while (1) {
        // 休眠2000毫秒（2秒）：降低CPU占用，避免空循环消耗过多资源
        media_lib_thread_sleep(2000);

        // 查询WebRTC当前状态（如连接状态、媒体流传输情况等）
        // 可在该函数中添加状态检测或异常处理逻辑
        query_webrtc();
    }
}
