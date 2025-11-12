/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       USB 摄像头实验
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

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "led.h"
#include "lcd.h"
#include "ppbuffer.h"
#include "usb_stream.h"
#include "jpeg_decoder.h"
#include "esp_timer.h"


#define DEMO_KEY_RESOLUTION "resolution"
static const char *TAG = "uvc_camera_lcd_demo";
#define DEMO_UVC_XFER_BUFFER_SIZE               ( 88 * 1024) /* 双缓冲 */
#define BIT0_FRAME_START                        (0x01 << 0)
static EventGroupHandle_t                       s_evt_handle;

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
static uint8_t *jpg_frame_buf1                         = NULL;
static uint8_t *jpg_frame_buf2                         = NULL;
static uint8_t *xfer_buffer_a                          = NULL;
static uint8_t *xfer_buffer_b                          = NULL;
static uint8_t *frame_buffer                           = NULL;
static PingPongBuffer_t *ppbuffer_handle               = NULL;
static uint16_t current_width                          = 0;
static uint16_t current_height                         = 0;
static bool if_ppbuffer_init                           = false;


/**
 * @brief       Jpeg解码器一张图片
 * @param       input_buf       :输入数据
 * @param       len             :大小
 * @param       output_buf      :输出数据
 * @retval      无
 */
static int esp_jpeg_decoder_one_picture(uint8_t *input_buf, size_t len, uint8_t *output_buf)
{
    esp_err_t ret = ESP_OK;

    /* jpeg解码配置 */
    esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = (uint8_t *)input_buf,
        .indata_size = len,
        .outbuf = (uint8_t *)(output_buf),
        .outbuf_size = lcd_dev.width * lcd_dev.height * sizeof(uint16_t),
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
        .flags = {
            .swap_color_bytes = 0,
        }
    };

    /* jpeg解码 */
    esp_jpeg_image_output_t outimg;
    esp_jpeg_decode(&jpeg_cfg, &outimg);

    ESP_LOGI(TAG, "JPEG image decoded! Size of the decoded image is: %dpx x %dpx", outimg.width, outimg.height);

    return ret;
}

/**
 * @brief       自适应JPG帧缓冲器
 * @param       length       :大小
 * @retval      无
 */
static void adaptive_jpg_frame_buffer(size_t length)
{
    if (jpg_frame_buf1 != NULL)
    {
        free(jpg_frame_buf1);
    }

    if (jpg_frame_buf2 != NULL)
    {
        free(jpg_frame_buf2);
    }
    /* 申请内存 */
    jpg_frame_buf1 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf1 != NULL);
    jpg_frame_buf2 = (uint8_t *)heap_caps_aligned_alloc(16, length, MALLOC_CAP_SPIRAM);
    assert(jpg_frame_buf2 != NULL);
    /* 申请ppbuffer存储区域 */
    ESP_ERROR_CHECK(ppbuffer_create(ppbuffer_handle, jpg_frame_buf2, jpg_frame_buf1));
    if_ppbuffer_init = true;
}

/**
 * @brief       摄像头回调函数
 * @param       frame       :从UVC设备接收到的图像帧
 * @param       ptr         :转入参数（未使用）
 * @retval      无
 */
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    if (current_width != frame->width || current_height != frame->height)
    {
        current_width = frame->width;
        current_height = frame->height;
        adaptive_jpg_frame_buffer(current_width * current_height * 2);
    }

    static void *jpeg_buffer = NULL;
    /* 获取可写缓冲区 */
    ppbuffer_get_write_buf(ppbuffer_handle, &jpeg_buffer);
    assert(jpeg_buffer != NULL);
    /* JPEG解码 */
    esp_jpeg_decoder_one_picture((uint8_t *)frame->data, frame->data_bytes, jpeg_buffer);
    /* 通知缓冲区写完成 */
    ppbuffer_set_write_done(ppbuffer_handle);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/**
 * @brief       usb摄像头任务函数
 * @param       arg     :未使用
 * @retval      无
 */
static void usb_display_task(void *arg)
{
    uint16_t *lcd_buffer = NULL;
    int64_t count_start_time = 0;
    int frame_count = 0;
    int fps = 0;
    int x_start = 0;
    int y_start = 0;

    while (!if_ppbuffer_init)
    {
        vTaskDelay(1);
    }

    while (1)
    {
        /* 获取可读缓冲区 */
        if (ppbuffer_get_read_buf(ppbuffer_handle, (void *)&lcd_buffer) == ESP_OK)
        {
            if (current_width == lcd_dev.width && current_height <= lcd_dev.height)
            {
                x_start = 0;
                y_start = (lcd_dev.height - current_height) / 2;
                esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_start + current_width, y_start + current_height, lcd_buffer);
            }
            /* 通知缓冲区读完成 */
            ppbuffer_set_read_done(ppbuffer_handle);

            if (count_start_time == 0)
            {
                count_start_time = esp_timer_get_time();
            }

            if (++frame_count == 20)
            {
                frame_count = 0;
                fps = 20 * 1000000 / (esp_timer_get_time() - count_start_time);
                count_start_time = esp_timer_get_time();
                ESP_LOGI(TAG, "camera fps: %d %d*%d", fps, current_width, current_height);
            }
        }

        vTaskDelay(1);
    }
}

/**
 * @brief       在nvs分区获取数值
 * @param       key     :名称
 * @param       value   :数据
 * @param       size    :大小
 * @retval      无
 */
static void usb_get_value_from_nvs(char *key, void *value, size_t *size)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("memory", NVS_READWRITE, &my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
    else
    {
        err = nvs_get_blob(my_handle, key, value, size);
        switch (err)
        {
            case ESP_OK:
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGI(TAG, "%s is not initialized yet!", key);
                break;
            default :
                ESP_LOGE(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
        }

        nvs_close(my_handle);
    }
}

/**
 * @brief       在nvs分区保存数值
 * @param       key     :名称
 * @param       value   :数据
 * @param       size    :大小
 * @retval      ESP_OK：设置成功；其他表示获取失败
 */
static esp_err_t usb_set_value_to_nvs(char *key, void *value, size_t size)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("memory", NVS_READWRITE, &my_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return ESP_FAIL;
    }
    else
    {
        err = nvs_set_blob(my_handle, key, value, size);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS set failed %s", esp_err_to_name(err));
        }

        err = nvs_commit(my_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS commit failed");
        }

        nvs_close(my_handle);
    }

    return err;
}

/**
 * @brief       USB数据流初始化
 * @param       无
 * @retval      ESP_OK：成功初始化；其他表示初始化失败
 */
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

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc streaming config failed");
    }
    return ret;
}

/**
 * @brief       查找USB摄像头当前的分辨率
 * @param       camera_frame_size :结构体
 * @retval      返回分辨率
 */
static size_t usb_camera_find_current_resolution(camera_frame_size_t *camera_frame_size)
{
    if (camera_resolution_info.camera_frame_list == NULL)
    {
        return -1;
    }

    size_t i = 0;
    while (i < camera_resolution_info.camera_frame_list_num)
    {
        if (camera_frame_size->width >= camera_resolution_info.camera_frame_list[i].width && camera_frame_size->height >= camera_resolution_info.camera_frame_list[i].height)
        {
            /* 查找下一个分辨率
               如果当前的分辨率最小，则切换到大的分辨率*/
            camera_frame_size->width = camera_resolution_info.camera_frame_list[i].width;
            camera_frame_size->height = camera_resolution_info.camera_frame_list[i].height;
            break;
        }
        else if (i == camera_resolution_info.camera_frame_list_num - 1)
        {
            camera_frame_size->width = camera_resolution_info.camera_frame_list[i].width;
            camera_frame_size->height = camera_resolution_info.camera_frame_list[i].height;
            break;
        }
        i++;
    }
    /* 打印当前分辨率 */
    ESP_LOGI(TAG, "Current resolution is %dx%d", camera_frame_size->width, camera_frame_size->height);
    return i;
}

/**
 * @brief       usb数据流回调函数
 * @param       event   : 事件
 * @param       arg     : 参数（未使用）
 * @retval      无
 */
static void usb_stream_state_changed_cd(usb_stream_state_t event,void *arg)
{
    switch(event)
    {
        /* 连接状态 */
        case STREAM_CONNECTED:
                /* 获取相机分辨率，并存储至nvs分区 */
                size_t size = sizeof(camera_frame_size_t);
                usb_get_value_from_nvs(DEMO_KEY_RESOLUTION, &camera_resolution_info.camera_frame_size, &size);
                size_t frame_index = 0;
                uvc_frame_size_list_get(NULL, &camera_resolution_info.camera_frame_list_num, NULL);

                if (camera_resolution_info.camera_frame_list_num)
                {
                    ESP_LOGI(TAG, "UVC: get frame list size = %u, current = %u", camera_resolution_info.camera_frame_list_num, frame_index);
                    uvc_frame_size_t *_frame_list = (uvc_frame_size_t *)malloc(camera_resolution_info.camera_frame_list_num * sizeof(uvc_frame_size_t));

                    camera_resolution_info.camera_frame_list = (uvc_frame_size_t *)realloc(camera_resolution_info.camera_frame_list, camera_resolution_info.camera_frame_list_num * sizeof(uvc_frame_size_t));
                    
                    if (NULL == camera_resolution_info.camera_frame_list)
                    {
                        ESP_LOGE(TAG, "camera_resolution_info.camera_frame_list");
                    }

                    uvc_frame_size_list_get(_frame_list, NULL, NULL);

                    for (size_t i = 0; i < camera_resolution_info.camera_frame_list_num; i++)
                    {
                        if (_frame_list[i].width <= lcd_dev.width && _frame_list[i].height <= lcd_dev.height)
                        {
                            camera_resolution_info.camera_frame_list[frame_index++] = _frame_list[i];
                            ESP_LOGI(TAG, "\tpick frame[%u] = %ux%u", i, _frame_list[i].width, _frame_list[i].height);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "\tdrop frame[%u] = %ux%u", i, _frame_list[i].width, _frame_list[i].height);
                        }
                    }
                    camera_resolution_info.camera_frame_list_num = frame_index;

                    if(camera_resolution_info.camera_frame_size.width != 0 && camera_resolution_info.camera_frame_size.height != 0) {
                        camera_resolution_info.camera_currect_frame_index = usb_camera_find_current_resolution(&camera_resolution_info.camera_frame_size);
                    }
                    else
                    {
                        camera_resolution_info.camera_currect_frame_index = 0;
                    }

                    if (-1 == camera_resolution_info.camera_currect_frame_index)
                    {
                        ESP_LOGE(TAG, "fine current resolution fail");
                        break;
                    }
                    ESP_ERROR_CHECK(uvc_frame_size_reset(camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].width,
                                                        camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].height, FPS2INTERVAL(30)));
                    camera_frame_size_t camera_frame_size = {
                        .width = camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].width,
                        .height = camera_resolution_info.camera_frame_list[camera_resolution_info.camera_currect_frame_index].height,
                    };

                    ESP_ERROR_CHECK(usb_set_value_to_nvs(DEMO_KEY_RESOLUTION, &camera_frame_size, sizeof(camera_frame_size_t)));

                    if (_frame_list != NULL)
                    {
                        free(_frame_list);
                    }
                    /* 等待USB摄像头连接 */
                    usb_streaming_control(STREAM_UVC, CTRL_RESUME, NULL);
                    xEventGroupSetBits(s_evt_handle, BIT0_FRAME_START);
                }
                else
                {
                    ESP_LOGW(TAG, "UVC: get frame list size = %u", camera_resolution_info.camera_frame_list_num);
                }
                /* 设备连接成功 */
                ESP_LOGI(TAG, "Device connected");
            break;
        /* 关闭连接 */
        case STREAM_DISCONNECTED:
                xEventGroupClearBits(s_evt_handle, BIT0_FRAME_START);
                /* 设备断开 */
                ESP_LOGI(TAG, "Device disconnected");
            break;
        default:
            ESP_LOGE(TAG, "Unknown event");
            break;
    }
}

/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;
    lcd_cfg_t lcd_config_info = {0};
    lcd_config_info.notify_flush_ready = NULL;
    lcd_config_info.user_ctx = NULL;

    ret = nvs_flash_init();             /* 初始化NVS */

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();                         /* 初始化LED */
    myiic_init();                       /* IIC初始化 */
    xl9555_init();                      /* 初始化按键 */
    lcd_init(lcd_config_info);          /* 初始化LCD */

    /* 创建事件组 */
    s_evt_handle = xEventGroupCreate();

    if (s_evt_handle == NULL)
    {
        ESP_LOGE(TAG, "line-%u event group create failed", __LINE__);
        assert(0);
    }

    /* 申请USB双缓冲 */
    xfer_buffer_a = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_a != NULL);
    xfer_buffer_b = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_b != NULL);

    /* mjpeg一帧缓冲 */
    frame_buffer = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(frame_buffer != NULL);

    /* 为ppbuffer_handle句柄申请缓冲 */
    ppbuffer_handle = (PingPongBuffer_t *)malloc(sizeof(PingPongBuffer_t));
    assert(ppbuffer_handle != NULL);

    /* 显示摄像头图形 */
    xTaskCreate(usb_display_task, "usb_display_task", 4 * 1024, NULL, 5, NULL);

    /* USB数据流初始化 */
    ESP_ERROR_CHECK(usb_stream_init());

    /* 注册回调函数 */
    ESP_ERROR_CHECK(usb_streaming_state_register(&usb_stream_state_changed_cd, NULL));

    /* 开启USB数据流转输  */
    ESP_ERROR_CHECK(usb_streaming_start());
    /* 等待连接  */
    ESP_ERROR_CHECK(usb_streaming_connect_wait(portMAX_DELAY));
}
