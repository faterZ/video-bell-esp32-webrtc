/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-06-25
 * @brief       录音机 实验
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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "led.h"
#include "key.h"
#include "my_spi.h"
#include "myiic.h"
#include "lcd.h"
#include "xl9555.h"
#include "fonts.h"
#include "spi_sd.h"
#include "text.h"
#include "exfuns.h"
#include "audioplay.h"
#include "recorder.h"
#include "myi2s.h"


/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;
    uint8_t key = 0;

    ret = nvs_flash_init();                             /* 初始化NVS */
    lcd_cfg_t lcd_config_info = {0};
    lcd_config_info.notify_flush_ready = NULL;

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    led_init();                                         /* 初始化LED */
    key_init();                                         /* 初始化按键 */
    myiic_init();                                       /* 初始化IIC */
    my_spi_init();                                      /* 初始化SPI */
    xl9555_init();                                      /* 初始化按键 */ 
    lcd_init(lcd_config_info);                          /* 初始化LCD */
    xl9555_pin_write(SPK_CTRL_IO, 1);                   /* 打开喇叭 */

    while (sd_spi_init())                               /* 检测不到SD卡 */
    {
        lcd_show_string(30, 120, 200, 16, 16, "SD Card Error!", RED);
        vTaskDelay(500);
        lcd_show_string(30, 140, 200, 16, 16, "Please Check! ", RED);
        vTaskDelay(500);
    }

    while (fonts_init())                                /* 检查字库 */
    {
        lcd_clear(WHITE);                               /* 清屏 */
        lcd_show_string(30, 30, 200, 16, 16, "ESP32-S3", RED);
        
        key = fonts_update_font(30, 50, 16, (uint8_t *)"0:", RED);  /* 更新字库 */

        while (key)                                     /* 更新失败 */
        {
            lcd_show_string(30, 50, 200, 16, 16, "Font Update Failed!", RED);
            vTaskDelay(200);
            lcd_fill(20, 50, 200 + 20, 90 + 16, WHITE);
            vTaskDelay(200);
        }

        lcd_show_string(30, 50, 200, 16, 16, "Font Update Success!   ", RED);
        vTaskDelay(1500);
        lcd_clear(WHITE);                               /* 清屏 */
    }
    
    ret = exfuns_init();                                /* 为fatfs相关变量申请内存 */
    vTaskDelay(500);                                    /* 实验信息显示延时 */

    text_show_string(30, 50, 200, 16, "正点原子ESP32S3 BOX", 16, 0, RED);
    text_show_string(30, 70, 200, 16, "WAV 录音机 实验", 16, 0, RED);
    text_show_string(30, 90, 200, 16, "ATOM@ALIENTEK", 16, 0, RED);

    while (1)
    {
        wav_recorder();                                 /* 录音 */
    }
}
