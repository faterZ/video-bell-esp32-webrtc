/* LCD测试模块 - 用于ESP32-S3 Box硬件验证 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "../components/BSP/LCD/lcd.h"
#include "../components/BSP/MYIIC/myiic.h"
#include "../components/BSP/XL9555/xl9555.h"

static const char *TAG = "LCD_TEST";

/**
 * @brief LCD测试初始化
 * 初始化IIC总线、XL9555 IO扩展器和LCD显示屏
 */
void lcd_test_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD test...");
    
    // 初始化IIC总线和XL9555 (控制LCD背光的IO扩展器)
    myiic_init();
    xl9555_init();
    
    // 初始化LCD显示屏
    lcd_cfg_t lcd_config = {
        .user_ctx = NULL,
        .notify_flush_ready = NULL,
    };
    lcd_init(lcd_config);
    
    ESP_LOGI(TAG, "LCD initialized: %dx%d", lcd_dev.width, lcd_dev.height);
}

/**
 * @brief LCD测试程序
 * 执行一系列测试：颜色填充、图形绘制、文本显示
 */
void lcd_test_run(void)
{
    ESP_LOGI(TAG, "Running LCD test pattern...");
    
    // 测试1: 红色填充
    ESP_LOGI(TAG, "Test 1: Red screen");
    lcd_clear(RED);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试2: 绿色填充
    ESP_LOGI(TAG, "Test 2: Green screen");
    lcd_clear(GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试3: 蓝色填充
    ESP_LOGI(TAG, "Test 3: Blue screen");
    lcd_clear(BLUE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试4: 白色填充
    ESP_LOGI(TAG, "Test 4: White screen");
    lcd_clear(WHITE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试5: 绘制图形
    ESP_LOGI(TAG, "Test 5: Drawing shapes");
    lcd_clear(BLACK);
    
    // 绘制矩形
    lcd_draw_rectangle(10, 10, 100, 100, RED);
    lcd_draw_rectangle(120, 10, 210, 100, GREEN);
    lcd_draw_rectangle(10, 120, 100, 210, BLUE);
    lcd_draw_rectangle(120, 120, 210, 210, YELLOW);
    
    // 绘制圆形
    lcd_draw_circle(lcd_dev.width / 2, lcd_dev.height / 2, 50, WHITE);
    lcd_draw_circle(lcd_dev.width / 2, lcd_dev.height / 2, 30, CYAN);
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试6: 显示文本
    ESP_LOGI(TAG, "Test 6: Displaying text");
    lcd_clear(BLACK);
    lcd_show_string(10, 10, 200, 24, 24, "ESP32-S3 Box", WHITE);
    lcd_show_string(10, 40, 200, 24, 24, "LCD Test OK!", GREEN);
    lcd_show_string(10, 70, 200, 16, 16, "USB Camera Ready", YELLOW);
    
    ESP_LOGI(TAG, "LCD test completed!");
}

#else
// 非ESP32-S3平台的空实现
void lcd_test_init(void)
{
    ESP_LOGW("LCD_TEST", "LCD test only available on ESP32-S3");
}

void lcd_test_run(void)
{
    ESP_LOGW("LCD_TEST", "LCD test skipped (not ESP32-S3)");
}
#endif
