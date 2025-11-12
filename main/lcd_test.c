/* LCD Test for ESP32-S3 Box */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "../components/BSP/LCD/lcd.h"
#include "../components/BSP/MYIIC/myiic.h"
#include "../components/BSP/XL9555/xl9555.h"

static const char *TAG = "LCD_TEST";

void lcd_test_init(void)
{
    ESP_LOGI(TAG, "Initializing LCD test...");
    
    // Initialize IIC and XL9555 (IO expander for LCD backlight control)
    myiic_init();
    xl9555_init();
    
    // Initialize LCD
    lcd_cfg_t lcd_config = {
        .user_ctx = NULL,
        .notify_flush_ready = NULL,
    };
    lcd_init(lcd_config);
    
    ESP_LOGI(TAG, "LCD initialized: %dx%d", lcd_dev.width, lcd_dev.height);
}

void lcd_test_run(void)
{
    ESP_LOGI(TAG, "Running LCD test pattern...");
    
    // Test 1: Fill screen with different colors
    ESP_LOGI(TAG, "Test 1: Red screen");
    lcd_clear(RED);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Test 2: Green screen");
    lcd_clear(GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Test 3: Blue screen");
    lcd_clear(BLUE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Test 4: White screen");
    lcd_clear(WHITE);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test 2: Draw some shapes
    ESP_LOGI(TAG, "Test 5: Drawing shapes");
    lcd_clear(BLACK);
    
    // Draw rectangles
    lcd_draw_rectangle(10, 10, 100, 100, RED);
    lcd_draw_rectangle(120, 10, 210, 100, GREEN);
    lcd_draw_rectangle(10, 120, 100, 210, BLUE);
    lcd_draw_rectangle(120, 120, 210, 210, YELLOW);
    
    // Draw circles
    lcd_draw_circle(lcd_dev.width / 2, lcd_dev.height / 2, 50, WHITE);
    lcd_draw_circle(lcd_dev.width / 2, lcd_dev.height / 2, 30, CYAN);
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Test 3: Display text
    ESP_LOGI(TAG, "Test 6: Displaying text");
    lcd_clear(BLACK);
    lcd_show_string(10, 10, 200, 24, 24, "ESP32-S3 Box", WHITE);
    lcd_show_string(10, 40, 200, 24, 24, "LCD Test OK!", GREEN);
    lcd_show_string(10, 70, 200, 16, 16, "USB Camera Ready", YELLOW);
    
    ESP_LOGI(TAG, "LCD test completed!");
}

#else
void lcd_test_init(void)
{
    ESP_LOGW("LCD_TEST", "LCD test only available on ESP32-S3");
}

void lcd_test_run(void)
{
    ESP_LOGW("LCD_TEST", "LCD test skipped (not ESP32-S3)");
}
#endif
