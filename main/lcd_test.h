/* LCD Test Header */

#ifndef LCD_TEST_H
#define LCD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LCD for testing
 */
void lcd_test_init(void);

/**
 * @brief Run LCD test pattern
 */
void lcd_test_run(void);

#ifdef __cplusplus
}
#endif

#endif // LCD_TEST_H
