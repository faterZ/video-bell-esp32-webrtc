/* Do simple board initialize

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "esp_log.h"
#include "codec_init.h"
#include "codec_board.h"
#include "esp_codec_dev.h"
#include "sdkconfig.h"
#include "settings.h"
#include "myiic.h"
#include "xl9555.h"

static const char *TAG = "Board";

void init_board()
{
    ESP_LOGI(TAG, "Init board (%s).", TEST_BOARD_NAME);
    set_codec_board_type(TEST_BOARD_NAME);

    // Ensure IO expander is ready so audio rails get power
    esp_err_t xl9555_ret = xl9555_init();
    if (xl9555_ret != ESP_OK) {
        ESP_LOGE(TAG, "XL9555 init failed (%s)", esp_err_to_name(xl9555_ret));
    } else {
        // Enable speaker / mic supply path (active high)
        xl9555_pin_write(SPK_CTRL_IO, 1);
    }

    // ESP32-S3-BOX uses ES8311 (DAC) + ES7210 (ADC over TDM)
    codec_init_cfg_t cfg = {
        .in_mode = CODEC_I2S_MODE_STD,
        .out_mode = CODEC_I2S_MODE_STD,
        .in_use_tdm = false,
        .reuse_dev = true,
    };

    int ret = init_codec(&cfg);
    if (ret != 0) {
        ESP_LOGE(TAG, "Codec init failed (%d)", ret);
    } else {
        ESP_LOGI(TAG, "Codec init succeed");
    }
}
