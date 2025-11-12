/* General settings

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Board name setting refer to `codec_board` README.md for more details
 * @note   使用ESP32S3_BOX，该板型使用USB摄像头，不支持DVP摄像头
 */
#if CONFIG_IDF_TARGET_ESP32P4
#define TEST_BOARD_NAME "ESP32_P4_DEV_V14"
#else
#define TEST_BOARD_NAME "ESP32S3_BOX"  // 修改为ESP32S3_BOX
#endif

/**
 * @brief  Video resolution settings
 * @note   ESP32-S3使用USB摄像头，分辨率720p (1280x720)
 */
#if CONFIG_IDF_TARGET_ESP32P4
#define VIDEO_WIDTH  1920
#define VIDEO_HEIGHT 1080
#define VIDEO_FPS    25
#else
#define VIDEO_WIDTH  1280
#define VIDEO_HEIGHT 720
#define VIDEO_FPS    15
#endif

/**
 * @brief  Set for wifi ssid
 * @note   需要修改为您的实际WiFi名称
 */
#define WIFI_SSID     "XXXX"

/**
 * @brief  Set for wifi password
 * @note   需要修改为您的实际WiFi密码
 */
#define WIFI_PASSWORD "XXXX"

/**
 * @brief  Whether enable data channel
 */
#define DATA_CHANNEL_ENABLED (false)

#if CONFIG_IDF_TARGET_ESP32P4
/**
 * @brief  GPIO for ring button
 *
 * @note  When use ESP32P4-Fuction-Ev-Board, GPIO35(boot button) is connected RMII_TXD1
 *        When enable `NETWORK_USE_ETHERNET` will cause socket error
 *        User must replace it to a unused GPIO instead (like GPIO27)
 */
#define DOOR_BELL_RING_BUTTON  35
#else
/**
 * @brief  GPIO for ring button
 *
 * @note  ESP32S3_BOX使用GPIO0作为BOOT按钮
 *        可用作门铃按钮触发
 */
#define DOOR_BELL_RING_BUTTON  0  // ESP32S3_BOX的BOOT按钮
#endif

#ifdef __cplusplus
}
#endif
