/**
 ****************************************************************************************************
 * @file        recorder.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       wav解码 代码
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

#ifndef __RECORDER_H
#define __RECORDER_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "audioplay.h"
#include "ff.h"
#include "xl9555.h"
#include "driver/i2s.h"
#include "led.h"
#include "wavplay.h"
#include "text.h"
#include "myi2s.h"
#include "key.h"

#define REC_RX_BUF_SIZE     4*1024        /* 定义RX 数组大小 */

/* 函数声明 */
void recoder_enter_rec_mode(void);
void recoder_wav_init(__WaveHeader *wavhead);
void recoder_msg_show(uint32_t tsec, uint32_t kbps);
void recoder_remindmsg_show(uint8_t mode);
void recoder_new_pathname(uint8_t *pname);
void wav_recorder(void);

#endif
