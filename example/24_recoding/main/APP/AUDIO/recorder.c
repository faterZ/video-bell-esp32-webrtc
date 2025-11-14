/**
 ****************************************************************************************************
 * @file        recorder.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       wav解码 代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "recorder.h"


uint32_t g_wav_size = 0;    /* wav数据大小(字节数,不包括文件头!!) */

uint8_t g_rec_sta = 0;  /**
                         * 录音状态
                         * [7]:0,没有开启录音;1,已经开启录音;
                         * [6:1]:保留
                         * [0]:0,正在录音;1,暂停录音;
                         */

SemaphoreHandle_t i2s_mutex;
static QueueSetHandle_t xQueueSet;  /* 定义队列集 */
SemaphoreHandle_t key0_xSemaphore;  /* 定义二值信号量 */
SemaphoreHandle_t key1_xSemaphore;  /* 定义二值信号量 */
SemaphoreHandle_t key2_xSemaphore;  /* 定义二值信号量 */
__WaveHeader *wavhead = NULL;       /* wav头部指针 */
uint32_t recsec = 0;                /* 录音时间 */
/* 双缓冲机制 */
uint8_t *current_buffer = NULL;     /* 当前缓存 */
uint8_t *next_buffer = NULL;        /* 写缓存 */


/**
 * @brief       进入PCM 录音模式
 * @param       无
 * @retval      无
 */
void recoder_enter_rec_mode(void)
{
    myi2s_init();                 /* 初始化I2S */
    i2s_set_samplerate_bits_sample(I2S_SAMPLE_RATE,I2S_BITS_PER_SAMPLE_16BIT);    /* 初始化I2S */
    i2s_trx_start();                /* 开启I2S */
    recoder_remindmsg_show(0);
}

/**
 * @brief       进入PCM 放音模式
 * @param       无
 * @retval      无
 */
void recoder_enter_play_mode(void)
{
    i2s_trx_stop();                 /* 停止录音 */
    i2s_deinit();                   /* 卸载I2S */
    recoder_remindmsg_show(1);      /* 显示提示信息 */
}

/**
 * @brief       初始化WAV头
 * @param       wavhead : wav文件头指针
 * @retval      无
 */
void recoder_wav_init(__WaveHeader *wavhead)
{
    wavhead->riff.ChunkID = 0x46464952;                  /* "RIFF" */
    wavhead->riff.ChunkSize = 0;                         /* 还未确定,最后需要计算 */
    wavhead->riff.Format = 0x45564157;                   /* "WAVE" */
    wavhead->fmt.ChunkID = 0x20746D66;                   /* "fmt " */
    wavhead->fmt.ChunkSize = 16;                         /* PCM格式块大小为16个字节 */
    wavhead->fmt.AudioFormat = 0x01;                     /* 0x01,表示PCM; 0x00,表示IMA ADPCM */
    wavhead->fmt.NumOfChannels = 2;                      /* 双声道 */
    wavhead->fmt.SampleRate = 16000;                     /* 采样速率为16000Hz */
    wavhead->fmt.BitsPerSample = 16;                     /* 16位PCM */
    wavhead->fmt.BlockAlign = wavhead->fmt.NumOfChannels * (wavhead->fmt.BitsPerSample / 8);  /* 块对齐=通道数*每个样本的字节数 */
    wavhead->fmt.ByteRate = wavhead->fmt.SampleRate * wavhead->fmt.BlockAlign; /* 字节速率=采样率*块对齐 */
    wavhead->data.ChunkID = 0x61746164;                  /* "data" */
    wavhead->data.ChunkSize = 0;                         /* 数据大小,最后需要计算 */
}

/**
 * @brief       显示录音时间和码率
 * @param       tsec : 时间（单位 : 秒）
 * @param       kbps : 码率
 * @retval      无
 */
void recoder_msg_show(uint32_t tsec, uint32_t kbps)
{
    lcd_show_string(30, 210, 200, 16, 16, "TIME:", RED);        /* 显示录音时间 */
    lcd_show_xnum(30 + 40, 210, tsec / 60, 2, 16, 0x80, RED);   /* 分钟 */
    lcd_show_char(30 + 56, 210, ':', 16, 0, RED);
    lcd_show_xnum(30 + 64, 210, tsec % 60, 2, 16, 0x80, RED);   /* 秒钟 */

    lcd_show_string(140, 210, 200, 16, 16, "KPBS:", RED);   /* 显示码率 */
    lcd_show_num(140 + 40, 210, kbps / 1000, 4, 16, RED);   /* 码率显示 */
}

/**
 * @brief       提示信息
 * @param       mode : 工作模式
 *   @arg       0, 录音模式
 *   @arg       1, 放音模式
 * @retval      无
 */
void recoder_remindmsg_show(uint8_t mode)
{
    lcd_fill(30, 120, lcd_dev.width, lcd_dev.height, WHITE);            /* 清除原来的显示 */

    if (mode == 0)  /* 录音模式 */
    {
        lcd_show_string(30, 120, 200, 16, 16, "KEY0:REC/PAUSE", BLUE);
        lcd_show_string(30, 140, 200, 16, 16, "KEY1:STOP&SAVE", BLUE);
        lcd_show_string(30, 160, 200, 16, 16, "KEY2:PLAY", BLUE);
    }
    else            /* 放音模式 */
    {
        lcd_show_string(30, 120, 200, 16, 16, "KEY0:STOP Play", BLUE);
        lcd_show_string(30, 140, 200, 16, 16, "KEY2:PLAY/PAUSE", BLUE);
    }
} 

/**
 * @brief       通过时间获取文件名
 * @note        仅限在SD卡保存,不支持FLASH DISK保存
 * @note        组合成:形如"0:RECORDER/REC00001.wav"的文件名
 * @param       pname : 文件路径
 * @retval      无
 */
void recoder_new_pathname(uint8_t *pname)
{
    uint8_t res;
    uint16_t index = 0;
    FIL *ftemp;
    ftemp = (FIL *)malloc(sizeof(FIL));           /* 开辟FIL字节的内存区域 */

    if (ftemp == NULL) 
    {
        return;  /* 内存申请失败 */
    }

    while (index < 0xFFFF)
    {
        sprintf((char *)pname, "0:RECORDER/REC%05d.wav", index);
        res = f_open(ftemp, (const TCHAR *)pname, FA_READ); /* 尝试打开这个文件 */

        if (res == FR_NO_FILE)
        {
            break;              /* 该文件名不存在=正是我们需要的. */
        }

        index++;
    }

    free(ftemp);
}

/**
 * @brief       WAV录音任务
 * @param       arg:传入任务句柄
 * @retval      无
 */
static void recorder_task(void * arg)
{
    QueueSetMemberHandle_t activate_member = NULL;
    UINT bw;
    size_t bytes_read = 0;                  /* 读取写入录音文件大小 */
    uint8_t file_write_res = 0;             /* 上一次写入完成标志 */
    uint8_t rval = 0;                       /* 获取TF状态 */
    FF_DIR recdir;                          /* 目录 */
    FIL *f_rec = NULL;                      /* 录音文件 */
    static uint8_t *pdatabuf = NULL;        /* 数据缓存指针 */
    static uint8_t *pdatabuf1 = NULL;       /* 数据缓存指针 */
    uint8_t *pname = NULL;                  /* 文件名称存储buf */
    FSIZE_t recorder_file_read_pos = 0;     /* 记录当前录音文件读取位置 */
    uint32_t g_wav_size_tatol = 0;          /* wav数据大小(字节数,不包括文件头!!) */

    /* 资源申请内存 */
    pdatabuf = malloc(REC_RX_BUF_SIZE);                     /* 存储录音数据 */
    pdatabuf1 = malloc(REC_RX_BUF_SIZE);                     /* 存储录音数据 */
    f_rec = (FIL*)malloc(sizeof(FIL));                      /* 文件指针 */
    wavhead = (__WaveHeader *)malloc(sizeof(__WaveHeader)); /* wav头部 */
    pname = malloc(255);                                    /* 存储文件名称buf */

    if (!f_rec || !wavhead || !pname || !pdatabuf || !pdatabuf1)
    {
        goto exit;
    }
    
    memset(pdatabuf,0,REC_RX_BUF_SIZE);
    memset(pdatabuf1,0,REC_RX_BUF_SIZE);

    current_buffer = pdatabuf;
    next_buffer = pdatabuf1;

    /* 打开文件夹，若没有，则自动创建 */
    while (f_opendir(&recdir, "0:/RECORDER"))
    {
        f_mkdir("0:/RECORDER"); /* 创建该目录 */
    }

    xTaskNotifyGive(arg);
    /* 创建互斥锁，防止数据存储干扰 */
    i2s_mutex = xSemaphoreCreateRecursiveMutex();
    xSemaphoreGiveRecursive(i2s_mutex);

    recoder_enter_rec_mode();   /* 进入录音模式 */
    pname[0] = 0;               /* pname没有任何文件名 */

    while (rval == 0)
    {
        activate_member = xQueueSelectFromSet(xQueueSet, 10);/* 等待队列集中的队列接收到消息 */

        if (activate_member == key0_xSemaphore)
        {
            xSemaphoreTake(activate_member, portMAX_DELAY);

            if (g_rec_sta & 0x01 && recorder_file_read_pos != 0 && file_write_res == 1) /* 暂停录音 */
            {
                g_rec_sta &= 0xFE;          /* 恢复录音 */
            }
            else if (g_rec_sta & 0x80 && file_write_res == 1)   /* 正在录音 */
            {
                g_rec_sta |= 0x01;          /* 暂停录音 */
                recorder_file_read_pos = f_tell(f_rec);         /* 记录当前文件写到那个位置 */
            }
            else                            /* 未开始录音 */
            {
                recsec = 0;
                recoder_new_pathname(pname);
                text_show_string(30, 190, lcd_dev.width, 16, "录制:", 16, 0, RED);
                text_show_string(30 + 40, 190, lcd_dev.width, 16, (char *)pname + 11, 16, 0, RED);
                recoder_wav_init(wavhead);  /* 初始化wav头 */
                i2s_set_samplerate_bits_sample(wavhead->fmt.SampleRate,I2S_BITS_PER_SAMPLE_16BIT);    /* 初始化I2S */
                i2s_trx_start();            /* 开启I2S */
                rval = f_open(f_rec, (const TCHAR *)pname, FA_CREATE_ALWAYS | FA_WRITE);
                /* 打开文件失败 */
                if (rval != FR_OK)
                {
                    g_rec_sta = 0;          /* 关闭录音 */
                    rval = 0xFE;            /* 提示SD卡问题 */
                }
                else
                {
                    f_lseek(f_rec, sizeof(__WaveHeader));   /* 偏移到数据存储地址 */
                    recoder_msg_show(0, 0); /* 提示录音时长 */
                    g_rec_sta |= 0x80;      /* 开始录音 */
                }
            }
        }
        else if (activate_member == key1_xSemaphore)
        {
            xSemaphoreTake(activate_member, portMAX_DELAY);
            if ((g_rec_sta & 0x80 )&& (file_write_res == 1) &&(g_wav_size_tatol == g_wav_size)) /* 有录音且上一次录音写入完成 */
            {
                xSemaphoreTakeRecursive(i2s_mutex, portMAX_DELAY);
                f_lseek(f_rec, 0);                          /* 偏移到文件头 */
                g_rec_sta = 0;
                wavhead->riff.ChunkSize = g_wav_size + 36;  /* 整个文件的大小-8; */
                wavhead->data.ChunkSize = g_wav_size;       /* 数据大小 */
                f_write(f_rec, (const void *)wavhead, sizeof(__WaveHeader), (UINT *)&bw); /* 写入头数据 */
                f_close(f_rec);
                g_wav_size_tatol = 0;
                g_wav_size = 0;
                xSemaphoreGiveRecursive(i2s_mutex);
            }
            g_rec_sta = 0;
            recsec = 0;
            lcd_fill(30, 190, lcd_dev.width, lcd_dev.height, WHITE); /* 清除显示 */
        }
        else if (activate_member == key2_xSemaphore)
        {
            xSemaphoreTake(activate_member, portMAX_DELAY);
            if (g_rec_sta == 0 && file_write_res == 1)
            {
                text_show_string(30, 190, lcd_dev.width, 16, "播放:", 16, 0, RED);
                text_show_string(30 + 40, 190, lcd_dev.width, 16, (char *)pname + 11, 16, 0, RED);
                recoder_enter_play_mode();  /* 进入播放模式 */
                audio_play_song(pname);     /* 播放录音 */
                lcd_fill(30, 190, lcd_dev.width, lcd_dev.height, WHITE); /* 清除显示 */
                recoder_enter_rec_mode();   /* 回到录音模式 */
                g_rec_sta = 0;
                bytes_read = 0;
                memset(current_buffer,0,REC_RX_BUF_SIZE);
            }
        }

        /* 处理录音中数据写入与状态显示 */
        if (g_rec_sta & 0x80 && !(g_rec_sta & 0x01))
        {
            xSemaphoreTakeRecursive(i2s_mutex, portMAX_DELAY);
            file_write_res = 0;
            bytes_read = i2s_rx_read(current_buffer, REC_RX_BUF_SIZE);    /* 读取录音文件 */
            g_wav_size_tatol += bytes_read;

            if (bytes_read == REC_RX_BUF_SIZE)
            {
                if (recorder_file_read_pos != 0)
                {
                    f_lseek(f_rec, recorder_file_read_pos);
                    recorder_file_read_pos = 0;
                }
                /* 写入到文件当中 */
                rval = f_write(f_rec, current_buffer, REC_RX_BUF_SIZE, (UINT*)&bw);

                if (rval != FR_OK)
                {
                    printf("write error:%d\r\n", rval);
                }
                else
                {
                    file_write_res = 1;
                    g_wav_size += bytes_read;
                    /* 切换缓冲区 */
                    uint8_t *temp = current_buffer;
                    current_buffer = next_buffer;
                    next_buffer = temp;
                }
            }

            xSemaphoreGiveRecursive(i2s_mutex);
        }

    }

exit:
    /* 清理资源 */
    free(pdatabuf);
    free(f_rec);
    free(wavhead);
    free(pname);
    /* 删除任务 */
    vTaskDelete(NULL);
}

/**
 * @brief       WAV录音
 * @param       无
 * @retval      无
 */
void wav_recorder(void)
{
    uint8_t key = 0;
    uint8_t key1 = 0;
    uint8_t timecnt = 0;       /* 计时器 */
    /* 创建录音线程 */
    BaseType_t task_created = xTaskCreatePinnedToCore(recorder_task,
                                                     "recorder_task",
                                                     4096,
                                                     xTaskGetCurrentTaskHandle(),
                                                     10, NULL, 0);
    assert(task_created == pdTRUE);
    
    /* 等待录音线程的通知继续 */
    ulTaskNotifyTake(false, portMAX_DELAY);
    /* 创建队列集 */
    xQueueSet = xQueueCreateSet(3);
    /* 创建信号量 */
    key0_xSemaphore = xSemaphoreCreateBinary();
    key1_xSemaphore = xSemaphoreCreateBinary();
    key2_xSemaphore = xSemaphoreCreateBinary();
    /* 把信号量加入到队列集 */
    xQueueAddToSet(key0_xSemaphore, xQueueSet);
    xQueueAddToSet(key1_xSemaphore, xQueueSet);
    xQueueAddToSet(key2_xSemaphore, xQueueSet);

    while (1)
    {
        key1 = xl9555_key_scan(0);
        key = key_scan(0);

        switch (key1)
        {
            case KEY0_PRES:
                xSemaphoreGive(key0_xSemaphore);
                break;
            case KEY1_PRES:
                xSemaphoreGive(key1_xSemaphore);
                break;
            default:
                break;
        }

        if (key == BOOT_PRES)
        {
            xSemaphoreGive(key2_xSemaphore);
        }

        uint32_t current_rec_time = g_wav_size / (16000 * 4);

        timecnt++;

        if (timecnt % 20 == 0)
        {
            LED_TOGGLE();  /* 闪烁提示 */
        }

        if (recsec != current_rec_time)
        {
            recsec = current_rec_time;
            recoder_msg_show(recsec, wavhead->fmt.SampleRate * wavhead->fmt.NumOfChannels * wavhead->fmt.BitsPerSample);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
