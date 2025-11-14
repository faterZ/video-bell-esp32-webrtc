/**
 * 麦克风回音测试头文件
 */

#ifndef __MIC_LOOPBACK_TEST_H__
#define __MIC_LOOPBACK_TEST_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动麦克风回音测试
 *        麦克风录到的声音会直接通过扬声器播放
 */
void mic_loopback_start(void);

/**
 * @brief 停止麦克风回音测试
 */
void mic_loopback_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MIC_LOOPBACK_TEST_H__ */
