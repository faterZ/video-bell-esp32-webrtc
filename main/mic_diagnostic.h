#ifndef MIC_DIAGNOSTIC_H
#define MIC_DIAGNOSTIC_H

/**
 * 运行完整的麦克风诊断测试
 * 会尝试所有可能的 I2S 配置，找出正确的麦克风连接方式
 */
void mic_diagnostic_run(void);

#endif // MIC_DIAGNOSTIC_H
