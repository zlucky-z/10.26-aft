/*
 * 模块功能：SDK计时辅助接口。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#ifndef NODE_TIMER_H
#define NODE_TIMER_H

// 返回单调时钟的毫秒计数。
uint32_t getms();
// 返回系统时钟的纳秒计数。
uint64_t getTime();
// 阻塞指定毫秒数，用于SDK串口轮询和节拍控制。
void delay(uint32_t ms);

#endif
