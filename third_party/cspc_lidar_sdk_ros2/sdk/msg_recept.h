/*
 * 模块功能：本文件负责third_party/cspc_lidar_sdk_ros2/sdk/msg_recept.h对应的运行、构建或配置职责。
 * 修改提示：本文件只负责上述职责，修改时保持公共接口和数据语义不变。
 */
#ifndef __H_MESSAGE_RECEPT_H__
#define __H_MESSAGE_RECEPT_H__

// 查询雷达版本信息是否已经完成并可供后续初始化使用。
bool get_version_t();
// 判断雷达加密/解密处理是否处于进行中。
bool encry_doing();


#endif
