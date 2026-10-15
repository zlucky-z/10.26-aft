# DWA C 版整机（10.15）

本仓库保存当前回退并部署到板端的 C 版单进程 DWA 工程。该快照对应 2026-10-14 的稳定版本：已包含新板串口映射和 BMI270 I²C 总线修正，但不包含 2026-10-15 后续尝试的共享脱困策略改动。

## 设备映射

- 左雷达：`/dev/ttyS4`
- 右雷达：`/dev/ttyS5`
- 电机控制器：`/dev/ttyS6`
- BMI270 IMU：`/dev/i2c-3`（本板实际接线；不是 `/dev/i2c-4`）

程序入口中的串口默认值和 IMU 驱动中的 I²C 节点均按上述板端映射配置。更换硬件接线时，应同步检查 `c/apps/robot_main.c` 与 `c/src/drivers/bmi270.h`。

## 目录

- `c/`：以 C 实现的规划、安全、感知、融合、电机桥、运行时、应用入口和回归测试；`c/src/drivers/` 中的少量 C++ 文件仅负责调用雷达 SDK。
- `config/dwa_planner.yaml`：规划与安全参数。
- `config/lidar_filter.yaml`：双雷达滤波参数。
- `third_party/cspc_lidar_sdk_ros2/sdk/`：COIN-D6 雷达供应商 SDK；SDK 自带的 C++ 源码是雷达适配所需依赖，不是旧版 C++ DWA 规划器或电机桥。

旧的 C++ DWA 规划器、电机桥实现、ROS 启动链、旧对拍程序和历史分析文件不属于本版本。除上述雷达适配层外，C++ 源码仅为 `third_party/` 下的供应商 SDK。

## 构建与测试

### Linux 宿主机回归测试

```bash
cmake -S c -B build-c \
  -DDWA_C_ENABLE_HOST_TOOLS=ON \
  -DCMAKE_C_STANDARD_LIBRARIES=-lm \
  -DCMAKE_CXX_STANDARD_LIBRARIES=-lm
cmake --build build-c
ctest --test-dir build-c --output-on-failure
```

Linux GCC 链接数学库需要上述 `-lm` 配置。Windows/MinGW 构建时可省略这两个标准库参数。

### ARM64 板端交叉编译

```bash
cmake -S c -B build-aarch64 \
  -DCMAKE_TOOLCHAIN_FILE=c/cmake/aarch64.cmake \
  -DDWA_C_ENABLE_HOST_TOOLS=OFF
cmake --build build-aarch64
```

主要产物包括：`robot_main`（整机单进程入口）、`dwa_main`（规划/回放入口）和 `bridge_main`（C 版电机桥入口）。仓库只跟踪源码、配置和测试，不提交本地构建目录或板端二进制。

## 板端启动

部署 `robot_main` 到板端 `bin/robot_main`，并将 `config/` 下配置文件放到对应位置。仓库内 C 版启动脚本：

```bash
./c/apps/run_robot.sh
```

接电机和雷达进行实车测试前，确认串口未被其他进程占用，并准备可立即使用的物理急停。不要在无人监管或无法立即断电时启动运动链路。

## 前向安全基线

- 碰撞前缘采用脚尖前缘 `toe_front = 0.90 m`。
- 前向安全候选参与净距与硬停止判断，避免贴近脚踏区域的障碍被通用自车过滤整体剔除。
- 脚尖纵向碰撞余量为 `toe_collision_margin_m = 0.10 m`；不扩大侧向碰撞宽度，避免将通道弯道墙的侧向包络整体提前。

该仓库描述的是上述回退版本，不应将后续未验证的脱困策略实验视作此基线的一部分。
