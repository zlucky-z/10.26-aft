# Automoving DWA 10.25

这是当前板端测试使用的 C 版单进程自动移动工程。程序入口 `robot_main` 将双雷达、滤波/融合、DWA、安全控制、IMU 和电机桥收拢到一个进程中。规划与安全模块共享统一的融合后 `base_link` 点云；供应商雷达 SDK 仍以 C++ 编译，其他主控制链路为 C 实现。

## 当前版本状态

- 车体主体宽度：`0.59 m`，主体半宽：`0.295 m`。
- 左右雷达安装位置：`x=0.545 m`、`y=+/-0.290 m`。
- 脚踏板包络：`x=0.57..0.75 m`、半宽 `0.23 m`。
- 脚尖安全包络：前缘 `x=0.90 m`、半宽 `0.23 m`。
- 动态速度融合状态机关闭：`dynamic_velocity_fusion_enabled: false`。
- DWA 运动接管触发距离：`0.40 m`。
- 几何二次轨迹选择关闭：`dwa_geometric_selector_enabled: false`。
- 通道入口/中心线软引导开启，但最终安全判断仍由 DWA 碰撞检查负责。
- 预测侧扫掠保护关闭：`side_hard_stop_sweep_enabled: false`。
- 旧侧边圆形硬停止关闭：`side_hard_stop_circle_enabled: false`。
- 当前侧边保护使用三角包络保护：`side_hard_stop_triangle_enabled: true`，距车体外沿的停止间隙为 `0.20 m`。

当前工作树包含未提交的 10.25 版本源码修改和对应回归测试；本 README 按当前配置和源码实际状态编写。

## 硬件接口

板端默认设备映射：

- 左雷达：`/dev/ttyS4`
- 右雷达：`/dev/ttyS5`
- 电机控制器：`/dev/ttyS6`
- BMI270 IMU：`/dev/i2c-3`

设备路径可通过 `robot_main` 参数覆盖。雷达波特率和 SDK 初始化参数位于 `c/apps/robot_main.c` 及雷达适配层中。更换板端接线时，必须同步确认串口和 I2C 节点，不能只替换 YAML。

## 目录结构

- `c/apps/`：`robot_main` 整机入口、`dwa_main` 回放入口、`bridge_main` 电机桥入口和启动脚本。
- `c/src/common/`：基础类型、几何和点云辅助代码。
- `c/src/config/`：YAML 参数解析和规划器参数默认值。
- `c/src/drivers/`：BMI270 和 COIN-D6 双雷达适配。
- `c/src/perception/`：雷达滤波、融合点云和占用栅格输入。
- `c/src/planning/`：DWA 采样、轨迹预测、碰撞检查和评分。
- `c/src/safety/`：侧边三角保护、前方安全、侧墙保持、脱困和原地转弯限速圆。
- `c/src/fusion/`：速度、档位和 DWA 接管状态逻辑。
- `c/src/motor/`：控制器协议、电机桥、Hall 速度控制和 IMU 补偿。
- `c/src/runtime/`：DWA 主循环、安全发布链路和诊断。
- `c/tests/`：宿主机回归测试。
- `config/dwa_planner.yaml`：规划、安全、车体和控制参数。
- `config/lidar_filter.yaml`：双雷达滤波参数。
- `third_party/cspc_lidar_sdk_ros2/`：COIN-D6 供应商 SDK。

本仓库不包含板端二进制、运行日志、本地构建目录或部署暂存包。

## 点云链路

生产控制链路使用同一份融合点云：

```text
左/右雷达采集
    -> 坐标转换到 base_link
    -> 雷达滤波、无效点和重复点处理
    -> 一次性自车包络过滤
    -> 融合点云
    -> DWA、限速圆和侧边三角保护共享
```

侧边保护和限速圆不再从另一份独立的障碍表复制数据。自车过滤仍按主体、脚踏板、脚尖包络分层处理；真实障碍不能因为靠近雷达而被固定的 `10 cm` 圆形规则直接屏蔽。

## 控制与保护策略

### 前进

- 手柄前进和前进转向交给 DWA 进行安全轨迹检查。
- DWA 根据统一融合点云评估候选轨迹，手柄转向意图用于同向候选筛选和评分。
- 几何二次轨迹选择当前关闭，不能额外把 DWA 选出的轨迹改成另一条几何候选。
- 前方硬停止不再由独立脚尖探针提前截断 DWA 的转弯机会；DWA 没有安全轨迹时才输出停车结果。

### 原地转弯和倒车限速圆

这是独立的限速模块，不参与前进 DWA 轨迹选择：

- 触发手柄区域包括完整倒车、纯左右横推，以及前进半区靠近左右横推的红色窄角区域。
- 纯正前进和红区外的前进转弯不触发。
- 障碍物进入原点为圆心、半径 `1.00 m` 的外圆后，控制器有效档位切为 1 档。
- 障碍物进入半径 `0.91 m` 的内圆后，线速度上限为 1 档速度的 `0.50`，当前 1 档峰值 `0.70 m/s`，即 `0.35 m/s`。
- 该限速圆修改的是线速度；纯原地旋转没有线速度可砍，不会因此自动限制角速度。
- 倒车旁路保留手柄方向，并通过统一发布链路应用有效档位和线速度限制。

### 侧边三角保护

- 只使用统一融合后的障碍点云。
- 只阻止朝障碍侧的转向，保留线速度，不再复用旧的整车 `(0,0)` 硬停止出口。
- 反方向转向不因另一侧命中而被禁止。
- 旧圆形保护和预测扫掠保护均关闭，避免在窄通道、S 弯和 L 弯中对 DWA 进行二次强制停车。

### 侧墙保持和脱困

- 侧墙保持用于小幅远离近侧墙的保护，不应替代 DWA 的前进避障。
- 统一脱困逻辑保留 IMU 角速度补偿、方向锁定、侧距安全检查和单次持续时间限制。
- 新手柄转向意图出现时，脱困方向不会继续锁存并长期接管。

## 构建

### Linux 宿主机

构建主程序和宿主机回归测试：

```bash
cmake -S c -B build-c \
  -DDWA_C_ENABLE_HOST_TOOLS=ON \
  -DCMAKE_C_STANDARD_LIBRARIES=-lm \
  -DCMAKE_CXX_STANDARD_LIBRARIES=-lm
cmake --build build-c
ctest --test-dir build-c --output-on-failure
```

Windows/MinGW 构建时通常不需要显式追加 `-lm`。

### ARM64 板端

在带有 `aarch64-linux-gnu-gcc` 和 `aarch64-linux-gnu-g++` 的 Linux 编译机上执行：

```bash
cmake -S c -B build-aarch64 \
  -DCMAKE_TOOLCHAIN_FILE=c/cmake/aarch64.cmake \
  -DDWA_C_ENABLE_HOST_TOOLS=OFF
cmake --build build-aarch64 --target robot_main -j2
```

主要产物：

- `build-aarch64/apps/robot_main`：整机单进程入口。
- `build-aarch64/apps/dwa_main`：规划/回放入口。
- `build-aarch64/apps/bridge_main`：独立电机桥入口。

## 板端启动

将以下文件放到同一项目目录：

```text
bin/robot_main
config/dwa_planner.yaml
config/lidar_filter.yaml
run_robot.sh
```

启动：

```bash
./run_robot.sh
```

脚本会把 stdout/stderr 同时输出到终端和 `logs/robot_*.log`。速度策略调试时可显式打开 INFO 诊断：

```bash
RM_DEBUG_SPEED=1 ./run_robot.sh
```

点云调试可额外设置：

```bash
RM_DEBUG_CLOUD=1 RM_UDP_POINT_DEBUG=1 ./run_robot.sh
```

替换板端程序前必须先停止正在运行的 `robot_main`，再使用临时文件传输并原子替换。实车测试前确认串口没有被其他进程占用，并准备可立即使用的物理急停。软件传感器联锁不能替代硬件急停。
