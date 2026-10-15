# 翻译记录（TRANSLATION_NOTES）

> P1+P2。逐行直译过程中所有"看不懂/有偏差风险/C++ 特性需要展开"的点都记在这里。
> 铁律：不做"优化"，只做等价变换。

## 全局

1. **原项目核心全部使用 double，不是 float。** 虽然任务描述提到"float 不隐式提升为 double"，但
   `native/` 的 dwa_types/dwa_geometry/planning 全部是 `double`。C 版保持 double，与 C++ 版
   完全同宽，parity 无 float 提升风险。方案文档中"float 保持 float"按实际情况理解为
   "数值类型宽度保持与 C++ 版一致"。
2. C++ 默认参数（如 `include_toe = true`、`epsilon = 1e-9`、`hit_unknown = nullptr`）→ C 显式
   传参/NULL；提供 `*_full` 便捷包装对应默认值调用点。
3. `std::min/std::max/clamp` → static inline 双参函数（min/max 语义与 std 一致，含 NaN 时行为
   同 libstdc++ 的实现方式）。
4. `constexpr double kDwaPi` → `#define DWA_PI`（同字面量）。

## common/dwa_types

- `DwaTrajectory::states` (vector) → 动态数组（count/capacity），push_back 语义保留；
  `capacity()` 跨周期复用语义保留（reserve 只扩不缩）。
- `DwaTrajectory::reject_reason` (std::string) → `const char*`。原代码只赋 5 个固定字面量
  （"grid_unknown"/"grid_collision"/"footprint_collision"/"no_valid_trajectory"/
  "invalid_forward_clearance_probe"）或 clear()，无拼接，静态字面量 + NULL(=空) 等价。
- `DwaOccupancyGrid::cells` → 指针+size+capacity；`valid()` 中 `cells.size() == width*height`
  用 cells_size 表达。
- `DwaGridBlockingIndex::blocking_x_by_row` (vector<vector<int>>) → 行数组（dwa_int_vec_t）。
  `resize(height)` 新行默认构造、`row.clear()` 保留容量、`reserve(width/8)` 均按原语义翻译。
  注意：`DwaGridBlockingIndex{}`（值重置）→ grid=NULL,row_count=0（容量保留）。
- `DwaObstacleSpatialIndex` 模板 `visit_square<Visitor>` → 函数指针 + void* 上下文
  （visitor 返回 true 即短路，与 lambda 返回 true 一致）。
- `offsets.assign(n,0)`/`resize`/`write_positions.assign(begin,end-1)` 的容量复用语义用
  assign_zero/resize helper 表达。
- C++ `build_*` 保留 vector capacity 的"复用已分配内存"注释语义已保留。

## common/dwa_geometry

- 纯函数直译，无特殊处理。`motion_step` 先更新 yaw 再用 next.yaw 算 x/y，顺序保持。

## planning

- **OpenMP**：`plan_dwa_into` 的 `#pragma omp parallel for schedule(static)` 候选循环，
  候选间无共享写、结果按索引写回，串行执行与之结果一致且确定。C 版为串行；
  parity_cpp 也未开 `-fopenmp`。（worker_count 字段保留但 C 版未用。）
- `DwaConfig::DwaAngularProfile` 及 `DwaPlanResult::angular_profiles /
  candidate_profile_indices / candidate_yawrates`：仅是 C++ 侧诊断/工作区字段，规划流程
  （plan_dwa_into）不读不写，未移植。`*_ms` 耗时字段保留恒 0。
- `dwa_scoring.hpp` 中 lambda `evaluate_obstacle` → `evaluate_obstacle_ctx_t` 结构体展开捕获
  （state/cos/sin/min_clearance*/useful_radius*/config/footprint_radius），逻辑逐行同序。
- `evaluate_forward_clearance_trajectory` 返回值语义 → `_into` 风格写入出参；invalid 分支
  对应 `DwaTrajectory invalid` 默认构造（reset_trajectory 复用缓冲）。
- `std::lower_bound(row, min_x)`（blocking index 行内有序 int）→ 行内跳过 < min_x、
  > max_x 即 break；行本身按 cell_x 升序 push，结果等价。
- `trajectory.states.back()`：generate_trajectory_into 保证 steps>=1，C 版同，
  `states[state_count-1]` 安全。

## config（P1）

- yaml 子集解析器：只支持 `节点: / ros__parameters: / key: value` 两层结构 + 行内
  `" #"` 注释剥离 + `[]` 浮点流式列表（gear_peak_speeds）。够解析现有 dwa_planner.yaml。
- 默认值来源（不修改任何 C++ 文件，逐条核对）：
  - `node_constructor.inc`：declare_parameter/declare_double/declare_int 的默认（多为回读
    成员当前值，即"成员默认"）+ 少数硬编码（grid_unknown_is_obstacle=true、
    planning_min_x=-0.15、controller 死区 89/111 等）。
  - `node_members.inc`：成员初始值（self_filter_body_front_x=0.48、
    side_hard_stop_circle_radius=0.44、stop_escape_trigger_sec=1.0、
    front_hard_stop_escape_yawrate=0.30、publish_debug_visualization=true 等）。
  - `DwaConfig` / `DynamicVelocityConfig` / `SideWallHoldConfig` /
    `DwaGeometricSelectorConfig` / `DwaCorridorGuideConfig` 的成员默认值。
- **max_velocity 特例**：constructor 先 `config_.max_velocity = declare(max_velocity,
  config_.max_velocity /*=DwaConfig 默认 1.00*/)`，再
  `dynamic_config_.forward_speed_limit = config_.max_velocity`，最后
  `config_.max_velocity = dynamic_config_.forward_speed_limit`。故"无 yaml 时 max_velocity
  默认 = 1.00"，yaml 给 0.70 时 = 0.70。已照此实现。
- **计算型默认**：`front_corridor_width`/`front_trigger_base_width` 默认 =
  `2*footprint_half_width`（=0.56），`front_trigger_start_x` 默认 =
  `0.5*(left_laser_x+right_laser_x)`（=0.48）。
- constructor 中对参数的 clamp/max（planning_worker_count≥1、footprint_self_filter_padding≥0、
  local_foot_mask_max_range≥0、forced_straight_* 三个、moving_prebrake_* 一组、
  moving_hall_speed_filter_alpha∈[0,1]、side_detection_angle∈[0,90]、
  manual_turn_forward_*、side_wall_hold_forward_margin∈[0,30]、hall_feedback_timeout≥0.01、
  marker_stride≥1、trajectory_score_top_n≥1、log_interval≥100）在加载后同样施加。
  注意 `moving_prebrake_y_down_rate_max = max(min, param)` 有依赖顺序，已按原顺序。
- deg→rad 参数（geometric selector / corridor guide 的角度项）按 constructor 原表达式
  `param*π/180` 转换后存储（rad）。
- constructor 中 env 覆盖（DWA_PREVIEW_SPEED 等）与 ROS-only 校验/throw 未移植（无 ROS 环境，
  参数校验失败场景由上游保证；P6 移植 runtime 时再决定是否补校验）。
- `gear_front_trigger_extra_angle_deg`、`gear_transition_*` 等 gear-only 参数原样保留字段，
  P4 fusion 阶段才消费。

## 疑点（留给后续阶段确认）

1. `node_parameters.inc` 实际是"动态参数更新回调"，真正的默认值声明在 `node_constructor.inc`；
   任务描述说"node_parameters.inc 里的默认值全部照搬"，按实际来源（constructor+members+
   各 Config 结构体）照搬。
2. `dwa_sampling.hpp.bak_*` / `dwa_scoring.hpp.bak_*` 备份文件未使用（当前 .hpp 为准）。
3. `laser_scan_fusion` 节点的 yaml 段未解析（P5 perception 阶段处理）。

## 构建说明（Windows 宿主机）

- MinGW Makefiles 生成器不支持含中文的源码路径（`F:\避障项目重构\...`），需把 `c/`、`native/`、`config/` 三个目录按原相对结构复制到 ASCII 路径后用 Ninja 构建：
  `cmake -S <copy>/c -B <copy>/c/build -G Ninja && cmake --build <copy>/c/build`
- 编译器：msys2 mingw-w64 gcc 15.2（宿主机已装，无需编译服务器）。
- 验收命令：
  - `tests/test_config.exe ..\..\..\config\dwa_planner.yaml`（P1 参数表）
  - `tests/test_planning.exe c.csv 1000` + `tools/parity/parity_cpp.exe cpp.csv 1000` + `tools/parity/parity_compare.exe c.csv cpp.csv 1e-5`（P2 parity）
- P1+P2 实测结果：test_config ALL PASS；parity 1000 场景 61763 行，mismatches=0，max_err=0（bit-exact）。

## 2026-09-24 P3/P4 状态序列 parity 补全记录

新增 `tests/test_p3p4.c`（C 侧）与 `tools/parity/parity_p3p4_cpp.cpp`（C++ 基准侧）：
同一 LCG 逐 tick 生成状态序列（48/500 场景 × 96 tick），覆盖 fusion 状态机（4 种
时序模式：随机游走/接近减速/恢复/静止起步）、side_wall_hold（含 INF 侧距）、
corridor_guide（真实双墙走廊 + 噪声，24 tick 周期切换墙存在/消失以触发丢失保持）、
dwa_geometric_selector（链 P2 planning，cluster/polar 两种扫描点模式）、
safety 全套纯函数（含 NaN/INF 侧距桶）。两侧 rng 调用顺序必须严格一致
（修改任一侧必须同步另一侧）。

### 本次发现并修复的直译笔误（corridor_guide.c，此前 ctest 只编译未运行，未暴露）

1. **cg_detect 内栈上 candidate 未初始化指针**：`dwa_corridor_guidance_reset()`
   只清标量不动数组指针（对应 C++ vector 容量保留语义），栈上局部
   `dwa_corridor_guidance_t candidate` 必须先手动置空 slices/candidate_gates
   六个成员再 reset（对应 C++ 默认构造）。已修。
2. **cg_gates_push_back(&result, ...) 笔误**：result 已是 `guidance*`，多打了 `&`
   传入二级指针导致 realloc 垃圾指针段错误。已修（line ~927）。

### dwa_geometric_selector.c 排序算法替换（重要直译决定）

原版 `std::sort(ranked, comp)` 的比较器带 `heading_tie_tolerance`（`|Δerror| > tol`
才分高下），**近似相等不可传递，非严格弱序**。此条件下排序结果依赖具体排序算法
实现：C 版最初直译为 merge sort，与 libstdc++ `std::sort`（introsort）对同一输入
产生不同顺序 → selected_index 分歧（97/4608 tick）。

处理：按铁律不改比较器，改为**逐行复刻 libstdc++（gcc 11.2）
`std::__sort` 全套算法**（gs_std_sort）：`__introsort_loop`（_S_threshold=16，
depth_limit=2*lg(n)，超深转 `__partial_sort(first,last,last)`=make_heap+sort_heap）
+ `__final_insertion_sort`（前 16 插入排序 + unguarded 线性插入）+
`__unguarded_partition`（**注意从 first+1 开始扫描**，pivot 在 first）+
`__move_median_to_first` + `__adjust_heap`/`__push_heap`/`__pop_heap`。
替换后与 C++ 基线 bit-exact。

⚠️ 后续注意：若板端/其他编译机的 libstdc++ 版本不同，C++ 原版 std::sort 在
非传递比较器下的行为本身可能不同——这是原版固有不确定性，非 C 版缺陷。

### parity_compare 增强

`split()` 字段上限 32 → 64（P3/P4 的 g 行有 56 字段，超限字段此前未被比较）。

### P3/P4 parity 实测结果（2026-09-24）

- 500 场景：384631 行，mismatches=0，max_err=0（bit-exact）
- 48 场景冒烟：36916 行，0 mismatch
- P2 回归：61763 行，0 mismatch（无回归）
- ctest 3/3 全过（config / planning / p3p4）

## 2026-09-24 P4.5 全库 double→float 化记录

### 转换方法（c/tools/floatify_p45.py，词法级处理，跳过注释/字符串）
- double→float；数学函数 f 后缀（atan2/fabs/floor/hypot/fmax/ceil/sqrt/sin/cos/tan → *f）
- DBL_EPSILON→FLT_EPSILON（dwa_scoring.c 归一化守卫语义变化：span∈(2.2e-16,1.19e-7]
  现在提前返回 0 不再放大，实测无影响）
- 浮点字面量全部 f 后缀（杜绝隐式提升）；(double) 强转→(float)；共 29 文件 1744 行
- 例外：config/dwa_yaml.{c,h} 保持 double——YAML 文本解析是 I/O 边界，解析保持 double
  精度、落盘 float 字段时才舍入（dwa_params.c 的 gear_peak_speeds 数组经 double 缓冲中转）
- DWA_PI 变为 float π；harness（c/tests/）一律用本地 double T_PI/TEST_PI 生成输入

### Parity 输入量化方案（关键决定）
C 侧库全 float 后，必须保证两侧输入 bit 一致，对拍误差才能归因于"float 计算"本身：
- C 侧：harness 场景值（double）赋入 float 字段时隐式舍入
- C++ 基准侧：q()（double→float→double）显式走同一路径；qz() 对各 config 默认值全字段
  量化（DwaConfig/DynamicVelocity/SideWallHold/CorridorGuide/GeometricSelector/
  FrontSafety/TurnClassification/SideSafety + Footprint/PathReference + gear_peak_speeds
  vector 逐元素）
- 依赖链注意：side_wall 的 release = hold + 0.05 必须先量化 hold 再算 release
- 一次性补丁脚本 C:/tmp_dwa/apply_p45_harness_edits.py（99 条精确替换全部断言命中）
- 双精度备份：C:/tmp_dwa/src_double_backup_20260924/ 与 harness_backup_20260924\

### test_config 适配
expect_double 改为与 (float)expected 比较（容差 1e-6≈10 ulp）；期望表达式改 TEST_PI。

### parity_compare 增强
末尾输出 per-(行类型,字段) 最大绝对误差表——P4.5 逐字段审查的数据源。

### P4.5 对拍结果（容差 1e-3，输入量化后）
P2（1000 场景，61763 行）：21 行超差（0.03%），全部是候选行 path_cost（max 0.0129）与
total_cost（=0.4×path_cost）。机理：path_cost 归一化 (v−min)/(max−min)，场景内候选
path_cost 跨度 span 小（~1e-4）时 ~2e-6 的 float 原始噪声被放大两个数量级。
选中轨迹（s 行）全部字段 ≤4.3e-5，**选择 0 翻转**，valid/reject_reason 100% 一致。

P3/P4（500 场景，384631 行）：154 行超差（0.04%）
- corridor(c/cs)/side_wall(w)/safety(sf/se/sc/sg)：**0 超差**，连续字段 max err ~3e-7
- fusion(f)：151 行（56/500 场景）。根因①（主）：closing_rate 更新的 dt>0.50 守卫恰好
  压在 harness DT={0.02,0.05,0.10,0.50} 的 0.50 刀锋上——float 时间戳差 0.50000001 被
  float 减法舍回恰好 0.5f（C 走 update），double 减法保留 0.50000001（C++ 走 RESET 置 0）。
  已用 CSV 实测验证（s=6 t=3: dt_double=0.50000001490116119 vs dt_float=0.5）。
  根因②：降档阈值 tie（s=30: C 早 1 tick 降档，gear_trigger 边界）。所有翻转 ≤8 tick
  重新收敛，无发散累积。
- geometric(g)：3/15165 行。selected_index/eligible_count 在 clearance/heading tie 边界
  翻转；1 例 cluster 点排序 tie 致 raw_heading 差 0.003；st->valid 从未翻转。

### 结论
全部超差可归因为阈值刀锋/tie 翻转（离散决策边界与 float 噪声同量级）或归一化放大，
无系统性偏差、无状态机发散。这些是 float 化的固有行为，非直译缺陷。板端影子模式（P8）
时重点复核 fusion 档位切换与 closing_rate 边界行为。

## 2026-09-24 P5 perception 记录（附策略调整）

新增 c/src/perception/ 三模块 + c/src/config/perception_params.{h,c}：
- laser_filter_core.{h,c}：FilterConfig/Stats/valid_range/filter_ranges/sector_minimums。
  原版统计累加器（sum/squared_sum/mean/variance/stddev）为 double，按全库 float
  硬要求转 float；实测 48 场景无离散翻转
- laser_scan_fusion.{h,c}：只移植纯计算核心（Mount/valid_range/append_scan 坐标
  变换）；ROS 节点外壳（话题/锁/PointCloud2 序列化）不属本层。fusion 节点自己的
  Mount 默认 x/y=0.50/±0.27，与 DWA 节点 LaserMount 默认 0.48/±0.25 不同，勿混
- local_occupancy_grid.{h,c}：reset/mark_free/mark_occupied/raytrace/inflate/
  clear_footprint；reset 内部负责 cells 缓冲分配（对应 C++ cells.assign）
- perception_params：laser_scan_fusion 段（dwa_planner.yaml）+ lidar_filter.yaml
  左右两段；含构造函数语义（window_size 偶数+1、sector_count<0 归 0、deg→rad）
  注：deg→rad 用 float π（DWA_PI），原版 preprocessor 用 double kPi——真实 yaml
  值（±180 等）下差异不可见，P6 组装时复查

parity（tests/test_p5.c + tools/parity/parity_p5_cpp.cpp）：覆盖 filter/fusion/
grid 全操作 + planning grid/blocking_index 路径（补 P3 遗留缺口，footprint_hits_grid
含/不含 blocking index、plan_dwa_into 带 grid 两种工作区模式）。结果：48 场景
0 mismatch，max_err=2.3e-4（tol 1e-3），ctest 4/4。

### 本次犯的错（第三次同类，已做结构性修复）
1. fuh 行格式串 19 个 %.17g vs 18 个实参（%zu 读栈垃圾）；C++ 侧另有首行排列错位。
   P3 曾犯同类。修复：根 CMakeLists 加 **-Werror=format**，全部目标编译期强制
2. harness 输入两段舍入：C 侧 `0.05f + (float)(rng*0.15)`（float 算术）vs C++ 侧
   `q(0.05 + rng*0.15)`（double 单次舍入）→ 输入差 1 ulp，违反 P4.5 输入 bit 一致
   原则。规则：**harness 输入生成一律 double RHS 整体表达式、赋值时单次隐式舍入**，
   与 C++ 侧 q() 完全同构；禁止 float 字面量参与中间运算
3. laser_filter_core.c 漏 #include <stdlib.h>（calloc/free 隐式声明）

### 策略调整（用户 09-24 指示）
架构优先，实测兜底：合成场景对拍只做冒烟级（不再跑 500 场景统计），优先完成
P6/P7/apps 完整架构，真实验证放最后（真实数据回放 + P8 影子模式），按实测修模块。

## 2026-09-24 P6 runtime+hal 完成记录

### 新增文件（node 层三文件 + 内部接口 + 入口）
- c/src/runtime/dwa_node_internal.h：tick/safety/perception 三文件间的私有接口
  （dnp_/dns_ 前缀），不对外暴露
- c/src/runtime/dwa_node_tick.c：node_tick.inc 655 行直译
- c/src/runtime/dwa_node_safety.c：node_safety_control.inc + node_diagnostics.inc 直译
- c/src/runtime/dwa_node_perception.c：node_perception_grid.inc + node_perception_scan.inc
  直译（含 dwa_node_decode_base_cloud 输入侧工具，P8 回放用）
- c/apps/dwa_main.c：入口（yaml 加载 + 周期主循环 + cmd_sink 打印输出面）

### 关键直译决定
1. **RAII → goto**：C++ tick 的 scope_exit guard 与 TickTimingScope 用显式
   enter/leave + `goto tick_exit` 统一收尾（单线程 tick，语义等价）
2. **ROS publish → cmd_sink**：publish_regular_command 落点 = 回调
   dwa_node_set_cmd_sink + node->last_published_cmd 记录；hall 控制帧为 8 元素
   float 数组（Float32MultiArray 数据面）。P7 电机桥/P8 影子在此接入
3. **RViz 发布 = no-op 桩**：publish_footprint/obstacle_points/occupancy_grid/
   fused_occupied_cells/geometric_selection/candidates/selected 及
   publish_dynamic_velocity_diagnostics 全部 no-op；kTrajectoryPublish 计时段不记账
4. **mutable 成员**：body_masked_points_this_tick_ 在 const 方法内递增 → C 侧
   `++*(size_t *)&node->...`（去 const，保持 C++ mutable 语义）
5. **vector 拷贝语义**：tick 里 fused_cloud_points 的 vector 拷贝 → 直接引用缓冲
   （scan_mutex 保护下读，数据逐位一致；生命周期差别已注释）
6. **log_trajectory_scores**：top-N 用确定性最小值扫描替代 std::sort（仅日志，
   默认关闭，不影响控制）；reject 计数用小数组替代 std::map
7. **geometric_selection 生命周期**：声明即 init（对应 C++ 默认构造），
   tick_exit 统一 destroy——首个 goto 前已初始化，无未定义行为
8. **dwa_geometric_selector_reset_geometry_target**：原 static 函数加公共包装
   （tick 无有效轨迹时调用），无行为改动

### 原版固有日志 bug（bug-for-bug 保留，勿"修复"）
node_tick 的主日志行 "dwa tick=..." 中 front_clear 段有 **8 个 float 实参对
7 个 %.17g 说明符**（nearest_front,x,y, gear_front, radar_dist,radar.x,radar.y,
front_radar_dist——格式串少了 gear_front 的槽位）。C++ 原版即如此：Linux/SysV
ABI 下 float 槽位整体后移一位，Windows/MS ABI 下整行后移一位（int 槽也受影响，
故 MinGW 构建里 raw_gear/intent 显示错位更明显）。仅影响日志显示，控制流无关。
直译保持格式串与实参表逐字一致。若日后要修，须两侧（native/ 和 c/）一起改。

### 验证
- 构建零警告（-Werror=format 生效）；ctest 4/4
- dwa_main 空输入 3-tick：干净跑通（cloud stale → 回中旁路 → 停车输出）
- 合成走廊 50-tick 深冒烟（临时 smoke_tick.c，未入库）：前方墙 1.6m/双侧墙
  0.75m + 前进意图 0.5 + 3 档反馈；输出 last(v=0.500,w=0.000) 正确
  （1.6m > moving_dwa_trigger 0.7 → 直控分支），fusion 档位闭环（raw 3 → target 1
  过渡中）、侧距/预减速/栅格全部工作，零崩溃
- P2/P3P4/P5 parity 回归（tol 1e-3）：与 P4.5 验收结论同族（刀锋/tie 类），
  库代码本轮无行为改动

### 涉及修改的既有文件
- dwa_node.{h,c}：+输出 sink/档位日志状态/诊断开关成员；const mutex 去.const
  转换（= C++ mutable）；补 dwa_geometry.h include
- dwa_geometric_selector.{h,c}：+reset_geometry_target 公共包装
- 根 CMakeLists：+add_subdirectory(apps)；runtime/CMakeLists：+三个新源文件

---

## 2026-09-24 P7：电机桥移植（motor_bridge_cpp → c/src/motor + apps/bridge_main.c）

### 范围与文件清单
- `c/src/motor/motor_protocol.{h,c}`（mp_ 前缀）：协议常量（MP_SOP 0xA5 等）、
  mp_crc8、**mp_py_round（Python round 语义 = 银行家舍入 round-half-to-even）**、
  mp_clamp_coord/mp_clamp_double、坐标换算、mp_pack_gear_horn、
  mp_build_frame（返回 mp_frame_t{uint8_t bytes[16]; size_t len}）、mp_frame_hex/Equal
- `c/src/motor/motor_control_profiles.{h,c}`（mc_ 前缀）：4 判定函数 + 4 控制器
  （mc_hall_speed_controller_t 构造参数 gain/max_down_rate/max_up_rate/
  deadband/minimum_y + 全部状态字段）。注意 `maximum_y = max(kCenter,
  min(200, raw_cap_y))` 的嵌套展开
- `c/src/motor/stop_escape_compensator.{h,c}`（se_ 前缀）：init 只收 5 参数
  （tie_margin/forward_min_x/forward_max_x/max_lateral 用 C++ 默认实参
  0.03/-0.20/1.20/1.20）；decision reason 为字面量
- `c/src/motor/motor_serial.{h,c}`（ms_ 前缀）：POSIX termios/flock 串口，
  `#if !defined(_WIN32)` 守卫（Windows 上 open 恒失败，仅 dry_run 可用）；
  ms_sample_t（kind 枚举、frame 字节复制、time = ms_now_monotonic() =
  hal 原始 CLOCK_MONOTONIC 值）；rx 缓冲 64KB memmove 消费；
  ms_parse_rx_buffer 译 drive/status/2.3/2.1/2.0/legacy_feedback 全分支
- `c/src/motor/motor_bridge_config.{h,c}`：mb_config_t 全字段（topic 字符串、
  protocol 枚举 + 名字）；mb_config_defaults；mb_config_parse（argv 逐项直译，
  stoi/stod 语义用 strtol/strtod + consumed 校验复刻）；校验块错误消息逐字对齐；
  gear_peak_speeds 用 total_count 保 >5 校验（注意：**条数校验必须先于正性
  校验**，否则越界读）
- `c/src/motor/motor_bridge.{h,c}`（mb_ 前缀，~1800 行核心）+ `c/apps/bridge_main.c`
- `c/src/drivers/cspc_lidar_adapter.{h,cpp}`：C 接口（cspc_laser_scan_t + 回调）
  + extern "C" 包装 node_start()/data_handling()。

### 关键决策：电机桥全程 double，不做 float 化
P4.5 的 float 化范围是 DWA 规划库（c/src 的 perception/planning/guidance/
safety/fusion/runtime），**桥不在内**。理由：P7 验收是串口帧字节级一致，
mp_py_round（银行家舍入）的 0.5 刀锋对精度敏感——double→float 会翻转帧字节
（round(0.5)=0 vs round(0.5+ε)=1 类翻转）。桥对拍为 bit-exact 验收。

### 双时钟基 bug-for-bug 保留
C++ 原版混用两个时钟：`steady_seconds()`（steady_clock，**进程起算**）与
`SerialWriter::now_monotonic()`（CLOCK_MONOTONIC 原始，**开机起算**）。
sample.time（后者，大数）与 now（前者，小数）比较 → `wheel_feedback_is_fresh`
实际恒真。C 版复刻：`mb_now()`（默认 = 进程起算静态起点）+ `ms_now_monotonic()`
（hal 原始值）。**勿"修复"**——修复会改变超时判定行为，破坏等价。

### 原版死代码（直译保留，不接线）
- `sync_hall_speed_controller_to_output`：定义但全工程无调用点（grep 验证）
- `hard_stop_active_`：从未置 true → `hard_stop_latched` 状态不可达

### ros_mock 对拍架构（tools/parity）
- **节点副本生成**：CMake configure 阶段 file(READ)+string(REPLACE)+file(WRITE)
  把 motor_xy_bridge 节点源复制到 build 目录，仅将 `#include "motor_serial.hpp"`
  一行替换为记录型 mock —— 绕过"quoted include 先搜源文件目录"，保持原文件只读
- **include 劫持**：`-I` 顺序使 `<chrono>`/`<thread>`/`<rclcpp/...>`/消息头命中
  c/tools/parity/ros_mock/ 下 10 个假头；mock chrono 提供
  `ros_mock_time::g_now` 可注入时钟；mock sleep_for 为 no-op
- **mock SerialWriter**：write() 记录到全局 g_written_frames（对拍观测面）
- **`#define private public` 预包含策略**：先以正常模式包含节点会拉入的全部
  标准头（含 mock），define 之后节点内 `#include <...>` 命中 include guard
  变 no-op —— 直接 define 会破坏 libstdc++ 内部访问控制（`<sstream>` 的
  __xfer_bufptrs 编译失败）
- **mb_injection_t**：sink（controller_cmd/feedback/wheel_yaw/frame）×4 + 时钟
  + sleep 集中为一个子结构；`mb_bridge_init` 内 save→memset→restore，使对拍
  能在 init 前设置注入（构造期启动停止帧串也要走注入时钟）；调用方须先
  零初始化 bridge

### C 侧直译要点
- 状态字符串：`char state[MB_STATE_MAX=48]` + strcmp/`state_append_stop` 拼接，
  保留 C++ std::string 的 "_stop" 后缀语义
- 协议字符串→枚举：mp_protocol_t（20/21/23/GEAR_HORN），config 解析时映射；
  Sample.kind → ms_sample_kind_t 枚举；Sample.protocol 保留 const char* 字面量
- 逃逸点云：realloc 增长的静态缓冲
- mb_read_float：大端 FLOAT32/64 译码
- mb_on_timer：顶部捕获一次 now 贯穿使用（对应 C++ 局部 const）

### cspc_lidar SDK：唯一 C++ 例外
SDK core（~2500 行）保持 C++ 编译，cspc_lidar_adapter.cpp 以 extern "C" 包装；
仅 POSIX 构建（serial_port 依赖 termios），CMake `if(NOT WIN32)`。

### 对拍结果（P7-4 验收）
- harness：parity_p7_cpp.cpp（C++ 基准，~470 行）vs test_p7.c（C 侧，~500 行），
  同一 LCG（seed=(s+1)×0x9E3779B97F4A7C15），100 tick × 7 类事件注入，
  CSV 47 字段/行
- **300 场景 30301 行 0 mismatch，max_err=0，bit-exact（tol 1e-9）**
- 状态覆盖（PARITY_LOG=1，grep 三段式 state/reason/detail）：48 种路径全命中，
  含扩展场景新增的 controller_reverse_passthrough(13)/
  controller_reverse_safety_steer(438)/
  hall_moving_hold_emergency_stop(70+43+54)/unchanged_idle(~2236，8 个状态维度)
- ctest **5/5**（config/planning/p3p4/p5/p7）；dwa_main/bridge_main 冒烟通过
  （bridge_main dry-run：启动帧串 A5 01 81 02 EC 64 64 + timeout_stop 周期行为）

### 本轮环境坑（新增）
- **Ninja 扁平布局下 add_test 的 cwd 是 build/ 根**（不是
  build/c/tests/——该目录根本不存在）：test_config.c 默认相对路径
  `../../../config/dwa_planner.yaml` 逃到 C:/config/。修复：add_test 显式传
  `${CMAKE_SOURCE_DIR}/../config/dwa_planner.yaml`
- robocopy 经 PowerShell 对中文路径静默失败 → 改用 bash cp 逐文件同步

## 2026-09-24 P9：单进程 robot_main（SDK 双实例 + 生产参数复刻 + 板端修复）

### 架构
- `c/apps/robot_main.c`（~830 行）：单进程收编生产 10 进程拓扑
  （cspc×2 → 滤波×2 → 融合 → DWA → 电机桥；TF 节点仅可视化，省略）
- 线程模型对齐 ROS 单线程 executor：2×雷达线程（`cspc_pair_poll`
  阻塞整圈→滤波→触发融合）、桥线程 20ms（事件队列排空后 `mb_on_timer`，
  全部 `mb_on_*` 串行在桥线程）、主线程 100ms（`dwa_node_tick`+打印）
- 数据接线与 ROS topic 一一对应（见 robot_main.c 头注释）；打印格式
  对齐 9.20 基线包（正前方距离/[路径规划]/[通道]/[循环计时]）

### cspc SDK 双实例（cspc_lidar_pair.cpp）
- SDK 以全局单例组织，单进程双雷达需两份独立副本：`namespace cspc_left`
  与 `namespace cspc_right` 各包含全套 SDK 头+源
- 三个编译要点：
  1. 系统头在命名空间外全局预包含（否则 `#include <cstring>` 在
     namespace 内展开导致 `::memchr` 连锁报错）
  2. SDK 头必须随命名空间包含（全局包含会使自由函数落全局，
     命名空间内调用经 ADL 命中两份候选 → ambiguous）
  3. include guard 宏是编译单元全局的：第二命名空间前须 `#undef` 全部
     9 个 SDK guard；`locker.h` 用 `#pragma once` 且全为无状态工具类，
     全局包含一次两侧共享
- `CSPC_PAIR_SIDE_IMPL` 宏生成 per-side 函数，extern "C" 分派

### 本轮板端修复（三处，均已验证）
1. **mb_bridge_init 别名缺陷（ENOENT 根因）**：`memset(b,0,sizeof(*b))`
   会清零 `b->config`；robot_main 传 `&g_bridge.config`（指向 b 内部），
   之后所有 `config->` 读取拿到零值——port 变空串，`open("")` 恒 ENOENT，
   表现为"配置打印正确但串口打不开"。修复：函数入口做本地快照
   `config_saved = *config_in`，全部改从快照读取。bridge_main 用独立
   局部 config 故 P7 对拍未暴露
2. **port_lock_acquire 锁路径偏差**：原实现对整串做 '/'→'_' 替换得到
   相对路径 `_tmp_hu_m40_motor_ttyS6.lock`（cwd 只读时锁创建失败、
   不同 cwd 进程失去互斥）。Python 的 replace 只作用于 basename（no-op），
   修正为固定 `/tmp/hu_m40_motor_<base>.lock`
3. **退出路径 fortify 中止**：SDK detached 读线程设计上无法停止，正常
   return 触发 C++ 静态析构竞态 → `*** buffer overflow detected ***`。
   修复：robot_main 线程启动后的致命路径与正常收尾统一 `rm_exit()`
   （fflush + `_exit`，跳过析构）

### 板端验收（25s 冒烟，真机双雷达 + ttyS4/ttyS5，电机 ttyS6）
- 电机桥打开 /dev/ttyS6 成功（protocol 2.3、interval 20ms）
- 主循环 182 圈，[路径规划]/[通道]/[循环计时] 全程输出，格式与基线一致
- 循环耗时 ~1.25ms（100ms 周期余量充足）；SIGTERM 干净退出、无 fortify 中止
- ctest 5/5 回归通过；交叉产物 SHA256
  `4939eeec69dad5022625cba2ff079648c517c15eff6c1e4fe57c5889a1665703`

### 遗留观察（非阻塞）
- 正前方距离在 ~2.23m 与 ~0.015m 间交替：疑似某一侧雷达在车头正前
  梯形探测区内有极近点（待用户实测场景确认是否为环境真实遮挡）
- fbL/fbR=0：25s 内底板无 0x88 反馈帧（无手柄操作）；rx-debug 为 INFO
  级被 hal_log WARN 阈值压掉，用户实测手柄操作时可验证反馈链路

## 2026-09-24 P9 排障：自遮挡幻影点 + 单进程顿挫

### 幻影点定位（用户实测日志 + 诊断打印 + 生产实测三方证据）
- 实测 fwd 几乎全程 ~0.0155m；v 跳变 66.5/41.7/0/负值（逃逸机动组合）
- 幻影点来源：双雷达互看+机身/脚踏结构反射（raw≈0.05m 光束，右雷达
  lidar 角 28°/68-71° 等固定簇），被滤波"近距钳位保留"语义钳到
  range_min=0.06，融合变换后落 (0.49~0.52, ±0.22~0.26)
- **生产点云实测同样含这些点**（板上 rclpy 诊断脚本：生产融合 546 点
  vs 单进程 533~564，自遮挡光束同索引同位置）——生产靠 DWA footprint
  排除（P8 实录 near-obstacle 0.729m 未受其影响）；基线的等效机制是
  lidar_funsion.c 盲区裁剪（x<90cm 且 |y|<21cm）+ DWA 脚踏板段
  footprint（57~75cm 半宽 21cm）
- 注意：现网生产 cspc_lidar_cpp（9.23 构建）的 SDK 已删 0.15m 近距
  强度门限（third_party 副本注释可证），生产自身现在也携带自遮点

### 修复（本包 5 项）
1. rm_fuse 融合输出一次性剔除自身包络点（x<0.56 && |y|<0.27——车体
   主段物理占位，外部障碍不可能存在），DWA/逃逸/正前方距离共用干净
   点云；这是对基线盲区裁剪意图的等效实现，与生产输入的差异仅限
   footprint 本来就忽略的自身点
2. SDK printf 噪声静音：cspc_lidar_pair.cpp 在命名空间展开前
   `#define printf → no-op`（full----- 转速心跳/version 枚举打印），
   消除与主打印的 stdio 锁争用并保证日志与基线可对照；命名空间
   结束后 #undef 还原
3. 雷达线程分核：左 CPU2 / 右 CPU3（CPU0/1 专供主循环+桥线程）
4. 排障仪表（RM_DEBUG_CLOUD 门控）：主循环 stall 自动定位器
   （loop>20ms 时输出 tick_wall/tick_cpu/print 三段分解——tick_cpu
   ≈tick_wall 为计算爆炸，tick_cpu≪tick_wall 为调度/锁等待）；
   桥线程 1Hz 健康打印（rx bytes/frames/bad_crc/dropped + wheel/
   ctrl/gear/escape 状态）
5. rm_front_gap 恢复纯基线梯形几何（点云已干净）

### 静态验收
fwd 稳定 1.75~1.77m（不再 0.015/2.24 交替）；融合 522~525 点；
full----- 归零；RX 849 帧 0 bad_crc 0 dropped、ctrl=(100,100) gear=5；
无 stall。行驶顿挫验证待用户实测（stall 定位器随日志自动归因）。
