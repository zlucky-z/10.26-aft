/*
 * 模块功能：DWA轨迹状态初始化和基于速度指令的轨迹生成。
 * 直译自 native/planning/dwa_trajectory.hpp。
 */
#ifndef DWA_TRAJECTORY_H
#define DWA_TRAJECTORY_H

#include "dwa_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* reset_trajectory(trajectory, command) 直译。 */
void dwa_reset_trajectory(dwa_trajectory_t *trajectory, const dwa_velocity_t *command);

/* generate_trajectory_into(command, config, trajectory) 直译。 */
void dwa_generate_trajectory_into(
  const dwa_velocity_t *command,
  const dwa_config_t *config,
  dwa_trajectory_t *trajectory);

#ifdef __cplusplus
}
#endif

#endif /* DWA_TRAJECTORY_H */
