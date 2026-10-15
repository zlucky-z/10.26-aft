/*
 * 模块功能：互斥锁抽象（输入线程与 tick 主循环的数据面互斥）。
 * 对应 C++ 侧 std::mutex + std::lock_guard（scan_mutex_ 等）。
 * 可移植性：Windows SRWLOCK / POSIX pthread_mutex。
 * 语义与 std::mutex 一致：非递归、旁路优先。
 */
#ifndef DWA_HAL_MUTEX_H
#define DWA_HAL_MUTEX_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
#if defined(_WIN32)
  void *handle; /* SRWLOCK 以 InitializeSRWLock 零初始化，此处存指针便于演进 */
#else
  void *handle; /* pthread_mutex_t* */
#endif
} hal_mutex_t;

/* 初始化/销毁。hal_mutex_init 后 handle 非空。 */
void hal_mutex_init(hal_mutex_t *mutex);
void hal_mutex_destroy(hal_mutex_t *mutex);

/* 加锁/解锁（lock_guard 语义：成对使用）。 */
void hal_mutex_lock(hal_mutex_t *mutex);
void hal_mutex_unlock(hal_mutex_t *mutex);

#ifdef __cplusplus
}
#endif

#endif /* DWA_HAL_MUTEX_H */
