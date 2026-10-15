/* hal_mutex 实现：Windows SRWLOCK / POSIX pthread_mutex。 */
#include "hal_mutex.h"

#include <stdlib.h>

#if defined(_WIN32)

#include <windows.h>

void hal_mutex_init(hal_mutex_t *mutex)
{
  SRWLOCK *lock = (SRWLOCK *)malloc(sizeof(SRWLOCK));
  InitializeSRWLock(lock);
  mutex->handle = lock;
}

void hal_mutex_destroy(hal_mutex_t *mutex)
{
  free(mutex->handle);
  mutex->handle = NULL;
}

void hal_mutex_lock(hal_mutex_t *mutex)
{
  AcquireSRWLockExclusive((SRWLOCK *)mutex->handle);
}

void hal_mutex_unlock(hal_mutex_t *mutex)
{
  ReleaseSRWLockExclusive((SRWLOCK *)mutex->handle);
}

#else /* POSIX */

#include <pthread.h>

void hal_mutex_init(hal_mutex_t *mutex)
{
  pthread_mutex_t *m = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
  pthread_mutex_init(m, NULL);
  mutex->handle = m;
}

void hal_mutex_destroy(hal_mutex_t *mutex)
{
  pthread_mutex_destroy((pthread_mutex_t *)mutex->handle);
  free(mutex->handle);
  mutex->handle = NULL;
}

void hal_mutex_lock(hal_mutex_t *mutex)
{
  pthread_mutex_lock((pthread_mutex_t *)mutex->handle);
}

void hal_mutex_unlock(hal_mutex_t *mutex)
{
  pthread_mutex_unlock((pthread_mutex_t *)mutex->handle);
}

#endif
