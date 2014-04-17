/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 Gomspace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk) 

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _CSP_PLATFORM_H_
#define _CSP_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Set OS */
#if defined(CSP_POSIX)
	// On Linux, we can use a recursive mutex to support recursive
	// critical sections.  [mconst, 2014/4/17]
	#include <pthread.h>
	#define CSP_BASE_TYPE int
	#define CSP_MAX_DELAY (UINT32_MAX)
	#define CSP_INFINITY (UINT32_MAX)
	#define CSP_DEFINE_CRITICAL(lock) static pthread_mutex_t lock
	#define CSP_INIT_CRITICAL(lock) ({ \
	    pthread_mutexattr_t attr; \
	    pthread_mutexattr_init(&attr); \
	    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP); \
	    pthread_mutex_init(&lock, &attr); \
	    CSP_ERR_NONE; })
	#define CSP_ENTER_CRITICAL(lock) pthread_mutex_lock(&lock)
	#define CSP_EXIT_CRITICAL(lock) pthread_mutex_unlock(&lock)
#elif defined(CSP_WINDOWS) || defined(CSP_MACOSX)
	// This is the original GomSpace implementation.  It deadlocks
	// whenever the CSP code enters a critical section recursively.
	// [mconst, 2014/4/17]
	#define CSP_BASE_TYPE int
	#define CSP_MAX_DELAY (UINT32_MAX)
	#define CSP_INFINITY (UINT32_MAX)
	#define CSP_DEFINE_CRITICAL(lock) static csp_bin_sem_handle_t lock
	#define CSP_INIT_CRITICAL(lock) ({(csp_bin_sem_create(&lock) == CSP_SEMAPHORE_OK) ? CSP_ERR_NONE : CSP_ERR_NOMEM;})
	#define CSP_ENTER_CRITICAL(lock) do { csp_bin_sem_wait(&lock, CSP_MAX_DELAY); } while(0)
	#define CSP_EXIT_CRITICAL(lock) do { csp_bin_sem_post(&lock); } while(0)
#elif defined(CSP_FREERTOS)
	#include <freertos/FreeRTOS.h>
	#define CSP_BASE_TYPE portBASE_TYPE
	#define CSP_MAX_DELAY portMAX_DELAY
	#define CSP_INFINITY portMAX_DELAY
	#define CSP_DEFINE_CRITICAL(lock)
	#define CSP_INIT_CRITICAL(lock) ({CSP_ERR_NONE;})
	#define CSP_ENTER_CRITICAL(lock) do { portENTER_CRITICAL(); } while (0)
	#define CSP_EXIT_CRITICAL(lock) do { portEXIT_CRITICAL(); } while (0)
#else
	#error "OS must be either CSP_POSIX, CSP_MACOSX, CSP_FREERTOS OR CSP_WINDOWS"
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // _CSP_PLATFORM_H_
