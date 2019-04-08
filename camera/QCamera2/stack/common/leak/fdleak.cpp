/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define LOG_TAG "QCameraLeak"
#include <dlfcn.h>
#include <stdio.h>
#include <ctype.h>
#include <unwind.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <cutils/properties.h>
// System dependencies
#include <utils/Log.h>
#include <iostream>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include "fdleak.h"
#include "memleak.h"
#include <errno.h>
#include "mm_camera_dbg.h"
#define MAX_BACKTRACE_DEPTH 15
#define MAGIC_ALLOC 0x7abc0fb5
#define MAGIC_FREE 0x087cbc8a

struct fdlist_t {
  uintptr_t bt[MAX_BACKTRACE_DEPTH];
  int bt_depth;
  int fd;
  struct fdlist_t *next;
  int allocated;
} __attribute__((packed));
typedef struct fdlist_t fdlist_t;

struct map_info_holder {
  char *name;
  struct map_info_holder* next;
  uintptr_t start;
  uintptr_t end;
};

struct stack_crawl_state_t {
  uintptr_t *addr;
  int skip;
  size_t stack_count;
  size_t max_depth;
};

pthread_mutex_t hal_debug_fdleak_mut = PTHREAD_MUTEX_INITIALIZER;
static unsigned int fdleak_count = 0;
static fdlist_t *head = NULL;

typedef int (*real_open_type)(const char *, int, ...);
typedef int (*real_open2_type)(const char *, int);
typedef int (*real_pipe_type)(int *);
typedef int (*real_socket_type)(int, int, int);
typedef void* (*real_mmap_type)(void*, size_t, int, int, int, off_t);
typedef int (*real_close_type)(int);

static real_open_type __real_open = NULL;
static real_open2_type __real_open2 = NULL;
static real_pipe_type __real_pipe= NULL;
static real_socket_type __real_socket= NULL;
static real_mmap_type __real_mmap= NULL;
static real_close_type __real_close= NULL;

static inline void add(int fd_value)
{
  fdlist_t *fdlist;
  fdlist = (fdlist_t *) malloc(sizeof(fdlist_t));
  if (!fdlist)
    return;
  fdlist->allocated = MAGIC_ALLOC;
  fdlist->bt_depth = 0;
  fdlist->fd = fd_value;

  pthread_mutex_lock(&hal_debug_fdleak_mut);
  fdlist->bt_depth = mmcamera_stacktrace(fdlist->bt, MAX_BACKTRACE_DEPTH);
  fdlist->next = head;
  head = fdlist;
  fdleak_count++;
  pthread_mutex_unlock(&hal_debug_fdleak_mut);
}

extern "C" void fdleak_dump_list()
{
  struct map_info_holder *p_map_info;
  struct fdlist_t *temp;
  temp = head;
  pthread_mutex_lock(&hal_debug_fdleak_mut);
  p_map_info = lib_map_create(getpid());
  while (temp != NULL) {
    ALOGE("leaked fd %d\n",temp->fd);
    print_backtrace(p_map_info, temp->bt, temp->bt_depth);
    temp = temp->next;
  }
  lib_map_destroy(p_map_info);
  pthread_mutex_unlock(&hal_debug_fdleak_mut);
  fdleak_count = 0;
}

int __open(const char* dev_name, int flags, ...)
{
  int fd_value;
  mode_t mode = 0;

  if ((flags & O_CREAT) != 0) {
    va_list args;
    va_start(args, flags);
    mode = static_cast<mode_t>(va_arg(args, int));
    va_end(args);
  }
  fd_value = open(dev_name, flags, mode);
  if (errno == EMFILE) {
    ALOGE("FATAL during open %s ",strerror(errno));
  }
    if (fd_value > 0) {
      add( fd_value);
      return fd_value;
    }
 return fd_value;
}

int __open2(const char* dev_name, int flags)
{
  int fd_value;
  fd_value = __open_2(dev_name, flags);
  if (errno == EMFILE) {
    ALOGE("FATAL during open %s ",strerror(errno));
  }
    if (fd_value > 0) {
      add( fd_value);
      return fd_value;
    }
 return fd_value;
}

int __pipe(int fd[])
{
  int ret_value;

  ret_value = pipe(fd);
  if (errno == EMFILE) {
    ALOGE("FATAL during pipe creation %s" ,strerror(errno));
  }
  if (ret_value >= 0) {
    add( fd[0]);
    add( fd[1]);
    return ret_value;
  }
  return ret_value;
}

int __socket(int domain, int type, int protocol)
{
  int ds_fd ;

  ds_fd = socket(domain, type, protocol);
  if (errno == EMFILE) {
    ALOGE("FATAL during socket create %s",strerror(errno));
  }
  if (ds_fd > 0) {
    add( ds_fd);
    return ds_fd;
  }
  return ds_fd;
}

void* __mmap(void* addr, size_t size, int prot, int flags, int fd, off_t offset)
{
  void* ret;

  ret = mmap(addr, size, prot, flags, fd, offset);
  if (errno == EMFILE) {
    ALOGE("FATAL during mmap %s",strerror(errno));
  }
  if (fd > 0 && ret != MAP_FAILED) {
    add( fd);
    return ret;
  }
  return ret;
}

void delete_node(int fd_value)
{
  static fdlist_t *temp,*prev;
  temp = head;
  prev = NULL;
  while (temp != NULL) {
    if (temp->fd != fd_value) {
      prev = temp;
      temp = temp->next;
    } else {
        fdleak_count--;
        if (temp == head) {
          head = temp->next;
          free(temp);
          return;
        }else {
           prev->next = temp->next;
          free(temp);
          return;
        }
    }
  }
}

static int __close(int fd_value)
{
  pthread_mutex_lock(&hal_debug_fdleak_mut);
  delete_node(fd_value);
  pthread_mutex_unlock(&hal_debug_fdleak_mut);
  return (close(fd_value));
}

void remFdCheck(int fd_value)
{
  pthread_mutex_lock(&hal_debug_fdleak_mut);
  delete_node(fd_value);
  pthread_mutex_unlock(&hal_debug_fdleak_mut);
}

extern "C" int __wrap_open(const char* dev_name, int flags, ...)
{
  mode_t mode = 0;

  if ((flags & O_CREAT) != 0) {
    va_list args;
    va_start(args, flags);
    mode = static_cast<mode_t>(va_arg(args, int));
    va_end(args);
  }
  return __real_open(dev_name, flags, mode);
}

extern "C" int  __wrap_pipe(int *fd)
{
  return __real_pipe(fd);
}

extern "C" int  __wrap_socket(int domain, int type, int protocol)
{
  return __real_socket(domain,type,protocol);
}

extern "C" void*  __wrap_mmap(void* addr, size_t size, int prot, int flags, int fd, off_t offset)
{
  return __real_mmap(addr, size, prot, flags, fd, offset);
}

extern "C" int  __wrap_close(int fd_value)
{
   return __real_close(fd_value);
}

extern "C" int __wrap___open_2(const char* dev_name, int flags)
{
  return __real_open2(dev_name, flags);
}

static __attribute__((constructor)) void init(void)
{
  __real_open = open;
  __real_open2 = __open_2;
  __real_pipe = pipe;
  __real_socket = socket;
  __real_mmap = mmap;
  __real_close = close;
}
void hal_debug_enable_fdleak_trace()
{
  __real_open = __open;
  __real_open2 = __open2;
  __real_pipe = __pipe;
  __real_socket = __socket;
  __real_mmap = __mmap;
  __real_close = __close;
}
void hal_debug_dump_fdleak_trace()
{
    if (fdleak_count) {
      ALOGE("FATAL fdleak found in camera hal %d",
         fdleak_count);
      fdleak_dump_list();
    }
}
static __attribute__((destructor)) void finish(void)
{
  ALOGD("fdleak lib deinit.\n");
  if (fdleak_count)
    fdleak_dump_list();
}

