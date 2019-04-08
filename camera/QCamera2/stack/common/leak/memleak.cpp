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

#include "mm_camera_dbg.h"
#include "memleak.h"


#define MAX_BACKTRACE_DEPTH 15
#define MAGIC_ALLOC 0x7abc0fb5
#define MAGIC_FREE 0x087cbc8a

struct hdr_t {
  int alloc_traced;
  unsigned int size;
  struct hdr_t *prev;
  struct hdr_t *next;
  uintptr_t bt[MAX_BACKTRACE_DEPTH];
  int bt_depth;
  int allocated;
} __attribute__((packed));
typedef struct hdr_t hdr_t;

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

static pthread_mutex_t memory_mutex = PTHREAD_MUTEX_INITIALIZER;
static unsigned int leaks_bytes = 0;
static hdr_t *last = NULL;

#define ADDITIONAL_SIZE sizeof(hdr_t)

typedef void *(*real_malloc_type)(size_t);
typedef void  (*real_free_type)(void *);
typedef void *(*real_realloc_type)(void *, size_t);
typedef void *(*real_calloc_type)(size_t, size_t);

static real_malloc_type __real_malloc = NULL;
static real_calloc_type __real_calloc = NULL;
static real_realloc_type __real_realloc = NULL;
static real_free_type __real_free = NULL;

static struct map_info_holder *lib_map_find_name(struct map_info_holder *mi,
  uintptr_t pc, uintptr_t* rel_pc) {
  for (; mi != NULL; mi = mi->next) {
    if ((pc >= mi->start) && (pc < mi->end)) {
      *rel_pc = pc - mi->start;
      return mi;
    }
  }
  *rel_pc = pc;
  return NULL;
}

void print_backtrace(struct map_info_holder *p_map_info, uintptr_t* frames, int frame_count)
{
  int i;
  struct map_info_holder *p_map_find;
  uintptr_t offset, rel_pc = 0;

  for (i = 0 ; i < frame_count; ++i) {
    const char* symbol = NULL;
    offset = 0;
    Dl_info info;
    const char* soname = NULL;
    if (dladdr((void*) frames[i], &info) != 0) {
      offset = (uintptr_t)(info.dli_saddr);
      symbol = info.dli_sname;
      rel_pc = offset;
      p_map_find = lib_map_find_name(p_map_info, frames[i], &rel_pc);
      soname = p_map_find ? p_map_find->name : info.dli_fname;
    }
    if (soname == NULL) {
      soname = "<unknown>";
    }
    ALOGI("#%02d  pc %08x  %s (%s) \n", i, (unsigned int)rel_pc, symbol, soname);
  }
}

static _Unwind_Reason_Code unwind_func_call(struct _Unwind_Context *context, void *arg)
{
  struct stack_crawl_state_t *p_state = (struct stack_crawl_state_t *)arg;

  uintptr_t unwind_ip = _Unwind_GetIP(context);

  if (p_state->skip && unwind_ip) {
    p_state->skip--;
    return _URC_NO_REASON;
  }

  if (unwind_ip) {
    short* ptr = (short *)(unwind_ip);
    if ((*(ptr-1) & 0xff80) == 0x4780) {
      unwind_ip -= 2;
    } else {
      unwind_ip -= 4;
    }
  }
  p_state->addr[p_state->stack_count++] = unwind_ip;
  return (p_state->stack_count >= p_state->max_depth) ? _URC_END_OF_STACK : _URC_NO_REASON;
}

int mmcamera_stacktrace(uintptr_t *addrs,
  size_t max_entries)
{
  struct stack_crawl_state_t state;

  state.max_depth = max_entries;
  state.skip = 2;
  state.addr = addrs;
  state.stack_count = 0;

  _Unwind_Backtrace(unwind_func_call, &state);

  return state.stack_count;
}


static inline void add(hdr_t *hdr, size_t size)
{
  hdr->allocated = MAGIC_ALLOC;
  hdr->size = size;
  hdr->bt_depth = 0;
  hdr->prev = 0;
  hdr->next = 0;

  hdr->bt_depth = mmcamera_stacktrace(hdr->bt, MAX_BACKTRACE_DEPTH);
  hdr->next = last;
  if (last) {
      last->prev = hdr;
  }
  last = hdr;
  leaks_bytes += hdr->size;
}

static struct map_info_holder *lib_map_parse_line(char* line)
{
  uintptr_t start;
  uintptr_t end;
  int name_pos;
  int ret;

  ret = sscanf(line, "%" PRIxPTR "-%" PRIxPTR " %*4s %*x %*x:%*x %*d%n",
    &start, &end, &name_pos);
  if (ret < 2) {
    return NULL;
  }

  while (isspace(line[name_pos])) {
    name_pos += 1;
  }
  const char* name = line + name_pos;
  size_t name_len = strlen(name);

  if (name_len && name[name_len - 1] == '\n') {
    name_len -= 1;
  }

  struct map_info_holder *p_map_info =
    (struct map_info_holder *)calloc(1, sizeof(*p_map_info));
  if (p_map_info) {
    p_map_info ->name = (char *)calloc(1, name_len + 1);
    p_map_info->start = start;
    p_map_info->end = end;
    if(p_map_info -> name){
      memcpy(p_map_info->name, name, name_len);
      p_map_info->name[name_len] = '\0';
    }
  }
  return p_map_info;
}


struct map_info_holder *lib_map_create(pid_t pid) {

  struct map_info_holder *map_list = NULL;
  struct map_info_holder *map_holder;
  char data[1024]; // Used to read lines as well as to construct the filename.
  snprintf(data, sizeof(data), "/proc/%d/maps", pid);
  FILE* fp = fopen(data, "r");
  if (fp != NULL) {
    while (fgets(data, sizeof(data), fp) != NULL) {
      map_holder = lib_map_parse_line(data);
      if (map_holder) {
        map_holder->next = map_list;
        map_list = map_holder;
      }
    }
    fclose(fp);
  }
  return map_list;
}

void lib_map_destroy(struct map_info_holder *map_hold)
{
  struct map_info_holder *del;
  while (map_hold != NULL) {
    del = map_hold;
    map_hold = map_hold->next;
    free(del->name);
    free(del);
  }
}


extern "C" void print_allocated_memory()
{
  hdr_t *del; int cnt, cnt_all = 0;
  struct map_info_holder *p_map_info;

  ALOGI("%d bytes non freed memory.\n", leaks_bytes);
  pthread_mutex_lock(&memory_mutex);

  p_map_info = lib_map_create(getpid());
  del = last;
  while(del) {
    cnt_all++;

   ALOGI("%d ALLOCATED MEMORY AT %p %d bytes (%d bytes REMAINING)\n",
        cnt_all, del + 1, del->size, leaks_bytes);


    print_backtrace(p_map_info, del->bt, del->bt_depth);

    del = del->next;
  }

  lib_map_destroy(p_map_info);

  pthread_mutex_unlock(&memory_mutex);
}

static inline void remove_hdr(hdr_t *hdr)
{
  if (hdr->prev)
      hdr->prev->next = hdr->next;
  else
      last = hdr->next;

  if (hdr->next)
      hdr->next->prev = hdr->prev;

  leaks_bytes -= hdr->size;
  hdr->allocated = MAGIC_FREE;
}

void * __malloc(size_t size)
{
  hdr_t *hdr;
  pthread_mutex_lock(&memory_mutex);
  hdr = (hdr_t *) malloc(size + ADDITIONAL_SIZE);
  if (hdr){
    add(hdr, size);
    pthread_mutex_unlock(&memory_mutex);
    return hdr + 1;
  }

  LOGI("not enough memory.\n");
  pthread_mutex_unlock(&memory_mutex);
  return NULL;
}

void * __calloc(size_t nmemb, size_t size)
{
  hdr_t *hdr = NULL;
  pthread_mutex_lock(&memory_mutex);
  hdr = (hdr_t *)calloc(1, (nmemb * size) + ADDITIONAL_SIZE);
  if (hdr) {
    add(hdr, (nmemb * size));
    pthread_mutex_unlock(&memory_mutex);
    return hdr + 1;
  }

  pthread_mutex_unlock(&memory_mutex);
  return NULL;

}

void * __realloc(void *ptr, size_t size)
{
  hdr_t *hdr = (hdr_t *)ptr;

  pthread_mutex_lock(&memory_mutex);
  if (hdr) {
    hdr--;
    if (hdr->allocated == MAGIC_ALLOC) {
      remove_hdr(hdr);
    } else {

    /* If is not our allocation just realloc */
    pthread_mutex_unlock(&memory_mutex);
    return realloc(ptr, size);
    }
  }

  hdr = (hdr_t *)realloc(hdr, size + ADDITIONAL_SIZE);
  if (hdr) {
    add(hdr, size);
    pthread_mutex_unlock(&memory_mutex);
    return hdr + 1;
  }

  pthread_mutex_unlock(&memory_mutex);
  return NULL;
}

void __free(void *ptr)
{
  if (ptr){
    hdr_t *hdr = (hdr_t *) ptr;
    hdr--;

    pthread_mutex_lock(&memory_mutex);
    if(hdr->allocated == MAGIC_ALLOC) {
      remove_hdr(hdr);
      free(hdr);
    }else {
      free(ptr);
    }
    pthread_mutex_unlock(&memory_mutex);
  }
}

extern "C" void * __wrap_malloc(size_t size)
{
  return __real_malloc(size);
}

extern "C" void * __wrap_calloc(size_t nmemb, size_t size)
{
  return __real_calloc(nmemb, size);
}

extern "C" void * __wrap_realloc(void *ptr, size_t size)
{
  return __real_realloc(ptr, size);
}

extern "C" void __wrap_free(void *ptr)
{
  __real_free(ptr);
}


static __attribute__((constructor)) void init(void)
{
  __real_malloc = malloc;
  __real_calloc = calloc;
  __real_realloc = realloc;
  __real_free = free;
}

void hal_debug_enable_memleak_trace()
{
  __real_malloc = __malloc;
  __real_calloc = __calloc;
  __real_realloc = __realloc;
  __real_free = __free;
}
void hal_debug_dump_memleak_trace()
{
  print_allocated_memory();
}
static __attribute__((destructor)) void finish(void)
{
  LOGI( "memleak lib deinit.\n");
  print_allocated_memory();
}
