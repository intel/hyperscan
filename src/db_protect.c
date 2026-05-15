/*
Copyright (C) 2026 Intel Corporation
This software and the related documents are Intel copyrighted materials,
and your use of them is governed by the express license under which they were
provided to you ("License"). Unless the License provides otherwise,
you may not use, modify, copy, publish, distribute, disclose or transmit this
software or the related documents without Intel's prior written permission.
This software and the related documents are provided as is, with no express or
implied warranties, other than those that are expressly stated in the License.
*/

/**
 * \file
 * \brief Database memory protection via mmap/mprotect.
 *
 * When the default allocator is active, databases are allocated with
 * mmap (MAP_ANONYMOUS | MAP_PRIVATE) which guarantees page-aligned memory.
 * After HMAC computation the pages are marked PROT_READ so any
 * post-compilation tampering triggers SIGSEGV.
 *
 * When a custom database allocator has been set via
 * hs_set_database_allocator(), allocations are routed through the
 * user-supplied callbacks and mprotect is not applied (the user's
 * memory may not be page-aligned or mmap-backed).
 */

#include "allocator.h"
#include "database.h"
#include "hs_common.h"

#include <stdint.h>
#include <stdlib.h>

/** Return non-zero when the caller has replaced the default db allocator. */
static int using_custom_allocator(void) {
    return hs_database_alloc != malloc;
}

#ifndef _WIN32
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

static long get_page_size(void) {
    static long ps = 0;
    if (ps == 0) {
        ps = sysconf(_SC_PAGESIZE);
    }
    return ps;
}

struct MmapNode {
    void *ptr;
    size_t size;
    struct MmapNode *next;
};

static struct MmapNode *mmap_list = NULL;
static pthread_mutex_t mmap_list_mutex = PTHREAD_MUTEX_INITIALIZER;

// Track mmap allocations explicitly; page alignment is not a reliable signal
// once malloc starts returning page-aligned blocks under heavy churn.

static void mmap_list_add(void *ptr, size_t size) {
    struct MmapNode *node = (struct MmapNode *)malloc(sizeof(*node));
    if (!node) {
        return;
    }
    node->ptr = ptr;
    node->size = size;

    pthread_mutex_lock(&mmap_list_mutex);
    node->next = mmap_list;
    mmap_list = node;
    pthread_mutex_unlock(&mmap_list_mutex);
}

static size_t mmap_list_find(void *ptr) {
    size_t size = 0;
    pthread_mutex_lock(&mmap_list_mutex);
    for (struct MmapNode *node = mmap_list; node; node = node->next) {
        if (node->ptr == ptr) {
            size = node->size;
            break;
        }
    }
    pthread_mutex_unlock(&mmap_list_mutex);
    return size;
}

static size_t mmap_list_remove(void *ptr) {
    size_t size = 0;
    pthread_mutex_lock(&mmap_list_mutex);
    struct MmapNode *prev = NULL;
    struct MmapNode *node = mmap_list;
    while (node) {
        if (node->ptr == ptr) {
            size = node->size;
            if (prev) {
                prev->next = node->next;
            } else {
                mmap_list = node->next;
            }
            break;
        }
        prev = node;
        node = node->next;
    }
    pthread_mutex_unlock(&mmap_list_mutex);
    if (size) {
        free(node);
    }
    return size;
}

/** Round \a size up to the next multiple of the system page size. */
static size_t round_up_page(size_t size) {
    long ps = get_page_size();
    if (ps <= 0) {
        return size;
    }
    return (size + (size_t)ps - 1) & ~((size_t)ps - 1);
}

void *hs_db_alloc(size_t size) {
    if (using_custom_allocator()) {
        return hs_database_alloc(size);
    }
    size_t alloc_size = round_up_page(size);
    void *ptr = mmap(NULL, alloc_size, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (ptr == MAP_FAILED) {
        return NULL;
    }
    mmap_list_add(ptr, alloc_size);
    return ptr;
}

void hs_db_free(void *ptr, size_t size) {
    (void)size;
    if (!ptr) {
        return;
    }
    size_t alloc_size = mmap_list_remove(ptr);
    if (alloc_size) {
        int ret = munmap(ptr, alloc_size);
        if (ret != 0) {
            DEBUG_PRINTF("munmap failed: %d\n", ret);
        }
        (void)ret;
    } else {
        hs_database_free(ptr);
    }
}

void hs_db_protect(void *ptr, size_t size) {
    (void)size;
    if (!ptr) {
        return;
    }
    size_t alloc_size = mmap_list_find(ptr);
    if (!alloc_size) {
        return;
    }
    mprotect(ptr, alloc_size, PROT_READ);
}

void hs_db_unprotect(void *ptr, size_t size) {
    (void)size;
    if (!ptr) {
        return;
    }
    size_t alloc_size = mmap_list_find(ptr);
    if (!alloc_size) {
        return;
    }
    mprotect(ptr, alloc_size, PROT_READ | PROT_WRITE);
}

#else /* _WIN32 */

void *hs_db_alloc(size_t size) {
    if (using_custom_allocator()) {
        return hs_database_alloc(size);
    }
    return malloc(size);
}

void hs_db_free(void *ptr, size_t size) {
    (void)size;
    if (using_custom_allocator()) {
        hs_database_free(ptr);
        return;
    }
    free(ptr);
}

void hs_db_protect(void *ptr, size_t size) {
    (void)ptr; (void)size;
}

void hs_db_unprotect(void *ptr, size_t size) {
    (void)ptr; (void)size;
}

#endif
