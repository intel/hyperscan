/*
 * Copyright (c) 2016, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include "hs.h"
#include "ue2common.h"

#include "common.h"
#include "huge.h"

#ifndef _WIN32
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#if defined(HAVE_SHMGET)
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

UNUSED static int hsdb_shmid;

using namespace std;

long gethugepagesize(void);

hs_database_t *get_huge(hs_database_t *db) {
#if defined(HAVE_SHMGET) && defined(SHM_HUGETLB)
    /* move the database to huge pages where possible, but fail politely */
    hs_error_t err;
    size_t len;
    char *bytes;

    long hpage_size = gethugepagesize();
    if (hpage_size < 0) {
        printf("Couldn't determine huge page size\n");
        hsdb_shmid = -1;
        return db;
    }

    err = hs_serialize_database(db, &bytes, &len);
    if (err != HS_SUCCESS) {
        printf("Failed to serialize database for copy: %d\n", err);
        // this is weird - don't fail gracefully this time
        return nullptr;
    }

    size_t size;
    err = hs_serialized_database_size(bytes, len, &size);
    if (err != HS_SUCCESS) {
        printf("Failed to get database size: %d\n", err);
        // this is weird - don't fail gracefully this time
        return nullptr;
    }

    void *shmaddr;
    if ((hsdb_shmid = shmget(IPC_PRIVATE, ROUNDUP_N(size, gethugepagesize()),
                SHM_HUGETLB | IPC_CREAT | SHM_R | SHM_W)) < 0) {
        // This could fail if the user doesn't have permission to shmget(),
        // which is OK.
        goto fini;
    }

    shmaddr = shmat(hsdb_shmid, nullptr, SHM_RND);
    if (shmaddr == (char *)-1) {
        perror("Shared memory attach failure");
        goto fini;
    }

    // Mark this segment to be destroyed after this process detaches.
    shmctl(hsdb_shmid, IPC_RMID, nullptr);

    err = hs_deserialize_database_at(bytes, len, (hs_database_t *)shmaddr);
    if (err != HS_SUCCESS) {
        printf("Failed to deserialize database into shm: %d\n", err);
        shmdt((const void *)shmaddr);
        goto fini;
    }

    free(bytes);
    hs_free_database(db);
    return (hs_database_t *)shmaddr;

fini:
    free(bytes);
    hsdb_shmid = -1;
    return db;
#else
    return db;
#endif
}

void release_huge(hs_database_t *db) {
#if defined(HAVE_SHMGET) && defined(SHM_HUGETLB)
    if (hsdb_shmid != -1) {
        if (shmdt((const void *)db) != 0) {
            perror("Detach failure");
        }
    } else {
        // fallback
        hs_free_database(db);
    }
#else
    hs_free_database(db);
#endif
}

#define BUF_SIZE 4096
static long read_meminfo(const char *tag) {
    int fd;
    char buf[BUF_SIZE];
    int len;
    char *p, *q;
    long val;

    fd = open("/proc/meminfo", O_RDONLY);
    if (fd < 0) {
        perror("Couldn't open /proc/meminfo");
        return -1;
    }

    len = read(fd, buf, sizeof(buf));
    close(fd);
    if (len < 0) {
        perror("Error reading /proc/meminfo");
        return -1;
    }
    if (len == sizeof(buf)) {
        printf("/proc/meminfo is too large\n");
        return -1;
    }
    buf[len] = '\0';

    p = strstr(buf, tag);
    if (!p) {
        return -1;
    }

    p += strlen(tag);
    val = strtol(p, &q, 0);
    if (!isspace(*q)) {
        printf("Couldn't parse /proc/meminfo value\n");
        return -1;
    }

    return val;
}

long gethugepagesize(void) {
    long hpage_size;
    int hpage_kb;

    hpage_kb = read_meminfo("Hugepagesize:");
    if (hpage_kb < 0) {
        hpage_size = -1;
    } else {
        /* convert from kb to bytes */
        hpage_size = 1024 * hpage_kb;
    }

    return hpage_size;
}

#else

/* No huge page support on WIN32. */

hs_database_t *get_huge(hs_database_t *db) { return db; }

void release_huge(hs_database_t *db) { hs_free_database(db); }

#endif
