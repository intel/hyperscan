/*
 * Copyright (c) 2018, Intel Corporation
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

/** \file
 * \brief Chimera: database construction, etc.
 */

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "allocator.h"
#include "database.h"
#include "hs.h"
#include "ch.h"
#include "hs_internal.h"
#include "ch_common.h"
#include "ch_alloc.h"
#include "ch_database.h"
#include "ch_internal.h"

static really_inline
int db_correctly_aligned(const void *db) {
    return ISALIGNED_N(db, alignof(unsigned long long));
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_free_database(ch_database_t *hydb) {
    if (hydb && hydb->magic != CH_DB_MAGIC) {
        return CH_INVALID;
    }
    ch_database_free(hydb);

    return CH_SUCCESS;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_database_size(const ch_database_t *hydb, size_t *size) {
    if (!size) {
        return CH_INVALID;
    }

    ch_error_t ret = hydbIsValid(hydb);
    if (unlikely(ret != CH_SUCCESS)) {
        return ret;
    }

    *size = sizeof(struct ch_database) + hydb->length;
    return CH_SUCCESS;
}

/** \brief Identifier prepended to database info. */
static const char CHIMERA_IDENT[] = "Chimera ";

HS_PUBLIC_API
ch_error_t HS_CDECL ch_database_info(const ch_database_t *hydb, char **info) {
    if (!info) {
        return CH_INVALID;
    }
    *info = NULL;

    if (!hydb || !db_correctly_aligned(hydb) || hydb->magic != CH_DB_MAGIC) {
        return HS_INVALID;
    }

    const struct ch_bytecode *bytecode = ch_get_bytecode(hydb);
    char noMulti = (bytecode->flags & CHIMERA_FLAG_NO_MULTIMATCH);
    if (noMulti) {
        size_t len = strlen(CHIMERA_IDENT);
        *info = ch_misc_alloc(len + 1);
        if (!(*info)) {
            return CH_INVALID;
        }
        memcpy((*info), CHIMERA_IDENT, len);
        (*info)[len] = '\0';
        return CH_SUCCESS;
    }

    char *hsinfo = NULL;
    hs_error_t ret = hs_database_info(getHyperscanDatabase(bytecode), &hsinfo);
    if (ret != HS_SUCCESS) {
        assert(!hsinfo);
        return ret;
    }

    size_t hybridlen = strlen(CHIMERA_IDENT);
    size_t hslen = strlen(hsinfo);
    *info = ch_misc_alloc(hybridlen + hslen + 1);
    if (!(*info)) {
        ch_misc_free(hsinfo);
        return CH_INVALID;
    }

    memcpy((*info), CHIMERA_IDENT, hybridlen);
    memcpy((*info) + hybridlen, hsinfo, hslen);
    (*info)[hybridlen + hslen] = '\0';
    ch_misc_free(hsinfo);

    return CH_SUCCESS;
}
