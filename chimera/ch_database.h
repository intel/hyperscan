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
 * \brief Runtime code for ch_database manipulation.
 */

#ifndef CH_DATABASE_H_
#define CH_DATABASE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define PCRE_STATIC
#include <pcre.h>

#include "ch_compile.h" // for CH_MODE_ flags
#include "ue2common.h"
#include "hs_version.h"
#include "hs.h"

#define CH_DB_MAGIC 0xdedededeU //!< Magic number stored in \ref ch_database

/** \brief Main Chimera database header. */
struct ch_database {
    u32 magic;     //!< must be \ref CH_DB_MAGIC
    u32 version;   //!< release version
    u32 length;    //!< total allocated length in bytes
    u32 reserved0; //!< unused
    u32 reserved1; //!< unused
    u32 bytecode;  //!< offset relative to db start
    u32 padding[16];  //!< padding for alignment of rest of bytecode
    char bytes[];
};

/** \brief Chimera bytecode header, which follows the \ref ch_database and is
 * always 64-byte aligned. */
struct ch_bytecode {
    u32 length; //!< length of bytecode including this header struct
    u32 flags;          //!< whole-database flags (CHIMERA_FLAG_NO_MULTIMATCH,
                        // CHIMERA_FLAG_GROUPS)
    u32 patternCount;   //!< total number of patterns
    u32 activeSize;     //!< size of mmbit to store active pattern ids
    u32 databaseOffset; //!< offset for database following \ref ch_bytecode
                        // header
    u32 patternOffset;  //!< points to an array of u32 offsets, each pointing to
                        // a \ref ch_pattern
    u32 unguardedOffset;  //!< pointer to a list of unguarded pattern indices
    u32 unguardedCount;   //!< number of unguarded patterns
    u32 maxCaptureGroups; //!< max number of capture groups used by any pattern
};

/** \brief Per-pattern header.
 *
 * struct is followed in bytecode by:
 * 1. pcre bytecode (always present)
 * 2. pcre study data (sometimes)
 */
struct ch_pattern {
    u32 id;        //!< pattern ID to report to the user
    u32 flags;     //!< per-pattern flags (e.g. \ref CHIMERA_PATTERN_FLAG_UTF8)
    u32 maxWidth;  //!< maximum width of a match, or UINT_MAX for inf.
    u32 minWidth;  //!< minimum width of a match.
    u32 fixedWidth;//!< pattern has fixed width.
    u32 studyOffset;     //!< offset relative to struct start of study data,
                         // or zero if there is none
    u32 length; //!< length of struct plus pcre bytecode and study data
    pcre_extra extra; //!< pcre_extra struct, used to store study data ptr for
                      // the currently-running pcre at runtime.
};

static really_inline
const void *ch_get_bytecode(const struct ch_database *db) {
    assert(db);
    const void *bytecode = (const char *)db + db->bytecode;
    assert(ISALIGNED_16(bytecode));
    return bytecode;
}

struct hs_database;

static really_inline
const struct hs_database *getHyperscanDatabase(const struct ch_bytecode *db) {
    assert(db);
    const char *ptr = (const char *)db;
    const struct hs_database *hs_db;
    hs_db = (const struct hs_database *)(ptr + db->databaseOffset);
    assert(ISALIGNED_CL(hs_db));
    return hs_db;
}

static really_inline
const u32 *getUnguarded(const struct ch_bytecode *db) {
    assert(db);
    const char *ptr = (const char *)db;
    const u32 *unguarded = (const u32 *)(ptr + db->unguardedOffset);
    assert(ISALIGNED_N(unguarded, sizeof(u32)));
    return unguarded;
}

static really_inline
const struct ch_pattern *getPattern(const struct ch_bytecode *db, u32 i) {
    assert(db);
    assert(i < db->patternCount);
    const char *ptr = (const char *)db;
    const u32 *patternOffset = (const u32 *)(ptr + db->patternOffset);
    assert(patternOffset[i] < db->length);
    return (const struct ch_pattern *)(ptr + patternOffset[i]);
}

static really_inline
ch_error_t hydbIsValid(const struct ch_database *hydb) {
    if (!hydb || hydb->magic != CH_DB_MAGIC) {
        DEBUG_PRINTF("bad magic (%u != %u)\n", hydb->magic, CH_DB_MAGIC);
        return CH_INVALID;
    }

    if (hydb->version != HS_VERSION_32BIT) {
        DEBUG_PRINTF("bad version\n");
        return CH_DB_VERSION_ERROR;
    }

    return CH_SUCCESS;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CH_DATABASE_H_ */

