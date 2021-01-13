/*
 * Copyright (c) 2015-2020, Intel Corporation
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
 * \brief Runtime code for hs_database manipulation.
  */

#include <stdio.h>
#include <string.h>

#include "allocator.h"
#include "hs_common.h"
#include "hs_internal.h"
#include "hs_version.h"
#include "ue2common.h"
#include "database.h"
#include "crc32.h"
#include "rose/rose_internal.h"
#include "util/unaligned.h"

static really_inline
int db_correctly_aligned(const void *db) {
    return ISALIGNED_N(db, alignof(unsigned long long));
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_free_database(hs_database_t *db) {
    if (db && db->magic != HS_DB_MAGIC) {
        return HS_INVALID;
    }
    hs_database_free(db);

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialize_database(const hs_database_t *db, char **bytes,
                                          size_t *serialized_length) {
    if (!db || !bytes || !serialized_length) {
        return HS_INVALID;
    }

    if (!db_correctly_aligned(db)) {
        return HS_BAD_ALIGN;
    }

    hs_error_t ret = validDatabase(db);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    size_t length = sizeof(struct hs_database) + db->length;

    char *out = hs_misc_alloc(length);
    ret = hs_check_alloc(out);
    if (ret != HS_SUCCESS) {
        hs_misc_free(out);
        return ret;
    }

    memset(out, 0, length);

    u32 *buf = (u32 *)out;
    *buf = db->magic;
    buf++;
    *buf = db->version;
    buf++;
    *buf = db->length;
    buf++;
    memcpy(buf, &db->platform, sizeof(u64a));
    buf += 2;
    *buf = db->crc32;
    buf++;
    *buf = db->reserved0;
    buf++;
    *buf = db->reserved1;
    buf++;

    const char *bytecode = hs_get_bytecode(db);
    memcpy(buf, bytecode, db->length);

    *bytes = out;
    *serialized_length = length;
    return HS_SUCCESS;
}

// check that the database header's platform is compatible with the current
// runtime platform.
static
hs_error_t db_check_platform(const u64a p) {
    if (p != hs_current_platform
        && p != (hs_current_platform | hs_current_platform_no_avx2)
        && p != (hs_current_platform | hs_current_platform_no_avx512)
        && p != (hs_current_platform | hs_current_platform_no_avx512vbmi)) {
        return HS_DB_PLATFORM_ERROR;
    }
    // passed all checks
    return HS_SUCCESS;
}

// Decode and check the database header, returning appropriate errors or
// HS_SUCCESS if it's OK. The header should be allocated on the stack
// and later copied into the deserialized database.
static
hs_error_t db_decode_header(const char **bytes, const size_t length,
                            struct hs_database *header) {
    if (!*bytes) {
        return HS_INVALID;
    }

    if (length < sizeof(struct hs_database)) {
        return HS_INVALID;
    }

    // There's no requirement, really, that the serialized stream of bytes
    // we've been given is 4-byte aligned, so we use unaligned loads here.

    const u32 *buf = (const u32 *)*bytes;

    // Zero header so that none of it (e.g. its padding) is uninitialized.
    memset(header, 0, sizeof(struct hs_database));

    header->magic = unaligned_load_u32(buf++);
    if (header->magic != HS_DB_MAGIC) {
        return HS_INVALID;
    }

    header->version = unaligned_load_u32(buf++);
    if (header->version != HS_DB_VERSION) {
        return HS_DB_VERSION_ERROR;
    }

    header->length = unaligned_load_u32(buf++);
    if (length != sizeof(struct hs_database) + header->length) {
        DEBUG_PRINTF("bad length %zu, expecting %zu\n", length,
                     sizeof(struct hs_database) + header->length);
        return HS_INVALID;
    }

    header->platform = unaligned_load_u64a(buf);
    buf += 2;
    header->crc32 = unaligned_load_u32(buf++);
    header->reserved0 = unaligned_load_u32(buf++);
    header->reserved1 = unaligned_load_u32(buf++);

    *bytes = (const char *)buf;

    return HS_SUCCESS; // Header checks out
}

// Check the CRC on a database
static
hs_error_t db_check_crc(const hs_database_t *db) {
    const char *bytecode = hs_get_bytecode(db);
    u32 crc = Crc32c_ComputeBuf(0, bytecode, db->length);
    if (crc != db->crc32) {
        DEBUG_PRINTF("crc mismatch! 0x%x != 0x%x\n", crc, db->crc32);
        return HS_INVALID;
    }
    return HS_SUCCESS;
}

static
void db_copy_bytecode(const char *serialized, hs_database_t *db) {
    // we need to align things manually
    uintptr_t shift = (uintptr_t)db->bytes & 0x3f;
    db->bytecode = offsetof(struct hs_database, bytes) - shift;
    char *bytecode = (char *)db + db->bytecode;

    // Copy the bytecode into place
    memcpy(bytecode, serialized, db->length);
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_deserialize_database_at(const char *bytes,
                                               const size_t length,
                                               hs_database_t *db) {
    if (!bytes || !db) {
        return HS_INVALID;
    }

    // We require the user to deserialize into an 8-byte aligned region.
    if (!ISALIGNED_N(db, 8)) {
        return HS_BAD_ALIGN;
    }

    // Decode the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    // Zero new space for safety
    size_t dblength = sizeof(struct hs_database) + header.length;
    memset(db, 0, dblength);

    // Copy the decoded header into place
    memcpy(db, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, db);

    if (db_check_crc(db) != HS_SUCCESS) {
        return HS_INVALID;
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_deserialize_database(const char *bytes,
                                            const size_t length,
                                            hs_database_t **db) {
    if (!bytes || !db) {
        return HS_INVALID;
    }

    *db = NULL;

    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    // Allocate space for new database
    size_t dblength = sizeof(struct hs_database) + header.length;
    struct hs_database *tempdb = hs_database_alloc(dblength);
    ret = hs_check_alloc(tempdb);
    if (ret != HS_SUCCESS) {
        hs_database_free(tempdb);
        return ret;
    }

    // Zero new space for safety
    memset(tempdb, 0, dblength);

    // Copy the decoded header into place
    memcpy(tempdb, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, tempdb);

    if (db_check_crc(tempdb) != HS_SUCCESS) {
        hs_database_free(tempdb);
        return HS_INVALID;
    }

    *db = tempdb;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_database_size(const hs_database_t *db, size_t *size) {
    if (!size) {
        return HS_INVALID;
    }

    hs_error_t ret = validDatabase(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    *size = sizeof(struct hs_database) + db->length;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialized_database_size(const char *bytes,
                                                const size_t length,
                                                size_t *size) {
    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    if (!size) {
        return HS_INVALID;
    }

    *size = sizeof(struct hs_database) + header.length;
    return HS_SUCCESS;
}

hs_error_t dbIsValid(const hs_database_t *db) {
    if (db->magic != HS_DB_MAGIC) {
        DEBUG_PRINTF("bad magic\n");
        return HS_INVALID;
    }

    if (db->version != HS_DB_VERSION) {
        DEBUG_PRINTF("bad version\n");
        return HS_DB_VERSION_ERROR;
    }

    if (db_check_platform(db->platform) != HS_SUCCESS) {
        DEBUG_PRINTF("bad platform\n");
        return HS_DB_PLATFORM_ERROR;
    }

    if (!ISALIGNED_16(hs_get_bytecode(db))) {
        DEBUG_PRINTF("bad alignment\n");
        return HS_INVALID;
    }

    hs_error_t rv = db_check_crc(db);
    if (rv != HS_SUCCESS) {
        DEBUG_PRINTF("bad crc\n");
        return rv;
    }

    return HS_SUCCESS;
}

#if defined(_WIN32)
#define SNPRINTF_COMPAT _snprintf
#else
#define SNPRINTF_COMPAT snprintf
#endif

/** Allocate a buffer and prints the database info into it. Returns an
 * appropriate error code on failure, or HS_SUCCESS on success. */
static
hs_error_t print_database_string(char **s, u32 version, const platform_t plat,
                                 u32 raw_mode) {
    assert(s);
    *s = NULL;

    u8 release = (version >> 8) & 0xff;
    u8 minor = (version >> 16) & 0xff;
    u8 major = (version >> 24) & 0xff;

    const char *features = (plat & HS_PLATFORM_NOAVX512VBMI)
                               ? (plat & HS_PLATFORM_NOAVX512)
                                   ? (plat & HS_PLATFORM_NOAVX2) ? "" : "AVX2"
                                   : "AVX512"
                               : "AVX512VBMI";

    const char *mode = NULL;

    if (raw_mode == HS_MODE_STREAM) {
        mode = "STREAM";
    } else if (raw_mode == HS_MODE_VECTORED) {
        mode = "VECTORED";
    } else {
        assert(raw_mode == HS_MODE_BLOCK);
        mode = "BLOCK";
    }

    // Initial allocation size, which should be large enough to print our info.
    // If it isn't, snprintf will tell us and we can resize appropriately.
    size_t len = 256;

    while (1) {
        char *buf = hs_misc_alloc(len);
        hs_error_t ret = hs_check_alloc(buf);
        if (ret != HS_SUCCESS) {
            hs_misc_free(buf);
            return ret;
        }

        // Note: SNPRINTF_COMPAT is a macro defined above, to cope with systems
        // that don't have snprintf but have a workalike.
        int p_len = SNPRINTF_COMPAT(
            buf, len, "Version: %u.%u.%u Features: %s Mode: %s",
            major, minor, release, features, mode);
        if (p_len < 0) {
            DEBUG_PRINTF("snprintf output error, returned %d\n", p_len);
            hs_misc_free(buf);
            break;
        } else if ((size_t)p_len < len) { // output fit within buffer.
            assert(buf[p_len] == '\0');
            *s = buf;
            return HS_SUCCESS;
        } else { // output didn't fit: resize and reallocate.
            len = (size_t)p_len + 1; // must add one for null terminator.
            hs_misc_free(buf);
        }
    }

    return HS_NOMEM;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialized_database_info(const char *bytes,
                                                size_t length, char **info) {
    if (!info) {
        return HS_INVALID;
    }
    *info = NULL;

    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (ret != HS_SUCCESS) {
        return ret;
    }

    u32 mode = unaligned_load_u32(bytes + offsetof(struct RoseEngine, mode));

    return print_database_string(info, header.version, header.platform, mode);
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_database_info(const hs_database_t *db, char **info) {
    if (!info) {
        return HS_INVALID;
    }
    *info = NULL;

    if (!db || !db_correctly_aligned(db) || db->magic != HS_DB_MAGIC) {
        return HS_INVALID;
    }

    platform_t plat;
    plat = db->platform;

    const struct RoseEngine *rose = hs_get_bytecode(db);

    return print_database_string(info, db->version, plat, rose->mode);
}
