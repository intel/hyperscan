/*
 * Copyright (c) 2015-2026, Intel Corporation
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
#include <openssl/hmac.h>
#include <openssl/crypto.h>
#include "hs_db_hmac_key.h"
#include "nfa/nfa_internal.h"
#include "nfa/limex_internal.h"
#include "rose/rose_internal.h"
#include "util/compile_error.h"
#include "util/unaligned.h"

/**
 * Runtime equivalent of mmbit_size() for deserialization validation.
 */
static u32 mmbit_size_rt(u32 total_bits) {
    if (total_bits == 0) {
        return 0;
    }
    if (total_bits <= 256) {
        return (total_bits + 7) / 8;
    }
    u64a current_level = 1;
    u64a total = 0;
    while (current_level * 64 < total_bits) {
        total += current_level;
        current_level <<= 6;
    }
    u64a last_level = ((u64a)total_bits + 63) / 64;
    total += last_level;
    return (u32)(total * sizeof(u64a));
}

/**
 * Runtime equivalent of fatbit_size() for deserialization validation.
 */
static u32 fatbit_size_rt(u32 total_bits) {
    u32 mmsz = mmbit_size_rt(total_bits);
    return mmsz > 8 ? mmsz : 8;
}

static really_inline
int db_correctly_aligned(const void *db) {
    return ISALIGNED_N(db, alignof(unsigned long long));
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_free_database(hs_database_t *db) {
    if (db && unlikely(db->magic != HS_DB_MAGIC)) {
        return HS_INVALID;
    }
    if (db) {
        size_t db_len = sizeof(struct hs_database) + db->length;
        hs_db_unprotect(db, db_len);
        hs_db_free(db, db_len);
    }

    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_serialize_database(const hs_database_t *db, char **bytes,
                                          size_t *serialized_length) {
    if (unlikely(!db || !bytes || !serialized_length)) {
        return HS_INVALID;
    }

    if (unlikely(!db_correctly_aligned(db))) {
        return HS_BAD_ALIGN;
    }

    hs_error_t ret = validDatabase(db);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    size_t length = sizeof(struct hs_database) + db->length;

    char *out = hs_misc_alloc(length);
    ret = hs_check_alloc(out);
    if (unlikely(ret != HS_SUCCESS)) {
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
    memcpy(buf, db->hmac, 32);
    buf += (32 / sizeof(u32));

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
    if (unlikely(p != hs_current_platform
        && p != (hs_current_platform | hs_current_platform_no_avx2)
        && p != (hs_current_platform | hs_current_platform_no_avx512)
        && p != (hs_current_platform | hs_current_platform_no_avx512vbmi))) {
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
    if (unlikely(!*bytes)) {
        return HS_INVALID;
    }

    if (unlikely(length < sizeof(struct hs_database))) {
        return HS_INVALID;
    }

    // There's no requirement, really, that the serialized stream of bytes
    // we've been given is 4-byte aligned, so we use unaligned loads here.

    const u32 *buf = (const u32 *)*bytes;

    // Zero header so that none of it (e.g. its padding) is uninitialized.
    memset(header, 0, sizeof(struct hs_database));

    header->magic = unaligned_load_u32(buf++);
    if (unlikely(header->magic != HS_DB_MAGIC)) {
        return HS_INVALID;
    }

    header->version = unaligned_load_u32(buf++);
    if (unlikely(header->version != HS_DB_VERSION)) {
        return HS_DB_VERSION_ERROR;
    }

    header->length = unaligned_load_u32(buf++);
    if (unlikely(length != sizeof(struct hs_database) + header->length)) {
        DEBUG_PRINTF("bad length %zu, expecting %zu\n", length,
                     sizeof(struct hs_database) + header->length);
        return HS_INVALID;
    }

    header->platform = unaligned_load_u64a(buf);
    buf += 2;
    memcpy(header->hmac, buf, 32);
    buf += (32 / sizeof(u32));

    *bytes = (const char *)buf;

    return HS_SUCCESS; // Header checks out
}

// Check the integrity of a database using HMAC-SHA256
static
hs_error_t db_check_integrity(const hs_database_t *db) {
    const char *bytecode = hs_get_bytecode(db);
    u8 computed[32];
    unsigned int hmac_len = 32;
    if (!HMAC(EVP_sha256(), HS_DB_HMAC_KEY, sizeof(HS_DB_HMAC_KEY),
             (const unsigned char *)bytecode, db->length, computed,
             &hmac_len) || hmac_len != 32) {
        DEBUG_PRINTF("hmac computation failed!\n");
        return HS_INVALID;
    }
    if (unlikely(CRYPTO_memcmp(computed, db->hmac, 32) != 0)) {
        DEBUG_PRINTF("hmac mismatch!\n");
        return HS_INVALID;
    }
    return HS_SUCCESS;
}

/**
 * \brief Validate critical RoseEngine offsets to prevent out-of-bounds
 * access (CWE-125) when scanning a deserialized database.
 */

static
hs_error_t db_validate_limex_repeats(const struct RoseEngine *rose,
                                     u32 rose_size) {
    if (!rose->nfaInfoOffset || rose->nfaInfoOffset >= rose_size) {
        return HS_SUCCESS; /* no NFA engines - nothing to check */
    }

    const char *rose_base = (const char *)rose;
    const struct NfaInfo *infos =
        (const struct NfaInfo *)(rose_base + rose->nfaInfoOffset);

    for (u32 qi = 0; qi < rose->queueCount; qi++) {
        if (unlikely((const char *)(&infos[qi + 1]) > rose_base + rose_size)) {
            DEBUG_PRINTF("NfaInfo[%u] out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NfaInfo *ni = &infos[qi];
        if (!ni->nfaOffset || ni->nfaOffset >= rose_size) {
            continue;
        }

        if (unlikely(ni->nfaOffset + sizeof(struct NFA) > rose_size)) {
            DEBUG_PRINTF("NFA[%u] header out of bounds\n", qi);
            return HS_INVALID;
        }

        const struct NFA *nfa =
            (const struct NFA *)(rose_base + ni->nfaOffset);

        if (!isNfaType(nfa->type)) {
            continue;
        }

        if (unlikely(ni->nfaOffset + nfa->length > rose_size)) {
            DEBUG_PRINTF("NFA[%u] body out of bounds\n", qi);
            return HS_INVALID;
        }

        if (unlikely(nfa->length <= sizeof(struct NFA))) {
            DEBUG_PRINTF("NFA[%u] impossibly small\n", qi);
            return HS_INVALID;
        }

        const char *limex_base = (const char *)getImplNfa(nfa);
        u32 limex_len = nfa->length - (u32)sizeof(struct NFA);

        const u32 prefix_end =
            (u32)(offsetof(struct LimExNFA32, flags) + sizeof(u32));
        if (unlikely(limex_len < prefix_end)) {
            DEBUG_PRINTF("LimEx[%u] smaller than common prefix\n", qi);
            return HS_INVALID;
        }

        const struct LimExNFA32 *limex =
            (const struct LimExNFA32 *)limex_base;
        const u32 repeatCount = limex->repeatCount;
        const u32 rep_table_off = limex->repeatOffset;
        const u32 nfa_state_size = limex->stateSize;

        if (repeatCount == 0) {
            continue;
        }

        if (unlikely(repeatCount > limex_len / sizeof(u32))) {
            DEBUG_PRINTF("LimEx[%u] repeatCount too large: %u\n",
                         qi, repeatCount);
            return HS_INVALID;
        }

        if (unlikely(rep_table_off >= limex_len)) {
            DEBUG_PRINTF("LimEx[%u] repeatOffset out of bounds\n", qi);
            return HS_INVALID;
        }

        u64a table_end =
            (u64a)rep_table_off + (u64a)repeatCount * sizeof(u32);
        if (unlikely(table_end > limex_len)) {
            DEBUG_PRINTF("LimEx[%u] repeat offset table overflows\n", qi);
            return HS_INVALID;
        }

        const u32 *rep_offsets =
            (const u32 *)(limex_base + rep_table_off);

        for (u32 i = 0; i < repeatCount; i++) {
            u32 info_off = rep_offsets[i];

            if (unlikely(info_off >= limex_len ||
                         info_off + sizeof(struct NFARepeatInfo) > limex_len)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] info out of bounds\n",
                             qi, i);
                return HS_INVALID;
            }

            const struct NFARepeatInfo *info =
                (const struct NFARepeatInfo *)(limex_base + info_off);

            u64a ctrl_end = (u64a)nfa_state_size +
                            (u64a)info->packedCtrlOffset +
                            (u64a)info->stateSize;
            if (unlikely(ctrl_end > nfa->streamStateSize)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] packedCtrlOffset+stateSize "
                             "exceeds streamStateSize\n", qi, i);
                return HS_INVALID;
            }

            u64a state_end = (u64a)nfa_state_size +
                             (u64a)info->stateOffset;
            if (unlikely(state_end > nfa->streamStateSize)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] stateOffset exceeds "
                             "streamStateSize\n", qi, i);
                return HS_INVALID;
            }

            if (unlikely(info->ctrlIndex >= repeatCount)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] ctrlIndex out of range\n",
                             qi, i);
                return HS_INVALID;
            }

            u64a tug_end = (u64a)info_off + (u64a)info->tugMaskOffset;
            if (unlikely(tug_end >= limex_len)) {
                DEBUG_PRINTF("LimEx[%u] repeat[%u] tugMaskOffset "
                             "out of bounds\n", qi, i);
                return HS_INVALID;
            }
        }
    }

    return HS_SUCCESS;
}

static
hs_error_t db_validate_rose_offsets(const hs_database_t *db) {
    const struct RoseEngine *rose = hs_get_bytecode(db);

    if (unlikely(!rose)) {
        DEBUG_PRINTF("null rose engine\n");
        return HS_INVALID;
    }

    // Get the total RoseEngine size
    u32 rose_size = rose->size;

    // Validate rose_size itself is reasonable
    if (unlikely(rose_size < sizeof(struct RoseEngine) || rose_size > db->length)) {
        DEBUG_PRINTF("invalid rose size: %u (db length: %u)\n",
                     rose_size, db->length);
        return HS_INVALID;
    }

    // Validate critical offsets that are dereferenced during scanning.
    // Each offset, if non-zero, must be within rose_size bounds.
#define VALIDATE_OFFSET(field)                                              \
    if (unlikely(rose->field && rose->field >= rose_size)) {                \
        DEBUG_PRINTF(#field " out of bounds: %u >= %u\n",                  \
                     rose->field, rose_size);                               \
        return HS_INVALID;                                                  \
    }

    // Literal matcher offsets
    VALIDATE_OFFSET(fmatcherOffset);
    VALIDATE_OFFSET(ematcherOffset);
    VALIDATE_OFFSET(amatcherOffset);
    VALIDATE_OFFSET(sbmatcherOffset);
    VALIDATE_OFFSET(drmatcherOffset);
    VALIDATE_OFFSET(longLitTableOffset);
    VALIDATE_OFFSET(smallWriteOffset);

    // NFA / left-engine offsets
    VALIDATE_OFFSET(nfaInfoOffset);
    VALIDATE_OFFSET(leftOffset);
    VALIDATE_OFFSET(activeLeftIterOffset);

    // Program offsets
    VALIDATE_OFFSET(reportProgramOffset);
    VALIDATE_OFFSET(delayProgramOffset);
    VALIDATE_OFFSET(anchoredProgramOffset);
    VALIDATE_OFFSET(eodProgramOffset);
    VALIDATE_OFFSET(flushCombProgramOffset);
    VALIDATE_OFFSET(lastFlushCombProgramOffset);

    // Misc offsets
    VALIDATE_OFFSET(lastByteHistoryIterOffset);
    VALIDATE_OFFSET(eagerIterOffset);
    VALIDATE_OFFSET(somRevOffsetOffset);
    VALIDATE_OFFSET(combInfoMapOffset);

#undef VALIDATE_OFFSET

#define VALIDATE_FATBIT(size_field, count_field)                             \
    if (rose->count_field > 0) {                                            \
        if (unlikely(rose->count_field > rose_size)) {                      \
            DEBUG_PRINTF(#count_field " exceeds rose_size: %u > %u\n",     \
                         rose->count_field, rose_size);                     \
            return HS_INVALID;                                              \
        }                                                                   \
        if (rose->size_field < fatbit_size_rt(rose->count_field)) {         \
            DEBUG_PRINTF(#size_field " too small for " #count_field         \
                         ": %u < %u (count=%u)\n",                         \
                         rose->size_field,                                  \
                         fatbit_size_rt(rose->count_field),                 \
                         rose->count_field);                                \
            return HS_INVALID;                                              \
        }                                                                   \
    }

    VALIDATE_FATBIT(anchored_fatbit_size, anchored_count);
    VALIDATE_FATBIT(delay_fatbit_size, delay_count);
    VALIDATE_FATBIT(somLocationFatbitSize, somLocationCount);
    VALIDATE_FATBIT(handledKeyFatbitSize, handledKeyCount);
    VALIDATE_FATBIT(activeQueueArraySize, queueCount);

#undef VALIDATE_FATBIT

    /* Validate LimEx NFA repeat metadata (CWE-787). */
    if (unlikely(db_validate_limex_repeats(rose, rose_size) != HS_SUCCESS)) {
        DEBUG_PRINTF("LimEx repeat validation failed\n");
        return HS_INVALID;
    }

    DEBUG_PRINTF("rose offset validation passed\n");
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
    if (unlikely(!bytes || !db)) {
        return HS_INVALID;
    }

    // We require the user to deserialize into an 8-byte aligned region.
    if (unlikely(!ISALIGNED_N(db, 8))) {
        return HS_BAD_ALIGN;
    }

    // Decode the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Calculate total required size
    size_t dblength = sizeof(struct hs_database) + header.length;

    // Additional overflow check for size calculation
    if (unlikely(dblength < header.length)) {
        DEBUG_PRINTF("database size calculation overflow\n");
        return HS_INVALID;
    }

    // Validate total deserialized size against centralized defensive limit.
    if (unlikely(dblength > MAX_DATABASE_SIZE)) {
        DEBUG_PRINTF("database total size exceeds maximum: %zu > %zu\n",
                     dblength, (size_t)MAX_DATABASE_SIZE);
        return HS_INVALID;
    }

    // SECURITY NOTE: This function cannot verify that the user-provided 'db'
    // buffer is actually large enough to hold 'dblength' bytes. Users MUST
    // call hs_serialized_database_size() first to determine required size and
    // allocate sufficient memory. The checks above only prevent obviously
    // malicious oversized values to mitigate heap buffer overflow (CWE-122).
    memset(db, 0, dblength);

    // Copy the decoded header into place
    memcpy(db, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, db);

    if (unlikely(db_check_integrity(db) != HS_SUCCESS)) {
        return HS_INVALID;
    }

    // Validate RoseEngine offsets to prevent out-of-bounds access (CWE-125)
    if (unlikely(db_validate_rose_offsets(db) != HS_SUCCESS)) {
        DEBUG_PRINTF("rose offset validation failed\n");
        return HS_INVALID;
    }

    hs_db_protect(db, sizeof(struct hs_database) + header.length);
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_deserialize_database(const char *bytes,
                                            const size_t length,
                                            hs_database_t **db) {
    if (unlikely(!bytes || !db)) {
        return HS_INVALID;
    }

    *db = NULL;

    // Decode and check the header
    hs_database_t header;
    hs_error_t ret = db_decode_header(&bytes, length, &header);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Make sure the serialized database is for our platform
    ret = db_check_platform(header.platform);
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    // Allocate space for new database
    size_t dblength = sizeof(struct hs_database) + header.length;

    // Check for overflow in size calculation
    if (unlikely(dblength < header.length)) {
        DEBUG_PRINTF("database size calculation overflow\n");
        return HS_INVALID;
    }

    // Validate total allocation size against centralized defensive limit.
    if (unlikely(dblength > MAX_DATABASE_SIZE)) {
        DEBUG_PRINTF("database total size exceeds maximum: %zu > %zu\n",
                     dblength, (size_t)MAX_DATABASE_SIZE);
        return HS_INVALID;
    }

    struct hs_database *tempdb = (struct hs_database *)hs_db_alloc(dblength);
    ret = hs_check_alloc(tempdb);
    if (unlikely(ret != HS_SUCCESS)) {
        hs_db_free(tempdb, dblength);
        return ret;
    }

    // Zero new space for safety
    memset(tempdb, 0, dblength);

    // Copy the decoded header into place
    memcpy(tempdb, &header, sizeof(header));

    // Copy the bytecode into the correctly-aligned location, set offsets
    db_copy_bytecode(bytes, tempdb);

    if (unlikely(db_check_integrity(tempdb) != HS_SUCCESS)) {
        hs_db_free(tempdb, dblength);
        return HS_INVALID;
    }

    // Validate RoseEngine offsets to prevent out-of-bounds access (CWE-125)
    if (unlikely(db_validate_rose_offsets(tempdb) != HS_SUCCESS)) {
        DEBUG_PRINTF("rose offset validation failed\n");
        hs_db_free(tempdb, dblength);
        return HS_INVALID;
    }

    hs_db_protect(tempdb, dblength);
    *db = tempdb;
    return HS_SUCCESS;
}

HS_PUBLIC_API
hs_error_t HS_CDECL hs_database_size(const hs_database_t *db, size_t *size) {
    if (unlikely(!size)) {
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
    if (unlikely(ret != HS_SUCCESS)) {
        return ret;
    }

    if (unlikely(!size)) {
        return HS_INVALID;
    }

    *size = sizeof(struct hs_database) + header.length;
    return HS_SUCCESS;
}

hs_error_t dbIsValid(const hs_database_t *db) {
    if (unlikely(db->magic != HS_DB_MAGIC)) {
        DEBUG_PRINTF("bad magic\n");
        return HS_INVALID;
    }

    if (unlikely(db->version != HS_DB_VERSION)) {
        DEBUG_PRINTF("bad version\n");
        return HS_DB_VERSION_ERROR;
    }

    if (unlikely(db_check_platform(db->platform) != HS_SUCCESS)) {
        DEBUG_PRINTF("bad platform\n");
        return HS_DB_PLATFORM_ERROR;
    }

    if (unlikely(!ISALIGNED_16(hs_get_bytecode(db)))) {
        DEBUG_PRINTF("bad alignment\n");
        return HS_INVALID;
    }

    hs_error_t rv = db_check_integrity(db);
    if (unlikely(rv != HS_SUCCESS)) {
        DEBUG_PRINTF("bad integrity check\n");
        return rv;
    }

    // Validate bytecode offset is within the expected header region.
    if (db->bytecode < offsetof(struct hs_database, padding) ||
        db->bytecode > offsetof(struct hs_database, bytes)) {
        DEBUG_PRINTF("bad bytecode offset\n");
        return HS_INVALID;
    }

    const struct RoseEngine *rose = hs_get_bytecode(db);

    // Validate stream-state stateOffsets layout (CWE-122).
    // These invariants mirror fillStateOffsets() in rose_build_bytecode.cpp.
    // Every offset region dereferenced by init_stream() and its helpers
    // must be bounded by stateOffsets.end.
    if (rose->mode == HS_MODE_STREAM) {
        if (unlikely(rose->stateOffsets.end < sizeof(u8))) {
            DEBUG_PRINTF("stateOffsets.end too small: %u\n",
                         rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.history > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.history > end: %u > %u\n",
                         rose->stateOffsets.history, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->historyRequired >
            rose->stateOffsets.end - rose->stateOffsets.history)) {
            DEBUG_PRINTF("historyRequired overflows end: %u > %u\n",
                         rose->historyRequired,
                         rose->stateOffsets.end - rose->stateOffsets.history);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.exhausted > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.exhausted > end: %u > %u\n",
                         rose->stateOffsets.exhausted, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.logicalVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.logicalVec > end: %u > %u\n",
                         rose->stateOffsets.logicalVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.combVec > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.combVec > end: %u > %u\n",
                         rose->stateOffsets.combVec, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somValid > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somValid > end: %u > %u\n",
                         rose->stateOffsets.somValid, rose->stateOffsets.end);
            return HS_INVALID;
        }
        if (unlikely(rose->stateOffsets.somWritable > rose->stateOffsets.end)) {
            DEBUG_PRINTF("stateOffsets.somWritable > end: %u > %u\n",
                         rose->stateOffsets.somWritable, rose->stateOffsets.end);
            return HS_INVALID;
        }
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
