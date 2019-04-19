/*
 * Copyright (c) 2016-2017, Intel Corporation
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
#include "hs_common.h"
#include "hs_runtime.h"
#include "ue2common.h"
#include "util/cpuid_inline.h"
#include "util/join.h"

#if defined(DISABLE_AVX512_DISPATCH)
#define avx512_ disabled_
#define check_avx512() (0)
#endif

#define CREATE_DISPATCH(RTYPE, NAME, ...)                                      \
    /* create defns */                                                         \
    RTYPE JOIN(avx512_, NAME)(__VA_ARGS__);                                    \
    RTYPE JOIN(avx2_, NAME)(__VA_ARGS__);                                      \
    RTYPE JOIN(corei7_, NAME)(__VA_ARGS__);                                    \
    RTYPE JOIN(core2_, NAME)(__VA_ARGS__);                                     \
                                                                               \
    /* error func */                                                           \
    static inline RTYPE JOIN(error_, NAME)(__VA_ARGS__) {                      \
        return (RTYPE)HS_ARCH_ERROR;                                           \
    }                                                                          \
                                                                               \
    /* resolver */                                                             \
    static RTYPE (*JOIN(resolve_, NAME)(void))(__VA_ARGS__) {                  \
        if (check_avx512()) {                                                  \
            return JOIN(avx512_, NAME);                                        \
        }                                                                      \
        if (check_avx2()) {                                                    \
            return JOIN(avx2_, NAME);                                          \
        }                                                                      \
        if (check_sse42() && check_popcnt()) {                                 \
            return JOIN(corei7_, NAME);                                        \
        }                                                                      \
        if (check_ssse3()) {                                                   \
            return JOIN(core2_, NAME);                                         \
        }                                                                      \
        /* anything else is fail */                                            \
        return JOIN(error_, NAME);                                             \
    }                                                                          \
                                                                               \
    /* function */                                                             \
    HS_PUBLIC_API                                                              \
    RTYPE NAME(__VA_ARGS__) __attribute__((ifunc("resolve_" #NAME)))

CREATE_DISPATCH(hs_error_t, hs_scan, const hs_database_t *db, const char *data,
                unsigned length, unsigned flags, hs_scratch_t *scratch,
                match_event_handler onEvent, void *userCtx);

CREATE_DISPATCH(hs_error_t, hs_stream_size, const hs_database_t *database,
                size_t *stream_size);

CREATE_DISPATCH(hs_error_t, hs_database_size, const hs_database_t *db,
                size_t *size);
CREATE_DISPATCH(hs_error_t, dbIsValid, const hs_database_t *db);
CREATE_DISPATCH(hs_error_t, hs_free_database, hs_database_t *db);

CREATE_DISPATCH(hs_error_t, hs_open_stream, const hs_database_t *db,
                unsigned int flags, hs_stream_t **stream);

CREATE_DISPATCH(hs_error_t, hs_scan_stream, hs_stream_t *id, const char *data,
                unsigned int length, unsigned int flags, hs_scratch_t *scratch,
                match_event_handler onEvent, void *ctxt);

CREATE_DISPATCH(hs_error_t, hs_close_stream, hs_stream_t *id,
                hs_scratch_t *scratch, match_event_handler onEvent, void *ctxt);

CREATE_DISPATCH(hs_error_t, hs_scan_vector, const hs_database_t *db,
                const char *const *data, const unsigned int *length,
                unsigned int count, unsigned int flags, hs_scratch_t *scratch,
                match_event_handler onevent, void *context);

CREATE_DISPATCH(hs_error_t, hs_database_info, const hs_database_t *db, char **info);

CREATE_DISPATCH(hs_error_t, hs_copy_stream, hs_stream_t **to_id,
                const hs_stream_t *from_id);

CREATE_DISPATCH(hs_error_t, hs_reset_stream, hs_stream_t *id,
                unsigned int flags, hs_scratch_t *scratch,
                match_event_handler onEvent, void *context);

CREATE_DISPATCH(hs_error_t, hs_reset_and_copy_stream, hs_stream_t *to_id,
                const hs_stream_t *from_id, hs_scratch_t *scratch,
                match_event_handler onEvent, void *context);

CREATE_DISPATCH(hs_error_t, hs_serialize_database, const hs_database_t *db,
                char **bytes, size_t *length);

CREATE_DISPATCH(hs_error_t, hs_deserialize_database, const char *bytes,
                const size_t length, hs_database_t **db);

CREATE_DISPATCH(hs_error_t, hs_deserialize_database_at, const char *bytes,
                const size_t length, hs_database_t *db);

CREATE_DISPATCH(hs_error_t, hs_serialized_database_info, const char *bytes,
                size_t length, char **info);

CREATE_DISPATCH(hs_error_t, hs_serialized_database_size, const char *bytes,
                const size_t length, size_t *deserialized_size);

CREATE_DISPATCH(hs_error_t, hs_compress_stream, const hs_stream_t *stream,
                char *buf, size_t buf_space, size_t *used_space);

CREATE_DISPATCH(hs_error_t, hs_expand_stream, const hs_database_t *db,
                hs_stream_t **stream, const char *buf,size_t buf_size);

CREATE_DISPATCH(hs_error_t, hs_reset_and_expand_stream, hs_stream_t *to_stream,
                const char *buf, size_t buf_size, hs_scratch_t *scratch,
                match_event_handler onEvent, void *context);

/** INTERNALS **/

CREATE_DISPATCH(u32, Crc32c_ComputeBuf, u32 inCrc32, const void *buf, size_t bufLen);
