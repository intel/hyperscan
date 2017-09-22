/*
 * Copyright (c) 2017, Intel Corporation
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

#include "stream_compress.h"

#include "state.h"
#include "nfa/nfa_internal.h"
#include "rose/rose_internal.h"
#include "util/multibit.h"
#include "util/multibit_compress.h"
#include "util/uniform_ops.h"

#include <string.h>

#define COPY_IN(p, sz) do {                             \
        assert(currOffset + sz <= buf_size);            \
        memcpy(buf + currOffset, p, sz);                \
        currOffset += sz;                               \
        DEBUG_PRINTF("co = %zu\n", currOffset);         \
    } while (0);

#define COPY_OUT(p, sz) do {                            \
        if (currOffset + sz > buf_size) {               \
            return 0;                                   \
        }                                               \
        memcpy(p, buf + currOffset, sz);                \
        currOffset += sz;                               \
        DEBUG_PRINTF("co = %zu\n", currOffset);         \
    } while (0);

#define SIZE_COPY_IN(p, sz) do {                        \
        currOffset += sz;                               \
        DEBUG_PRINTF("co = %zu\n", currOffset);         \
    } while (0);

#define COPY_MULTIBIT_IN(p, total_bits) do {                                \
        size_t sz;                                                          \
        STREAM_QUAL u8 *bits = (STREAM_QUAL u8 *)p;                         \
        BUF_QUAL u8 *comp = (BUF_QUAL u8 *)(buf + currOffset);              \
        if (!mmbit_compress(bits, total_bits, comp, &sz,                    \
                            buf_size - currOffset)) {                       \
            return 0; /* error */                                           \
        }                                                                   \
        currOffset += sz;                                                   \
        DEBUG_PRINTF("co = %zu\n", currOffset);                             \
    } while (0);

#define COPY_MULTIBIT_OUT(p, total_bits) do {                               \
        size_t sz;                                                          \
        STREAM_QUAL u8 *bits = (STREAM_QUAL u8 *)p;                         \
        BUF_QUAL u8 *comp = (BUF_QUAL u8 *)(buf + currOffset);              \
        if (!mmbit_decompress(bits, total_bits, comp, &sz,                  \
                              buf_size - currOffset)) {                     \
            return 0; /* error */                                           \
        }                                                                   \
        currOffset += sz;                                                   \
        DEBUG_PRINTF("co = %zu\n", currOffset);                             \
    } while (0);

#define COPY_MULTIBIT_SIZE(p, total_bits) do {                              \
        STREAM_QUAL u8 *bits = (STREAM_QUAL u8 *)p;                         \
        size_t sz = mmbit_compsize(bits, total_bits);                       \
        currOffset += sz;                                                   \
        DEBUG_PRINTF("co = %zu\n", currOffset);                             \
    } while (0);

#define COPY COPY_OUT
#define COPY_MULTIBIT COPY_MULTIBIT_OUT
#define ASSIGN(lhs, rhs) do { lhs = rhs; } while (0)
#define FN_SUFFIX expand
#define STREAM_QUAL
#define BUF_QUAL const
#include "stream_compress_impl.h"

int expand_stream(struct hs_stream *stream, const struct RoseEngine *rose,
                  const char *buf, size_t buf_size) {
    return sc_expand(rose, stream, buf, buf_size);
}

#define COPY COPY_IN
#define COPY_MULTIBIT COPY_MULTIBIT_IN
#define ASSIGN(lhs, rhs) do { } while (0)
#define FN_SUFFIX compress
#define STREAM_QUAL const
#define BUF_QUAL
#include "stream_compress_impl.h"

size_t compress_stream(char *buf, size_t buf_size,
                       const struct RoseEngine *rose,
                       const struct hs_stream *stream) {
    return sc_compress(rose, stream, buf, buf_size);
}

#define COPY SIZE_COPY_IN
#define COPY_MULTIBIT COPY_MULTIBIT_SIZE
#define ASSIGN(lhs, rhs) do { } while (0)
#define FN_SUFFIX size
#define STREAM_QUAL const
#define BUF_QUAL UNUSED
#include "stream_compress_impl.h"

size_t size_compress_stream(const struct RoseEngine *rose,
                            const struct hs_stream *stream) {
    return sc_size(rose, stream, NULL, 0);
}
