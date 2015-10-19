/*
 * Copyright (c) 2015, Intel Corporation
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

#include "sidecar.h"
#include "sidecar_internal.h"
#include "sidecar_shufti.h"
#include "ue2common.h"
#include "nfa/vermicelli.h"
#include "util/bitutils.h"
#include "util/uniform_ops.h"

static really_inline
u32 findAndClearLSB_8(u8 *v) {
    u32 t = *v;
    u32 rv = findAndClearLSB_32(&t);
    *v = t;
    return rv;
}

static really_inline
u32 findAndClearLSB_128(m128 *v) {
    union {
        u32 words[sizeof(m128)/sizeof(u32)];
        m128 simd;
    } s;
    s.simd = *v;
    u32 rv = 0;
    for (u32 i = 0; i < ARRAY_LENGTH(s.words); i++) {
        u32 *w = &s.words[i];
        if (*w) {
            rv = findAndClearLSB_32(w) + 32 * i;
            break;
        }
    }

    *v = s.simd;
    return rv;
}

static never_inline
u32 findAndClearLSB_256(m256 *v) {
    union {
        u32 words[sizeof(m256)/sizeof(u32)];
        m256 simd;
    } s;
    s.simd = *v;
    u32 rv = 0;
    for (u32 i = 0; i < ARRAY_LENGTH(s.words); i++) {
        u32 *w = &s.words[i];
        if (*w) {
            rv = findAndClearLSB_32(w) + 32 * i;
            break;
        }
    }

    *v = s.simd;
    return rv;
}

#define DO_DEAD_CHECK 1

#define TAG 8
#define STATE_T u8
#include "sidecar_generic.h"

#define TAG 32
#define STATE_T u32
#include "sidecar_generic.h"

#define TAG 64
#define STATE_T u64a
#include "sidecar_generic.h"

#define TAG 128
#define STATE_T m128
#include "sidecar_generic.h"

#define TAG 256
#define STATE_T m256
#include "sidecar_generic.h"


static never_inline
void sidecarExec_N(const struct sidecar_N *n, const u8 *b, size_t len,
                   struct sidecar_enabled_N *enabled,
                   UNUSED struct sidecar_scratch *scratch,
                   u64a base_offset, SidecarCallback cb, void *context) {
    DEBUG_PRINTF("N: %hhu %hhu nc %hhu\n", n->c, b[0], n->nocase);
    if (!enabled->bits) {
        return;
    }

    const u8 *loc = vermicelliExec(n->c, n->nocase, b, b + len);

    if (loc == b + len) {
        return;
    }

    enabled->bits = 0;
    for (u32 i = 0; i < n->report_count; i++) {
        cb(loc - b + base_offset, n->reports[i], context);
    }
}

static really_inline
void sidecarEnabledInit_N(struct sidecar_enabled *enabled) {
    struct sidecar_enabled_N *e = (void *)enabled;
    e->bits = 0;
}

static really_inline
void sidecarExec_i_S(UNUSED const struct sidecar_S *n,
                     UNUSED const u8 *b, UNUSED size_t len,
                     UNUSED struct sidecar_enabled_S *enabled,
                     UNUSED u64a base_offset, UNUSED SidecarCallback cb,
                     UNUSED void *context) {
    if (!enabled->bits) {
        DEBUG_PRINTF("bail early, bail often\n");
        return;
    }

    u8 state;
    if (len >= 16) {
        state = sidecarExec_S_int(n, b, len, enabled->bits);
    } else {
        const u8 *lo = (const u8 *)&n->lo;
        const u8 *hi = (const u8 *)&n->hi;
        state = enabled->bits;
        for (u32 i = 0; i < len; i++) {
            u8 c = b[i];
            state &= lo[c & 0xf] | hi[c >> 4];
        }
    }

    state = ~state & enabled->bits;
    if (!state) {
        DEBUG_PRINTF("bail\n");
        return;
    }

    enabled->bits &= ~state;
    DEBUG_PRINTF("s = %02hhx e = %02hhx\n", state, enabled->bits);
    u8 unshared = n->unshared_mask;
    const u8 *masks = sidecar_ids_to_mask_const(n);
    const struct sidecar_id_offset *id_map = n->id_list;
    while (state) {
        u32 bit = findAndClearLSB_8(&state);
        DEBUG_PRINTF("found bit %u\n", bit);
        const u32 *id_base = (const u32 *)((const char *)n
                                           + id_map[bit].first_offset);
        u32 count = id_map[bit].count;
        for (u32 i = 0; i < count; ++i) {
            DEBUG_PRINTF("firing %u\n", id_base[i]);
            cb(base_offset, id_base[i], context);
            enabled->bits &= ~(masks[id_base[i]] & unshared);
        }
    }
    DEBUG_PRINTF("s = %02hhx e = %02hhx\n", state, enabled->bits);
}

static really_inline
void sidecarEnabledInit_S(struct sidecar_enabled *enabled) {
    struct sidecar_enabled_S *e = (void *)enabled;
    e->bits = 0;
}

static never_inline
void sidecarExec_S(const struct sidecar_S *n, const u8 *b, size_t len,
                   struct sidecar_enabled_S *enabled,
                   UNUSED struct sidecar_scratch *scratch,
                   u64a base_offset, SidecarCallback cb, void *context) {
    if (len > 1) {
        sidecarExec_i_S(n, b + 1, len - 1, enabled, base_offset + 1, cb,
                        context);
    }

    u8 bits = enabled->bits; /* first byte doesn't change enabled */
    sidecarExec_i_S(n, b, 1, enabled, base_offset, cb, context);
    enabled->bits = bits;
}

void sidecarExec(const struct sidecar *n, const u8 *buffer, size_t len,
                 struct sidecar_enabled *enabled,
                 UNUSED struct sidecar_scratch *scratch, u64a base_offset,
                 SidecarCallback cb, void *ctxt) {
    assert(n);
    assert(enabled);
    assert(len);

    assert(ISALIGNED_N(n, 16));
    assert(ISALIGNED_N(scratch, 16));

    if (!len) {
        return;
    }

#define EXEC_CASE(tag) \
    case SIDECAR_##tag:                                                 \
        sidecarExec_##tag((const struct sidecar_##tag *)n, buffer,  len, \
                          (struct sidecar_enabled_##tag *)enabled, scratch, \
                          base_offset, cb, ctxt);                       \
        break;

    switch(n->type) {
        EXEC_CASE(8)
        EXEC_CASE(32)
        EXEC_CASE(64)
        EXEC_CASE(128)
        EXEC_CASE(256)
        EXEC_CASE(N)
        EXEC_CASE(S)
    default:
        assert(0);
    }

#undef EXEC_CASE
}

void sidecarEnabledInit(const struct sidecar *n,
                        struct sidecar_enabled *enabled) {
    switch(n->type) {
    case SIDECAR_8:
        sidecarEnabledInit_8(enabled);
        break;
    case SIDECAR_32:
        sidecarEnabledInit_32(enabled);
        break;
    case SIDECAR_64:
        sidecarEnabledInit_64(enabled);
        break;
    case SIDECAR_128:
        sidecarEnabledInit_128(enabled);
        break;
    case SIDECAR_256:
        sidecarEnabledInit_256(enabled);
        break;
    case SIDECAR_N:
        sidecarEnabledInit_N(enabled);
        break;
    case SIDECAR_S:
        sidecarEnabledInit_S(enabled);
        break;
    default:
        assert(0);
    }
}

u32 sidecarScratchSize(const struct sidecar *n) {
    u32 width;

    switch(n->type) {
    case SIDECAR_8:
        width = sizeof(struct sidecar_mr_8);
        break;
    case SIDECAR_32:
        width = sizeof(struct sidecar_mr_32);
        break;
    case SIDECAR_64:
        width = sizeof(struct sidecar_mr_64);
        break;
    case SIDECAR_128:
        width = sizeof(struct sidecar_mr_128);
        break;
    case SIDECAR_256:
        width = sizeof(struct sidecar_mr_256);
        break;
    case SIDECAR_N:
        return 0; /* no scratch required for N */
    case SIDECAR_S:
        width = sizeof(struct sidecar_mr_8);
        break;
    default:
        assert(0);
        return 0;
    }

    /* + 1, for first byte offset */
    return width * (n->mask_bit_count + 1);
}

static really_inline
void sidecarEnabledUnion_N(struct sidecar_enabled *dest,
                           const struct sidecar_enabled *src) {
    struct sidecar_enabled_N *d = (void *)dest;
    const struct sidecar_enabled_N *s = (const void *)src;
    d->bits |= s->bits;
}

static really_inline
void sidecarEnabledUnion_S(struct sidecar_enabled *dest,
                           const struct sidecar_enabled *src) {
    struct sidecar_enabled_S *d = (void *)dest;
    const struct sidecar_enabled_S *s = (const void *)src;
    d->bits |= s->bits;
}

void sidecarEnabledUnion(const struct sidecar *n, struct sidecar_enabled *dest,
                         const struct sidecar_enabled *src) {
    switch(n->type) {
    case SIDECAR_8:
        sidecarEnabledUnion_8(dest, src);
        break;
    case SIDECAR_32:
        sidecarEnabledUnion_32(dest, src);
        break;
    case SIDECAR_64:
        sidecarEnabledUnion_64(dest, src);
        break;
    case SIDECAR_128:
        sidecarEnabledUnion_128(dest, src);
        break;
    case SIDECAR_256:
        sidecarEnabledUnion_256(dest, src);
        break;
    case SIDECAR_N:
        sidecarEnabledUnion_N(dest, src);
        break;
    case SIDECAR_S:
        sidecarEnabledUnion_S(dest, src);
        break;
    default:
        assert(0);
    }
}
