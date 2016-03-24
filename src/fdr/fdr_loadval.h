/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#ifndef FDR_LOADVAL_H
#define FDR_LOADVAL_H

#include "fdr_internal.h"
#include "ue2common.h"
#include "util/unaligned.h"
#include "util/simd_utils.h"

#define MAKE_LOADVAL(type, name) \
    static really_inline type name (const u8 * ptr, UNUSED const u8 * lo, UNUSED const u8 * hi)

#define NORMAL_SAFE(type)                                               \
    do {                                                                \
        assert(ptr >= lo);                                              \
        assert(ptr + sizeof(type) - 1 < hi);                            \
    } while(0)

#define ALIGNED_SAFE(type)           NORMAL_SAFE(type); assert(((size_t)ptr % sizeof(type)) == 0);
// these ones need asserts to test the property that we're not handling dynamically
#define CAUTIOUS_FORWARD_SAFE(type)  assert(ptr >= lo)
#define CAUTIOUS_BACKWARD_SAFE(type) assert((ptr + sizeof(type) - 1) < hi)

#define CF_INDEX_CHECK                        (ptr + i < hi)
#define CB_INDEX_CHECK     (lo <= ptr + i)
#define CE_INDEX_CHECK     (lo <= ptr + i) && (ptr + i < hi)

#define MAKE_LOOP(TYPE, COND, SHIFT_FIDDLE)                                    \
    TYPE v = 0;                                                                \
    for (TYPE i = 0; i < sizeof(TYPE); i++) {                                  \
        if (COND) {                                                            \
            v += (TYPE)ptr[i] << ((SHIFT_FIDDLE)*8);                           \
        }                                                                      \
    }                                                                          \
    return v;

#define MAKE_LOOP_BE(TYPE, COND) \
    MAKE_LOOP(TYPE, COND, sizeof(TYPE)-i-1)

#define MAKE_LOOP_LE(TYPE, COND) \
    MAKE_LOOP(TYPE, COND, i)


#define MAKE_LOOP_BE_CF(TYPE) CAUTIOUS_FORWARD_SAFE(TYPE);  MAKE_LOOP_BE(TYPE, CF_INDEX_CHECK)
#define MAKE_LOOP_BE_CB(TYPE) CAUTIOUS_BACKWARD_SAFE(TYPE); MAKE_LOOP_BE(TYPE, CB_INDEX_CHECK)
#define MAKE_LOOP_BE_CE(TYPE)                               MAKE_LOOP_BE(TYPE, CE_INDEX_CHECK)
#define MAKE_LOOP_LE_CF(TYPE) CAUTIOUS_FORWARD_SAFE(TYPE);  MAKE_LOOP_LE(TYPE, CF_INDEX_CHECK)
#define MAKE_LOOP_LE_CB(TYPE) CAUTIOUS_BACKWARD_SAFE(TYPE); MAKE_LOOP_LE(TYPE, CB_INDEX_CHECK)
#define MAKE_LOOP_LE_CE(TYPE)                               MAKE_LOOP_LE(TYPE, CE_INDEX_CHECK)

// no suffix = normal (unaligned)
// _a        = aligned
// _cf       = cautious forwards, base is always in bounds, but may read over the end of the buffer (test against hi)
// _cb       = cautious backwards, final byte is always in bounds, but may read over the start of the buffer (test against lo)
// _ce       = cautious everywhere (in both directions); test against hi and lo

// u8 loadvals
MAKE_LOADVAL(u8, lv_u8) {
    NORMAL_SAFE(u8);
    return *ptr;
}

MAKE_LOADVAL(u8, lv_u8_cf) {
    CAUTIOUS_FORWARD_SAFE(u8);
    if (ptr < hi) {
        return *ptr;
    } else {
        return 0;
    }
}

MAKE_LOADVAL(u8, lv_u8_cb) {
    CAUTIOUS_BACKWARD_SAFE(u8);
    if (lo <= ptr) {
        return *ptr;
    } else {
        return 0;
    }
}

MAKE_LOADVAL(u8, lv_u8_ce) {
    if ((lo <= ptr) && (ptr < hi)) {
        return *ptr;
    } else {
        return 0;
    }
}

MAKE_LOADVAL(u16, lv_u16) {
    NORMAL_SAFE(u16);
    return unaligned_load_u16(ptr);
}

MAKE_LOADVAL(u16, lv_u16_a) {
    ALIGNED_SAFE(u16);
    return *(const u16 *)ptr;
}

MAKE_LOADVAL(u32, lv_u32) {
    NORMAL_SAFE(u32);
    return unaligned_load_u32(ptr);
}

MAKE_LOADVAL(u32, lv_u32_a) {
    ALIGNED_SAFE(u32);
    return *(const u32 *)ptr;
}

MAKE_LOADVAL(u64a, lv_u64a) {
    NORMAL_SAFE(u32);
    return unaligned_load_u64a(ptr);
}

MAKE_LOADVAL(u64a, lv_u64a_a) {
    ALIGNED_SAFE(u64a);
    return *(const u64a *)ptr;
}

MAKE_LOADVAL(u16, lv_u16_cf) { MAKE_LOOP_LE_CF(u16); }
MAKE_LOADVAL(u16, lv_u16_cb) { MAKE_LOOP_LE_CB(u16); }
MAKE_LOADVAL(u16, lv_u16_ce) { MAKE_LOOP_LE_CE(u16); }

MAKE_LOADVAL(u32, lv_u32_cf) { MAKE_LOOP_LE_CF(u32); }
MAKE_LOADVAL(u32, lv_u32_cb) { MAKE_LOOP_LE_CB(u32); }
MAKE_LOADVAL(u32, lv_u32_ce) { MAKE_LOOP_LE_CE(u32); }

MAKE_LOADVAL(u64a, lv_u64a_cf) { MAKE_LOOP_LE_CF(u64a); }
MAKE_LOADVAL(u64a, lv_u64a_cb) { MAKE_LOOP_LE_CB(u64a); }
MAKE_LOADVAL(u64a, lv_u64a_ce) { MAKE_LOOP_LE_CE(u64a); }

MAKE_LOADVAL(m128, lv_m128) {
    NORMAL_SAFE(m128);
    return loadu128(ptr);
}

MAKE_LOADVAL(m128, lv_m128_a) {
    ALIGNED_SAFE(m128);
    assert((size_t)ptr % sizeof(m128) == 0);
    return *(const m128 *)ptr;
}

// m128 cases need to be manually created

MAKE_LOADVAL(m128, lv_m128_cf) {
    CAUTIOUS_FORWARD_SAFE(m128);
    union {
        u8 val8[16];
        m128 val128;
    } u;

    for (u32 i = 0; i < 16; i++) {
        if (ptr + i < hi) {
            u.val8[i] = ptr[i];
        } else {
            u.val8[i] = 0;
        }
    }
    return u.val128;
}

MAKE_LOADVAL(m128, lv_m128_cb) {
    CAUTIOUS_BACKWARD_SAFE(m128);
    union {
        u8 val8[16];
        m128 val128;
    } u;

    for (u32 i = 0; i < 16; i++) {
        if (lo <= ptr + i) {
            u.val8[i] = ptr[i];
        } else {
            u.val8[i] = 0;
        }
    }
    return u.val128;
}

MAKE_LOADVAL(m128, lv_m128_ce) {
    union {
        u8 val8[16];
        m128 val128;
    } u;

    for (u32 i = 0; i < 16; i++) {
        if ((lo <= ptr + i) && (ptr + i < hi)) {
            u.val8[i] = ptr[i];
        } else {
            u.val8[i] = 0;
        }
    }
    return u.val128;
}

#endif
