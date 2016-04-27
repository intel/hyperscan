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

#include "ue2common.h"
#include "util/unaligned.h"

#define MAKE_LOADVAL(type, name)                \
    static really_inline                                                \
    type name(const u8 *ptr, UNUSED const u8 *lo, UNUSED const u8 *hi)

#define NORMAL_SAFE(type)                                               \
    do {                                                                \
        assert(ptr >= lo);                                              \
        assert(ptr + sizeof(type) - 1 < hi);                            \
    } while(0)

#define MAKE_LOOP_CE(TYPE)                                              \
    TYPE v = 0;                                                         \
    for (TYPE i = 0; i < sizeof(TYPE); i++) {                           \
        if ((lo <= ptr + i) && (ptr + i < hi)) {                        \
            v += (TYPE)ptr[i] << (i*8);                                 \
        }                                                               \
    }                                                                   \
    return v;

// no suffix = normal (unaligned)
// _ce       = cautious everywhere (in both directions); test against hi and lo

MAKE_LOADVAL(u16, lv_u16) {
    NORMAL_SAFE(u16);
    return unaligned_load_u16(ptr);
}

MAKE_LOADVAL(u64a, lv_u64a) {
    NORMAL_SAFE(u32);
    return unaligned_load_u64a(ptr);
}

MAKE_LOADVAL(u16, lv_u16_ce) { MAKE_LOOP_CE(u16); }

MAKE_LOADVAL(u64a, lv_u64a_ce) { MAKE_LOOP_CE(u64a); }

#endif
