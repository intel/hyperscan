/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef COPY_BYTES_H
#define COPY_BYTES_H

#include "unaligned.h"
#include "simd_utils.h"

static really_inline
void copy_upto_64_bytes(u8 *dst, const u8 *src, unsigned int len) {
    switch (len) {
    case 0:
        break;
    case 1:
        *dst = *src;
        break;
    case 2:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        break;
    case 3:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        dst[2] = src[2];
        break;
    case 4:
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 5:
    case 6:
    case 7:
        unaligned_store_u32(dst + len - 4, unaligned_load_u32(src + len - 4));
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 8:
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        unaligned_store_u64a(dst + len - 8, unaligned_load_u64a(src + len - 8));
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    case 16:
        storeu128(dst, loadu128(src));
        break;
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
    case 28:
    case 29:
    case 30:
    case 31:
        storeu128(dst + len - 16, loadu128(src + len - 16));
        storeu128(dst, loadu128(src));
        break;
    case 32:
        storeu256(dst, loadu256(src));
        break;
#ifdef HAVE_AVX512
    case 64:
        storebytes512(dst, loadu512(src), 64);
        break;
    default:
        assert(len < 64);
        u64a k = (1ULL << len) - 1;
        storeu_mask_m512(dst, k, loadu_maskz_m512(k, src));
        break;
#else
    default:
        assert(0);
        break;
#endif
    }
}

#endif
