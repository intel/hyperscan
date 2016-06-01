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

#ifndef MULTIACCEL_COMMON_H_
#define MULTIACCEL_COMMON_H_

#include "config.h"
#include "ue2common.h"
#include "util/join.h"
#include "util/bitutils.h"

/*
 * When doing shifting, remember that the total number of shifts should be n-1
 */
#define VARISHIFT(src, dst, len) \
    do { \
        (dst) &= (src) >> (len); \
    } while (0)
#define STATIC_SHIFT1(x) \
    do { \
        (x) &= (x) >> 1; \
    } while (0)
#define STATIC_SHIFT2(x) \
    do { \
        (x) &= (x) >> 2;\
    } while (0)
#define STATIC_SHIFT4(x) \
    do { \
        (x) &= (x) >> 4; \
    } while (0)
#define STATIC_SHIFT8(x) \
    do { \
        (x) &= (x) >> 8; \
    } while (0)
#define SHIFT1(x) \
    do {} while (0)
#define SHIFT2(x) \
    do { \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT3(x) \
    do { \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT4(x) \
    do { \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT5(x) \
    do { \
        SHIFT4(x); \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT6(x) \
    do { \
        SHIFT4(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT7(x) \
    do { \
        SHIFT4(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT8(x) \
    do { \
        SHIFT4(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT9(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT10(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT11(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT12(x); \
    do { \
        SHIFT8(x);\
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT13(x); \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT14(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT15(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT16(x) \
    do { \
        SHIFT8(x); \
        STATIC_SHIFT8(x); \
    } while (0)
#define SHIFT17(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT18(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT19(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT20(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT21(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT22(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT23(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT24(x) \
    do { \
        SHIFT16(x); \
        STATIC_SHIFT8(x); \
    } while (0)
#define SHIFT25(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT1(x); \
    } while (0)
#define SHIFT26(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT27(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
    } while (0)
#define SHIFT28(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT29(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT30(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT31(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT1(x); \
        STATIC_SHIFT2(x); \
        STATIC_SHIFT4(x); \
    } while (0)
#define SHIFT32(x) \
    do { \
        SHIFT24(x); \
        STATIC_SHIFT8(x); \
    } while (0)

/*
 * this function is used by 32-bit multiaccel matchers. 32-bit matchers accept
 * a 32-bit integer as a buffer, where low 16 bits is movemask result and
 * high 16 bits are "don't care" values. this function is not expected to return
 * a result higher than 16.
 */
static really_inline
const u8 *match32(const u8 *buf, const u32 z) {
    if (unlikely(z != 0)) {
        u32 pos = ctz32(z);
        assert(pos < 16);
        return buf + pos;
    }
    return NULL;
}

/*
 * this function is used by 64-bit multiaccel matchers. 64-bit matchers accept
 * a 64-bit integer as a buffer, where low 32 bits is movemask result and
 * high 32 bits are "don't care" values. this function is not expected to return
 * a result higher than 32.
 */
static really_inline
const u8 *match64(const u8 *buf, const u64a z) {
    if (unlikely(z != 0)) {
        u32 pos = ctz64(z);
        assert(pos < 32);
        return buf + pos;
    }
    return NULL;
}

#endif /* MULTIACCEL_COMMON_H_ */
