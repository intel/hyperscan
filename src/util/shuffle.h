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

/** \file
 * \brief Naive dynamic shuffles.
 *
 * These are written with the assumption that the provided masks are sparsely
 * populated and never contain more than 32 on bits. Other implementations will
 * be faster and actually correct if these assumptions don't hold true.
 */

#ifndef SHUFFLE_H
#define SHUFFLE_H

#include "config.h"
#include "bitutils.h"
#include "simd_utils.h"
#include "ue2common.h"

#if defined(__BMI2__) || (defined(_WIN32) && defined(__AVX2__))
#define HAVE_PEXT
#endif

static really_inline
u32 shuffleDynamic32(u32 x, u32 mask) {
#if defined(HAVE_PEXT)
    // Intel BMI2 can do this operation in one instruction.
    return _pext_u32(x, mask);
#else

    u32 result = 0, num = 1;
    while (mask != 0) {
        u32 bit = findAndClearLSB_32(&mask);
        if (x & (1U << bit)) {
            assert(num != 0); // more than 32 bits!
            result |= num;
        }
        num <<= 1;
    }
    return result;
#endif
}

static really_inline
u32 shuffleDynamic64(u64a x, u64a mask) {
#if defined(HAVE_PEXT) && defined(ARCH_64_BIT)
    // Intel BMI2 can do this operation in one instruction.
    return _pext_u64(x, mask);
#else

    u32 result = 0, num = 1;
    while (mask != 0) {
        u32 bit = findAndClearLSB_64(&mask);
        if (x & (1ULL << bit)) {
            assert(num != 0); // more than 32 bits!
            result |= num;
        }
        num <<= 1;
    }
    return result;
#endif
}

#undef HAVE_PEXT

#endif // SHUFFLE_H
