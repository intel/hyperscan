/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#ifndef UTIL_ARCH_X86_CRC32_H_
#define UTIL_ARCH_X86_CRC32_H_

#include "util/arch/x86/x86.h"
#include "util/intrinsics.h"

#ifdef ARCH_64_BIT
#define CRC_WORD 8
#define CRC_TYPE u64a
#define CRC_FUNC _mm_crc32_u64
#else
#define CRC_WORD 4
#define CRC_TYPE u32
#define CRC_FUNC _mm_crc32_u32
#endif

/*
 * Use the crc32 instruction from SSE4.2 to compute our checksum - same
 * polynomial as the above function.
 */
static really_inline
u32 crc32c_sse42(u32 running_crc, const unsigned char* p_buf,
                      const size_t length) {
    u32 crc = running_crc;

    // Process byte-by-byte until p_buf is aligned

    const unsigned char *aligned_buf = ROUNDUP_PTR(p_buf, CRC_WORD);
    size_t init_bytes = aligned_buf - p_buf;
    size_t running_length = ((length - init_bytes)/CRC_WORD)*CRC_WORD;
    size_t end_bytes = length - init_bytes - running_length;

    while (p_buf < aligned_buf) {
        crc = _mm_crc32_u8(crc, *p_buf++);
    }

    // Main aligned loop, processes a word at a time.

    for (size_t li = 0; li < running_length/CRC_WORD; li++) {
        CRC_TYPE block = *(const CRC_TYPE *)p_buf;
        crc = CRC_FUNC(crc, block);
        p_buf += CRC_WORD;
    }

    // Remaining bytes

    for(size_t li = 0; li < end_bytes; li++) {
        crc = _mm_crc32_u8(crc, *p_buf++);
    }

    return crc;
}

#endif // UTIL_ARCH_X86_CRC32_H_