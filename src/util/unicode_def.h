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

#ifndef UNICODE_DEF_H
#define UNICODE_DEF_H

#include "ue2common.h"

#define MAX_UNICODE 0x10FFFF
#define INVALID_UNICODE 0xffffffff /* unicode could never go above 2^31 */

#define UTF_2CHAR_MIN (1U << 7)
#define UTF_3CHAR_MIN (1U << 11)
#define UTF_4CHAR_MIN (1U << 16)
#define UTF_CONT_SHIFT 6
#define UTF_CONT_BYTE_RANGE (1U << UTF_CONT_SHIFT)
#define UTF_CONT_BYTE_HEADER  ((u8)0x80) /* 10xx xxxx */
#define UTF_TWO_BYTE_HEADER   ((u8)0xc0) /* 110x xxxx */
#define UTF_THREE_BYTE_HEADER ((u8)0xe0) /* 1110 xxxx */
#define UTF_FOUR_BYTE_HEADER  ((u8)0xf0) /* 1111 0xxx */

#define UTF_CONT_BYTE_VALUE_MASK 0x3f

#define UTF_CONT_MIN UTF_CONT_BYTE_HEADER
#define UTF_CONT_MAX (UTF_TWO_BYTE_HEADER - 1)

#define UTF_TWO_BYTE_MIN UTF_TWO_BYTE_HEADER
#define UTF_TWO_BYTE_MAX (UTF_THREE_BYTE_HEADER - 1)

#define UTF_THREE_BYTE_MIN UTF_THREE_BYTE_HEADER
#define UTF_THREE_BYTE_MAX (UTF_FOUR_BYTE_HEADER - 1)

#define UTF_FOUR_BYTE_MIN UTF_FOUR_BYTE_HEADER
#define UTF_FOUR_BYTE_MAX ((u8)0xf4)

#define UTF_CONT_CR CharReach(UTF_CONT_MIN, UTF_CONT_MAX)
#define UTF_ASCII_CR CharReach(0, 127)
#define UTF_START_CR CharReach(UTF_TWO_BYTE_MIN, UTF_FOUR_BYTE_MAX)
#define UTF_TWO_START_CR CharReach(UTF_TWO_BYTE_MIN, UTF_TWO_BYTE_MAX)
#define UTF_THREE_START_CR CharReach(UTF_THREE_BYTE_MIN, UTF_THREE_BYTE_MAX)
#define UTF_FOUR_START_CR CharReach(UTF_FOUR_BYTE_MIN, UTF_FOUR_BYTE_MAX)

#define UNICODE_SURROGATE_MIN 0xd800
#define UNICODE_SURROGATE_MAX 0xdfff

#ifdef __cplusplus

namespace ue2 {
typedef u32 unichar; /* represents a unicode code point */

static UNUSED
u8 makeContByte(u8 val) {
    return UTF_CONT_BYTE_HEADER | (val & UTF_CONT_BYTE_VALUE_MASK);
}

} // namespace

#endif // __cplusplus

#endif
