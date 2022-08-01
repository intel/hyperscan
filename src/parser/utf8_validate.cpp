/*
 * Copyright (c) 2015-2022, Intel Corporation
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

#include "utf8_validate.h"

#include "ue2common.h"
#include "util/unicode_def.h"

#include <cstring>

namespace ue2 {

static
bool hasValidContBytes(const u8 *s, size_t num) {
    /* continuer bytes must all be of the form 10xx xxxx */
    for (size_t i = 0; i < num; i++) {
        if ((s[i] & 0xc0) != UTF_CONT_BYTE_HEADER) {
            return false;
        }
    }
    return true;
}

static
bool isAllowedCodepoint(u32 val) {
    if (val >= 0xd800 && val <= 0xdfff) {
        return false; // High and low surrogate halves
    }
    if (val > 0x10ffff) {
        return false; // As per limit in RFC 3629
    }

    return true;
}

bool isValidUtf8(const char *expression, const size_t len) {
    if (!expression) {
        return true;
    }

    const u8 *s = (const u8 *)expression;
    u32 val;

    size_t i = 0;
    while (i < len) {
        DEBUG_PRINTF("byte %zu: 0x%02x\n", i, s[i]);
        // One octet.
        if (s[i] <= 0x7f) {
            DEBUG_PRINTF("one octet\n");
            i++;
            continue;
        }

        // Two octets.
        if ((s[i] & 0xe0) == UTF_TWO_BYTE_HEADER) {
            DEBUG_PRINTF("two octets\n");
            if (i + 2 > len) {
                break;
            }
            if (!hasValidContBytes(&s[i] + 1, 1)) {
                break;
            }
            val = ((s[i] & 0x1f) << 6) | (s[i + 1] & UTF_CONT_BYTE_VALUE_MASK);
            DEBUG_PRINTF("val=0x%x\n", val);
            if (val < 1U << 7) {
                DEBUG_PRINTF("overlong encoding\n");
                break;
            }
            if (!isAllowedCodepoint(val)) {
                DEBUG_PRINTF("codepoint not allowed\n");
                break;
            }
            i += 2;
            continue;
        }

        // Three octets.
        if ((s[i] & 0xf0) == UTF_THREE_BYTE_HEADER) {
            DEBUG_PRINTF("three octets\n");
            if (i + 3 > len) {
                break;
            }
            if (!hasValidContBytes(&s[i] + 1, 2)) {
                break;
            }
            val = ((s[i] & 0xf) << 12) |
                  ((s[i + 1] & UTF_CONT_BYTE_VALUE_MASK) << 6) |
                  (s[i + 2] & UTF_CONT_BYTE_VALUE_MASK);
            if (val < 1U << 11) {
                DEBUG_PRINTF("overlong encoding\n");
                break;
            }
            if (!isAllowedCodepoint(val)) {
                DEBUG_PRINTF("codepoint not allowed\n");
                break;
            }
            i += 3;
            continue;
        }

        // Four octets.
        if ((s[i] & 0xf8) == UTF_FOUR_BYTE_HEADER) {
            DEBUG_PRINTF("four octets\n");
            if (i + 4 > len) {
                break;
            }
            if (!hasValidContBytes(&s[i] + 1, 3)) {
                break;
            }
            val = ((s[i] & 0xf) << 18) |
                  ((s[i + 1] & UTF_CONT_BYTE_VALUE_MASK) << 12) |
                  ((s[i + 2] & UTF_CONT_BYTE_VALUE_MASK) << 6) |
                  (s[i + 3] & UTF_CONT_BYTE_VALUE_MASK);
            if (val < 1U << 16) {
                DEBUG_PRINTF("overlong encoding\n");
                break;
            }
            if (!isAllowedCodepoint(val)) {
                DEBUG_PRINTF("codepoint not allowed\n");
                break;
            }
            i += 4;
            continue;
        }

        // Something else?
        DEBUG_PRINTF("bad byte 0x%02x\n", s[i]);
        break;
    }

    DEBUG_PRINTF("i=%zu, len=%zu\n", i, len);
    return i == len;
}

} // namespace ue2
