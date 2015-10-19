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

#include "vermicelli.h"

static really_inline
const u8 *find_xverm_run(char c, char nocase, u32 repeat, UNUSED const u8 *buf,
                         const u8 *buf_start, const u8 *buf_end, char negate) {
    DEBUG_PRINTF("looking for 0x%hhx{%u} in %p [%zd, %zd)\n", c, repeat, buf,
                  buf_start - buf, buf_end - buf);

    /* TODO optimise on where it is easy to get a dense bitfield of character
     * matches */
    if (repeat == 1) {
        return negate ? nvermicelliExec(c, nocase, buf_start, buf_end)
                      : vermicelliExec(c, nocase, buf_start, buf_end);
    }

    while (1) {
        const u8 *s;
        if (negate) {
            s = nvermicelliExec(c, nocase, buf_start, buf_end);
        } else if (buf_end - buf_start >= VERM_BOUNDARY && !nocase) {
            s = vermicelliDoubleExec(c, c, nocase, buf_start, buf_end);

            if (s != buf_end && *s != c) { /* double verm is not certain to be
                                            * precise */
                s = vermicelliExec(c, nocase, s, buf_end);
            }
        } else {
            s = vermicelliExec(c, nocase, buf_start, buf_end);
        }
        if (s == buf_end) {
            return s;
        }

        DEBUG_PRINTF("cand %zd\n", s - buf);

        const u8 *test_e = MIN(s + repeat, buf_end);

        const u8 *rv = negate ? vermicelliExec(c, nocase, s, test_e)
                              : nvermicelliExec(c, nocase, s, test_e);

        assert(rv > buf_start);
        assert(rv <= buf_end);

        if (rv == test_e) {
            return s;
        }

        buf_start = rv;
    }
}

static really_inline
const u8 *find_verm_run(char c, char nocase, u32 repeat, const u8 *buf,
                        const u8 *buf_start, const u8 *buf_end) {
    return find_xverm_run(c, nocase, repeat, buf, buf_start, buf_end, 0);
}

static really_inline
const u8 *find_nverm_run(char c, char nocase, u32 repeat, const u8 *buf,
                         const u8 *buf_start, const u8 *buf_end) {
    return find_xverm_run(c, nocase, repeat, buf, buf_start, buf_end, 1);
}
