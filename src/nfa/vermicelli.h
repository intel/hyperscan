/*
 * Copyright (c) 2015-2020, Intel Corporation
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
 * \brief Vermicelli: single-byte and double-byte acceleration.
 */

#ifndef VERMICELLI_H
#define VERMICELLI_H

#include "util/bitutils.h"
#include "util/simd_utils.h"
#include "util/unaligned.h"

#include "vermicelli_sse.h"

static really_inline
const u8 *vermicelliExec(char c, char nocase, const u8 *buf,
                         const u8 *buf_end) {
    DEBUG_PRINTF("verm scan %s\\x%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars = VERM_SET_FN(c); /* nocase already uppercase */

    // Handle small scans.
#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? vermMiniNocase(chars, buf, buf_end, 0)
                      : vermMini(chars, buf, buf_end, 0);
        if (ptr) {
            return ptr;
        }
        return buf_end;
    }
#else
    if (buf_end - buf < VERM_BOUNDARY) {
        for (; buf < buf_end; buf++) {
            char cur = (char)*buf;
            if (nocase) {
                cur &= CASE_CLEAR;
            }
            if (cur == c) {
                break;
            }
        }
        return buf;
    }
#endif

    uintptr_t min = (uintptr_t)buf % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf forward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *ptr = nocase ? vermUnalignNocase(chars, buf, 0)
                               : vermUnalign(chars, buf, 0);
        if (ptr) {
            return ptr;
        }

        buf += VERM_BOUNDARY - min;
        assert(buf < buf_end);
    }

    // Aligned loops from here on in
    const u8 *ptr = nocase ? vermSearchAlignedNocase(chars, buf, buf_end - 1, 0)
                           : vermSearchAligned(chars, buf, buf_end - 1, 0);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end
    ptr = nocase ? vermUnalignNocase(chars, buf_end - VERM_BOUNDARY, 0)
                 : vermUnalign(chars, buf_end - VERM_BOUNDARY, 0);
    return ptr ? ptr : buf_end;
}

/* like vermicelliExec except returns the address of the first character which
 * is not c */
static really_inline
const u8 *nvermicelliExec(char c, char nocase, const u8 *buf,
                         const u8 *buf_end) {
    DEBUG_PRINTF("nverm scan %s\\x%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars = VERM_SET_FN(c); /* nocase already uppercase */

    // Handle small scans.
#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? vermMiniNocase(chars, buf, buf_end, 1)
                      : vermMini(chars, buf, buf_end, 1);
        if (ptr) {
            return ptr;
        }
        return buf_end;
    }
#else
    if (buf_end - buf < VERM_BOUNDARY) {
        for (; buf < buf_end; buf++) {
            char cur = (char)*buf;
            if (nocase) {
                cur &= CASE_CLEAR;
            }
            if (cur != c) {
                break;
            }
        }
        return buf;
    }
#endif

    size_t min = (size_t)buf % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf forward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *ptr = nocase ? vermUnalignNocase(chars, buf, 1)
                               : vermUnalign(chars, buf, 1);
        if (ptr) {
            return ptr;
        }

        buf += VERM_BOUNDARY - min;
        assert(buf < buf_end);
    }

    // Aligned loops from here on in
    const u8 *ptr = nocase ? vermSearchAlignedNocase(chars, buf, buf_end - 1, 1)
                           : vermSearchAligned(chars, buf, buf_end - 1, 1);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end
    ptr = nocase ? vermUnalignNocase(chars, buf_end - VERM_BOUNDARY, 1)
                 : vermUnalign(chars, buf_end - VERM_BOUNDARY, 1);
    return ptr ? ptr : buf_end;
}

static really_inline
const u8 *vermicelliDoubleExec(char c1, char c2, char nocase, const u8 *buf,
                               const u8 *buf_end) {
    DEBUG_PRINTF("double verm scan %s\\x%02hhx%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c1, c2, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars1 = VERM_SET_FN(c1); /* nocase already uppercase */
    VERM_TYPE chars2 = VERM_SET_FN(c2); /* nocase already uppercase */

#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? dvermMiniNocase(chars1, chars2, buf, buf_end)
                      : dvermMini(chars1, chars2, buf, buf_end);
        if (ptr) {
            return ptr;
        }

        /* check for partial match at end */
        u8 mask = nocase ? CASE_CLEAR : 0xff;
        if ((buf_end[-1] & mask) == (u8)c1) {
            DEBUG_PRINTF("partial!!!\n");
            return buf_end - 1;
        }

        return buf_end;
    }
#endif

    assert((buf_end - buf) >= VERM_BOUNDARY);
    uintptr_t min = (uintptr_t)buf % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf forward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *ptr = nocase
                        ? dvermPreconditionNocase(chars1, chars2, buf)
                        : dvermPrecondition(chars1, chars2, buf);
        if (ptr) {
            return ptr;
        }

        buf += VERM_BOUNDARY - min;
        assert(buf < buf_end);
    }

    // Aligned loops from here on in
    const u8 *ptr = nocase ? dvermSearchAlignedNocase(chars1, chars2, c1, c2,
                                                      buf, buf_end)
                           : dvermSearchAligned(chars1, chars2, c1, c2, buf,
                                                buf_end);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end
    ptr = nocase ? dvermPreconditionNocase(chars1, chars2,
                                           buf_end - VERM_BOUNDARY)
                 : dvermPrecondition(chars1, chars2, buf_end - VERM_BOUNDARY);

    if (ptr) {
        return ptr;
    }

    /* check for partial match at end */
    u8 mask = nocase ? CASE_CLEAR : 0xff;
    if ((buf_end[-1] & mask) == (u8)c1) {
        DEBUG_PRINTF("partial!!!\n");
        return buf_end - 1;
    }

    return buf_end;
}

static really_inline
const u8 *vermicelliDoubleMaskedExec(char c1, char c2, char m1, char m2,
                                     const u8 *buf, const u8 *buf_end) {
    DEBUG_PRINTF("double verm scan (\\x%02hhx&\\x%02hhx)(\\x%02hhx&\\x%02hhx) "
                 "over %zu bytes\n", c1, m1, c2, m2, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars1 = VERM_SET_FN(c1);
    VERM_TYPE chars2 = VERM_SET_FN(c2);
    VERM_TYPE mask1 = VERM_SET_FN(m1);
    VERM_TYPE mask2 = VERM_SET_FN(m2);

#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = dvermMiniMasked(chars1, chars2, mask1, mask2, buf,
                                        buf_end);
        if (ptr) {
            return ptr;
        }

        /* check for partial match at end */
        if ((buf_end[-1] & m1) == (u8)c1) {
            DEBUG_PRINTF("partial!!!\n");
            return buf_end - 1;
        }

        return buf_end;
    }
#endif

    assert((buf_end - buf) >= VERM_BOUNDARY);
    uintptr_t min = (uintptr_t)buf % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf forward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *p = dvermPreconditionMasked(chars1, chars2, mask1, mask2, buf);
        if (p) {
            return p;
        }

        buf += VERM_BOUNDARY - min;
        assert(buf < buf_end);
    }

    // Aligned loops from here on in
    const u8 *ptr = dvermSearchAlignedMasked(chars1, chars2, mask1, mask2, c1,
                                             c2, m1, m2, buf, buf_end);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end
    ptr = dvermPreconditionMasked(chars1, chars2, mask1, mask2,
                                  buf_end - VERM_BOUNDARY);

    if (ptr) {
        return ptr;
    }

    /* check for partial match at end */
    if ((buf_end[-1] & m1) == (u8)c1) {
        DEBUG_PRINTF("partial!!!\n");
        return buf_end - 1;
    }

    return buf_end;
}

// Reverse vermicelli scan. Provides exact semantics and returns (buf - 1) if
// character not found.
static really_inline
const u8 *rvermicelliExec(char c, char nocase, const u8 *buf,
                          const u8 *buf_end) {
    DEBUG_PRINTF("rev verm scan %s\\x%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars = VERM_SET_FN(c); /* nocase already uppercase */

    // Handle small scans.
#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? rvermMiniNocase(chars, buf, buf_end, 0)
                      : rvermMini(chars, buf, buf_end, 0);
        if (ptr) {
            return ptr;
        }
        return buf - 1;
    }
#else
    if (buf_end - buf < VERM_BOUNDARY) {
        for (buf_end--; buf_end >= buf; buf_end--) {
            char cur = (char)*buf_end;
            if (nocase) {
                cur &= CASE_CLEAR;
            }
            if (cur == c) {
                break;
            }
        }
        return buf_end;
    }
#endif

    size_t min = (size_t)buf_end % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf backward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *ptr = nocase ? rvermUnalignNocase(chars,
                                                    buf_end - VERM_BOUNDARY,
                                                    0)
                               : rvermUnalign(chars, buf_end - VERM_BOUNDARY,
                                              0);

        if (ptr) {
            return ptr;
        }

        buf_end -= min;
        if (buf >= buf_end) {
            return buf_end;
        }
    }

    // Aligned loops from here on in.
    const u8 *ptr = nocase ? rvermSearchAlignedNocase(chars, buf, buf_end, 0)
                           : rvermSearchAligned(chars, buf, buf_end, 0);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end, return buf - 1 if not found.
    ptr = nocase ? rvermUnalignNocase(chars, buf, 0)
                 : rvermUnalign(chars, buf, 0);
    return ptr ? ptr : buf - 1;
}

/* like rvermicelliExec except returns the address of the last character which
 * is not c */
static really_inline
const u8 *rnvermicelliExec(char c, char nocase, const u8 *buf,
                           const u8 *buf_end) {
    DEBUG_PRINTF("rev verm scan %s\\x%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars = VERM_SET_FN(c); /* nocase already uppercase */

    // Handle small scans.
#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? rvermMiniNocase(chars, buf, buf_end, 1)
                      : rvermMini(chars, buf, buf_end, 1);
        if (ptr) {
            return ptr;
        }
        return buf - 1;
    }
#else
    if (buf_end - buf < VERM_BOUNDARY) {
        for (buf_end--; buf_end >= buf; buf_end--) {
            char cur = (char)*buf_end;
            if (nocase) {
                cur &= CASE_CLEAR;
            }
            if (cur != c) {
                break;
            }
        }
        return buf_end;
    }
#endif

    size_t min = (size_t)buf_end % VERM_BOUNDARY;
    if (min) {
        // Input isn't aligned, so we need to run one iteration with an
        // unaligned load, then skip buf backward to the next aligned address.
        // There's some small overlap here, but we don't mind scanning it twice
        // if we can do it quickly, do we?
        const u8 *ptr = nocase ? rvermUnalignNocase(chars,
                                                    buf_end - VERM_BOUNDARY,
                                                    1)
                               : rvermUnalign(chars, buf_end - VERM_BOUNDARY,
                                              1);

        if (ptr) {
            return ptr;
        }

        buf_end -= min;
        if (buf >= buf_end) {
            return buf_end;
        }
    }

    // Aligned loops from here on in.
    const u8 *ptr = nocase ? rvermSearchAlignedNocase(chars, buf, buf_end, 1)
                           : rvermSearchAligned(chars, buf, buf_end, 1);
    if (ptr) {
        return ptr;
    }

    // Tidy up the mess at the end, return buf - 1 if not found.
    ptr = nocase ? rvermUnalignNocase(chars, buf, 1)
                 : rvermUnalign(chars, buf, 1);
    return ptr ? ptr : buf - 1;
}

/* returns highest offset of c2 (NOTE: not c1) */
static really_inline
const u8 *rvermicelliDoubleExec(char c1, char c2, char nocase, const u8 *buf,
                                const u8 *buf_end) {
    DEBUG_PRINTF("rev double verm scan %s\\x%02hhx%02hhx over %zu bytes\n",
                 nocase ? "nocase " : "", c1, c2, (size_t)(buf_end - buf));
    assert(buf < buf_end);

    VERM_TYPE chars1 = VERM_SET_FN(c1); /* nocase already uppercase */
    VERM_TYPE chars2 = VERM_SET_FN(c2); /* nocase already uppercase */

#ifdef HAVE_AVX512
    if (buf_end - buf <= VERM_BOUNDARY) {
        const u8 *ptr = nocase
                      ? rdvermMiniNocase(chars1, chars2, buf, buf_end)
                      : rdvermMini(chars1, chars2, buf, buf_end);

        if (ptr) {
            return ptr;
        }

        // check for partial match at end ???
        return buf - 1;
    }
#endif

    assert((buf_end - buf) >= VERM_BOUNDARY);
    size_t min = (size_t)buf_end % VERM_BOUNDARY;
    if (min) {
        // input not aligned, so we need to run one iteration with an unaligned
        // load, then skip buf forward to the next aligned address. There's
        // some small overlap here, but we don't mind scanning it twice if we
        // can do it quickly, do we?
        const u8 *ptr = nocase ? rdvermPreconditionNocase(chars1, chars2,
                                                          buf_end - VERM_BOUNDARY)
                               : rdvermPrecondition(chars1, chars2,
                                                    buf_end - VERM_BOUNDARY);

        if (ptr) {
            return ptr;
        }

        buf_end -= min;
        if (buf >= buf_end) {
            return buf_end;
        }
    }

    // Aligned loops from here on in
    if (nocase) {
        return rdvermSearchAlignedNocase(chars1, chars2, c1, c2, buf, buf_end);
    } else {
        return rdvermSearchAligned(chars1, chars2, c1, c2, buf, buf_end);
    }
}

#endif /* VERMICELLI_H */
