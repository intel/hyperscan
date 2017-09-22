/*
 * Copyright (c) 2017, Intel Corporation
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

/* noodle scan parts for AVX512 */

static really_inline
m512 getMask(u8 c, bool noCase) {
    u8 k = caseClear8(c, noCase);
    return set64x8(k);
}

static really_inline
m512 getCaseMask(void) {
    return set64x8(CASE_CLEAR);
}

// The short scan routine. It is used both to scan data up to an
// alignment boundary if needed and to finish off data that the aligned scan
// function can't handle (due to small/unaligned chunk at end)
static really_inline
hwlm_error_t scanSingleShort(const struct noodTable *n, const u8 *buf,
                             size_t len, bool noCase, m512 caseMask, m512 mask1,
                             const struct cb_info *cbi, size_t start,
                             size_t end) {
    const u8 *d = buf + start;
    ptrdiff_t scan_len = end - start;
    DEBUG_PRINTF("scan_len %zu\n", scan_len);
    assert(scan_len <= 64);
    if (!scan_len) {
        return HWLM_SUCCESS;
    }

    __mmask64 k = (~0ULL) >> (64 - scan_len);
    DEBUG_PRINTF("load mask 0x%016llx\n", k);

    m512 v = loadu_maskz_m512(k, d);

    if (noCase) {
        v = and512(v, caseMask);
    }

    // reuse the load mask to indicate valid bytes
    u64a z = masked_eq512mask(k, mask1, v);

    SINGLE_ZSCAN();

    return HWLM_SUCCESS;
}

static really_inline
hwlm_error_t scanSingle512(const struct noodTable *n, const u8 *buf, size_t len,
                           bool noCase, m512 caseMask, m512 mask1,
                           const struct cb_info *cbi, size_t start,
                           size_t end) {
    const u8 *d = buf + start;
    const u8 *e = buf + end;
    DEBUG_PRINTF("start %p end %p \n", d, e);
    assert(d < e);
    if (d + 64 >= e) {
        goto tail;
    }

    // peel off first part to cacheline boundary
    const u8 *d1 = ROUNDUP_PTR(d, 64);
    if (scanSingleShort(n, buf, len, noCase, caseMask, mask1, cbi, start,
                        d1 - buf) == HWLM_TERMINATED) {
        return HWLM_TERMINATED;
    }
    d = d1;

    for (; d + 64 < e; d += 64) {
        DEBUG_PRINTF("d %p e %p \n", d, e);
        m512 v = noCase ? and512(load512(d), caseMask) : load512(d);

        u64a z = eq512mask(mask1, v);
        __builtin_prefetch(d + 128);

        SINGLE_ZSCAN();
    }

tail:
    DEBUG_PRINTF("d %p e %p \n", d, e);
    // finish off tail

    return scanSingleShort(n, buf, len, noCase, caseMask, mask1, cbi, d - buf,
                           e - buf);
}

static really_inline
hwlm_error_t scanDoubleShort(const struct noodTable *n, const u8 *buf,
                             size_t len, bool noCase, m512 caseMask, m512 mask1,
                             m512 mask2, const struct cb_info *cbi,
                             u64a *lastz0, size_t start, size_t end) {
    DEBUG_PRINTF("start %zu end %zu last 0x%016llx\n", start, end, *lastz0);
    const u8 *d = buf + start;
    ptrdiff_t scan_len = end - start;
    if (!scan_len) {
        return HWLM_SUCCESS;
    }
    assert(scan_len <= 64);
    __mmask64 k = (~0ULL) >> (64 - scan_len);
    DEBUG_PRINTF("load mask 0x%016llx scan_len %zu\n", k, scan_len);

    m512 v = loadu_maskz_m512(k, d);
    if (noCase) {
        v = and512(v, caseMask);
    }

    u64a z0 = masked_eq512mask(k, mask1, v);
    u64a z1 = masked_eq512mask(k, mask2, v);
    u64a z = (*lastz0 | (z0 << 1)) & z1;
    DEBUG_PRINTF("z 0x%016llx\n", z);

    DOUBLE_ZSCAN();
    *lastz0 = z0 >> (scan_len - 1);
    return HWLM_SUCCESS;
}

static really_inline
hwlm_error_t scanDouble512(const struct noodTable *n, const u8 *buf, size_t len,
                           bool noCase, m512 caseMask, m512 mask1, m512 mask2,
                           const struct cb_info *cbi, size_t start,
                           size_t end) {
    const u8 *d = buf + start;
    const u8 *e = buf + end;
    u64a lastz0 = 0;
    DEBUG_PRINTF("start %zu end %zu \n", start, end);
    assert(d < e);
    if (d + 64 >= e) {
        goto tail;
    }

    // peel off first part to cacheline boundary
    const u8 *d1 = ROUNDUP_PTR(d, 64);
    if (scanDoubleShort(n, buf, len, noCase, caseMask, mask1, mask2, cbi,
                        &lastz0, start, d1 - buf) == HWLM_TERMINATED) {
        return HWLM_TERMINATED;
    }
    d = d1;

    for (; d + 64 < e; d += 64) {
        DEBUG_PRINTF("d %p e %p 0x%016llx\n", d, e, lastz0);
        m512 v = noCase ? and512(load512(d), caseMask) : load512(d);

        /* we have to pull the masks out of the AVX registers because we can't
           byte shift between the lanes */
        u64a z0 = eq512mask(mask1, v);
        u64a z1 = eq512mask(mask2, v);
        u64a z = (lastz0 | (z0 << 1)) & z1;
        lastz0 = z0 >> 63;

        // On large packet buffers, this prefetch appears to get us about 2%.
        __builtin_prefetch(d + 256);

        DEBUG_PRINTF("z 0x%016llx\n", z);

        DOUBLE_ZSCAN();
    }

tail:
    DEBUG_PRINTF("d %p e %p off %zu \n", d, e, d - buf);
    // finish off tail

    return scanDoubleShort(n, buf, len, noCase, caseMask, mask1, mask2, cbi,
                           &lastz0, d - buf, end);
}
