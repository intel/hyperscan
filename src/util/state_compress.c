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

/** \file
 * \brief Mask-based state compression, used by the NFA.
 */
#include "config.h"
#include "ue2common.h"
#include "arch.h"
#include "bitutils.h"
#include "unaligned.h"
#include "pack_bits.h"
#include "partial_store.h"
#include "popcount.h"
#include "state_compress.h"

#include <string.h>

/*
 * 32-bit store/load.
 */

void storecompressed32(void *ptr, const u32 *x, const u32 *m, u32 bytes) {
    assert(popcount32(*m) <= bytes * 8);

    u32 v = compress32(*x, *m);
    partial_store_u32(ptr, v, bytes);
}

void loadcompressed32(u32 *x, const void *ptr, const u32 *m, u32 bytes) {
    assert(popcount32(*m) <= bytes * 8);

    u32 v = partial_load_u32(ptr, bytes);
    *x = expand32(v, *m);
}

/*
 * 64-bit store/load.
 */

void storecompressed64(void *ptr, const u64a *x, const u64a *m, u32 bytes) {
    assert(popcount64(*m) <= bytes * 8);

    u64a v = compress64(*x, *m);
    partial_store_u64a(ptr, v, bytes);
}

void loadcompressed64(u64a *x, const void *ptr, const u64a *m, u32 bytes) {
    assert(popcount64(*m) <= bytes * 8);

    u64a v = partial_load_u64a(ptr, bytes);
    *x = expand64(v, *m);
}

/*
 * 128-bit store/load.
 */

#if defined(ARCH_32_BIT)
static really_inline
void storecompressed128_32bit(void *ptr, m128 xvec, m128 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 x[4];
    memcpy(x, &xvec, sizeof(xvec));
    u32 m[4];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[4] = { popcount32(m[0]), popcount32(m[1]),
                    popcount32(m[2]), popcount32(m[3]) };

    // Compress each 32-bit chunk individually.
    u32 v[4] = { compress32(x[0], m[0]), compress32(x[1], m[1]),
                 compress32(x[2], m[2]), compress32(x[3], m[3]) };

    // Write packed data out.
    pack_bits_32(ptr, v, bits, 4);
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
void storecompressed128_64bit(void *ptr, m128 xvec, m128 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a x[2];
    memcpy(x, &xvec, sizeof(xvec));
    u64a m[2];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[2] = { popcount64(m[0]), popcount64(m[1]) };

    // Compress each 64-bit chunk individually.
    u64a v[2] = { compress64(x[0], m[0]), compress64(x[1], m[1]) };

    // Write packed data out.
    pack_bits_64(ptr, v, bits, 2);
}
#endif

void storecompressed128(void *ptr, const m128 *x, const m128 *m,
                        UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    storecompressed128_64bit(ptr, *x, *m);
#else
    storecompressed128_32bit(ptr, *x, *m);
#endif
}

#if defined(ARCH_32_BIT)
static really_inline
m128 loadcompressed128_32bit(const void *ptr, m128 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 m[8];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[4] = { popcount32(m[0]), popcount32(m[1]),
                    popcount32(m[2]), popcount32(m[3]) };
    u32 v[4];

    unpack_bits_32(v, (const u8 *)ptr, bits, 4);

    u32 x[4] = { expand32(v[0], m[0]), expand32(v[1], m[1]),
                 expand32(v[2], m[2]), expand32(v[3], m[3]) };

    return _mm_set_epi32(x[3], x[2], x[1], x[0]);
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
m128 loadcompressed128_64bit(const void *ptr, m128 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a m[2] = { movq(mvec), movq(_mm_srli_si128(mvec, 8)) };

    u32 bits[2] = { popcount64(m[0]), popcount64(m[1]) };
    u64a v[2];

    unpack_bits_64(v, (const u8 *)ptr, bits, 2);

    u64a x[2] = { expand64(v[0], m[0]), expand64(v[1], m[1]) };

    return _mm_set_epi64x(x[1], x[0]);
}
#endif

void loadcompressed128(m128 *x, const void *ptr, const m128 *m,
                       UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    *x = loadcompressed128_64bit(ptr, *m);
#else
    *x = loadcompressed128_32bit(ptr, *m);
#endif
}

/*
 * 256-bit store/load.
 */

#if defined(ARCH_32_BIT)
static really_inline
void storecompressed256_32bit(void *ptr, m256 xvec, m256 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 x[8];
    memcpy(x, &xvec, sizeof(xvec));
    u32 m[8];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[8] = { popcount32(m[0]), popcount32(m[1]),
                    popcount32(m[2]), popcount32(m[3]),
                    popcount32(m[4]), popcount32(m[5]),
                    popcount32(m[6]), popcount32(m[7])};

    // Compress each 32-bit chunk individually.
    u32 v[8] = { compress32(x[0], m[0]), compress32(x[1], m[1]),
                 compress32(x[2], m[2]), compress32(x[3], m[3]),
                 compress32(x[4], m[4]), compress32(x[5], m[5]),
                 compress32(x[6], m[6]), compress32(x[7], m[7]) };

    // Write packed data out.
    pack_bits_32(ptr, v, bits, 8);
}
#endif

#if defined(ARCH_64_BIT)
static really_really_inline
void storecompressed256_64bit(void *ptr, m256 xvec, m256 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a x[4];
    memcpy(x, &xvec, sizeof(xvec));
    u64a m[4];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[4] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]) };

    // Compress each 64-bit chunk individually.
    u64a v[4] = { compress64(x[0], m[0]), compress64(x[1], m[1]),
                  compress64(x[2], m[2]), compress64(x[3], m[3]) };

    // Write packed data out.
    pack_bits_64(ptr, v, bits, 4);
}
#endif

void storecompressed256(void *ptr, const m256 *x, const m256 *m,
                        UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    storecompressed256_64bit(ptr, *x, *m);
#else
    storecompressed256_32bit(ptr, *x, *m);
#endif
}

#if defined(ARCH_32_BIT)
static really_inline
m256 loadcompressed256_32bit(const void *ptr, m256 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 m[8];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[8] = { popcount32(m[0]), popcount32(m[1]),
                    popcount32(m[2]), popcount32(m[3]),
                    popcount32(m[4]), popcount32(m[5]),
                    popcount32(m[6]), popcount32(m[7])};
    u32 v[8];

    unpack_bits_32(v, (const u8 *)ptr, bits, 8);

    u32 x[8] = { expand32(v[0], m[0]), expand32(v[1], m[1]),
                 expand32(v[2], m[2]), expand32(v[3], m[3]),
                 expand32(v[4], m[4]), expand32(v[5], m[5]),
                 expand32(v[6], m[6]), expand32(v[7], m[7]) };

#if !defined(HAVE_AVX2)
    m256 xvec = { .lo = _mm_set_epi32(x[3], x[2], x[1], x[0]),
                  .hi = _mm_set_epi32(x[7], x[6], x[5], x[4]) };
#else
    m256 xvec = _mm256_set_epi32(x[7], x[6], x[5], x[4],
                                 x[3], x[2], x[1], x[0]);
#endif
    return xvec;
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
m256 loadcompressed256_64bit(const void *ptr, m256 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a m[4];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[4] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]) };
    u64a v[4];

    unpack_bits_64(v, (const u8 *)ptr, bits, 4);

    u64a x[4] = { expand64(v[0], m[0]), expand64(v[1], m[1]),
                  expand64(v[2], m[2]), expand64(v[3], m[3]) };

#if !defined(HAVE_AVX2)
    m256 xvec = { .lo = _mm_set_epi64x(x[1], x[0]),
                  .hi = _mm_set_epi64x(x[3], x[2]) };
#else
    m256 xvec = _mm256_set_epi64x(x[3], x[2], x[1], x[0]);
#endif
    return xvec;
}
#endif

void loadcompressed256(m256 *x, const void *ptr, const m256 *m,
                       UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    *x = loadcompressed256_64bit(ptr, *m);
#else
    *x = loadcompressed256_32bit(ptr, *m);
#endif
}

/*
 * 384-bit store/load.
 */

#if defined(ARCH_32_BIT)
static really_inline
void storecompressed384_32bit(void *ptr, m384 xvec, m384 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 x[12];
    memcpy(x, &xvec, sizeof(xvec));
    u32 m[12];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[12] = { popcount32(m[0]), popcount32(m[1]),
                     popcount32(m[2]), popcount32(m[3]),
                     popcount32(m[4]), popcount32(m[5]),
                     popcount32(m[6]), popcount32(m[7]),
                     popcount32(m[8]), popcount32(m[9]),
                     popcount32(m[10]), popcount32(m[11]) };

    // Compress each 32-bit chunk individually.
    u32 v[12] = { compress32(x[0], m[0]), compress32(x[1], m[1]),
                  compress32(x[2], m[2]), compress32(x[3], m[3]),
                  compress32(x[4], m[4]), compress32(x[5], m[5]),
                  compress32(x[6], m[6]), compress32(x[7], m[7]),
                  compress32(x[8], m[8]), compress32(x[9], m[9]),
                  compress32(x[10], m[10]), compress32(x[11], m[11])};

    // Write packed data out.
    pack_bits_32(ptr, v, bits, 12);
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
void storecompressed384_64bit(void *ptr, m384 xvec, m384 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a x[6];
    memcpy(x, &xvec, sizeof(xvec));
    u64a m[6];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[6] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]),
                    popcount64(m[4]), popcount64(m[5]) };

    // Compress each 64-bit chunk individually.
    u64a v[6] = { compress64(x[0], m[0]), compress64(x[1], m[1]),
                  compress64(x[2], m[2]), compress64(x[3], m[3]),
                  compress64(x[4], m[4]), compress64(x[5], m[5]) };

    // Write packed data out.
    pack_bits_64(ptr, v, bits, 6);
}
#endif

void storecompressed384(void *ptr, const m384 *x, const m384 *m,
                        UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    storecompressed384_64bit(ptr, *x, *m);
#else
    storecompressed384_32bit(ptr, *x, *m);
#endif
}

#if defined(ARCH_32_BIT)
static really_inline
m384 loadcompressed384_32bit(const void *ptr, m384 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 m[12];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[12] = { popcount32(m[0]), popcount32(m[1]),
                     popcount32(m[2]), popcount32(m[3]),
                     popcount32(m[4]), popcount32(m[5]),
                     popcount32(m[6]), popcount32(m[7]),
                     popcount32(m[8]), popcount32(m[9]),
                     popcount32(m[10]), popcount32(m[11]) };
    u32 v[12];

    unpack_bits_32(v, (const u8 *)ptr, bits, 12);

    u32 x[12] = { expand32(v[0], m[0]), expand32(v[1], m[1]),
                  expand32(v[2], m[2]), expand32(v[3], m[3]),
                  expand32(v[4], m[4]), expand32(v[5], m[5]),
                  expand32(v[6], m[6]), expand32(v[7], m[7]),
                  expand32(v[8], m[8]), expand32(v[9], m[9]),
                  expand32(v[10], m[10]), expand32(v[11], m[11]) };

    m384 xvec = { .lo = _mm_set_epi32(x[3], x[2], x[1], x[0]),
                  .mid = _mm_set_epi32(x[7], x[6], x[5], x[4]),
                  .hi = _mm_set_epi32(x[11], x[10], x[9], x[8]) };
    return xvec;
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
m384 loadcompressed384_64bit(const void *ptr, m384 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a m[6];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[6] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]),
                    popcount64(m[4]), popcount64(m[5]) };
    u64a v[6];

    unpack_bits_64(v, (const u8 *)ptr, bits, 6);

    u64a x[6] = { expand64(v[0], m[0]), expand64(v[1], m[1]),
                  expand64(v[2], m[2]), expand64(v[3], m[3]),
                  expand64(v[4], m[4]), expand64(v[5], m[5]) };

    m384 xvec = { .lo = _mm_set_epi64x(x[1], x[0]),
                  .mid = _mm_set_epi64x(x[3], x[2]),
                  .hi = _mm_set_epi64x(x[5], x[4]) };
    return xvec;
}
#endif

void loadcompressed384(m384 *x, const void *ptr, const m384 *m,
                       UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    *x = loadcompressed384_64bit(ptr, *m);
#else
    *x = loadcompressed384_32bit(ptr, *m);
#endif
}

/*
 * 512-bit store/load.
 */

#if defined(ARCH_32_BIT)
static really_inline
void storecompressed512_32bit(void *ptr, m512 xvec, m512 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 x[16];
    memcpy(x, &xvec, sizeof(xvec));
    u32 m[16];
    memcpy(m, &mvec, sizeof(mvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[16] = { popcount32(m[0]), popcount32(m[1]),
                     popcount32(m[2]), popcount32(m[3]),
                     popcount32(m[4]), popcount32(m[5]),
                     popcount32(m[6]), popcount32(m[7]),
                     popcount32(m[8]), popcount32(m[9]),
                     popcount32(m[10]), popcount32(m[11]),
                     popcount32(m[12]), popcount32(m[13]),
                     popcount32(m[14]), popcount32(m[15])};

    // Compress each 32-bit chunk individually.
    u32 v[16] = { compress32(x[0], m[0]), compress32(x[1], m[1]),
                  compress32(x[2], m[2]), compress32(x[3], m[3]),
                  compress32(x[4], m[4]), compress32(x[5], m[5]),
                  compress32(x[6], m[6]), compress32(x[7], m[7]),
                  compress32(x[8], m[8]), compress32(x[9], m[9]),
                  compress32(x[10], m[10]), compress32(x[11], m[11]),
                  compress32(x[12], m[12]), compress32(x[13], m[13]),
                  compress32(x[14], m[14]), compress32(x[15], m[15]) };

    // Write packed data out.
    pack_bits_32(ptr, v, bits, 16);
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
void storecompressed512_64bit(void *ptr, m512 xvec, m512 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a m[8];
    memcpy(m, &mvec, sizeof(mvec));
    u64a x[8];
    memcpy(x, &xvec, sizeof(xvec));

    // Count the number of bits of compressed state we're writing out per
    // chunk.
    u32 bits[8] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]),
                    popcount64(m[4]), popcount64(m[5]),
                    popcount64(m[6]), popcount64(m[7]) };

    // Compress each 64-bit chunk individually.
    u64a v[8] = { compress64(x[0], m[0]), compress64(x[1], m[1]),
                  compress64(x[2], m[2]), compress64(x[3], m[3]),
                  compress64(x[4], m[4]), compress64(x[5], m[5]),
                  compress64(x[6], m[6]), compress64(x[7], m[7]) };

    // Write packed data out.
    pack_bits_64(ptr, v, bits, 8);
}
#endif

void storecompressed512(void *ptr, const m512 *x, const m512 *m,
                        UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    storecompressed512_64bit(ptr, *x, *m);
#else
    storecompressed512_32bit(ptr, *x, *m);
#endif
}

#if defined(ARCH_32_BIT)
static really_inline
m512 loadcompressed512_32bit(const void *ptr, m512 mvec) {
    // First, decompose our vectors into 32-bit chunks.
    u32 m[16];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[16] = { popcount32(m[0]), popcount32(m[1]),
                     popcount32(m[2]), popcount32(m[3]),
                     popcount32(m[4]), popcount32(m[5]),
                     popcount32(m[6]), popcount32(m[7]),
                     popcount32(m[8]), popcount32(m[9]),
                     popcount32(m[10]), popcount32(m[11]),
                     popcount32(m[12]), popcount32(m[13]),
                     popcount32(m[14]), popcount32(m[15]) };
    u32 v[16];

    unpack_bits_32(v, (const u8 *)ptr, bits, 16);

    u32 x[16] = { expand32(v[0], m[0]), expand32(v[1], m[1]),
                  expand32(v[2], m[2]), expand32(v[3], m[3]),
                  expand32(v[4], m[4]), expand32(v[5], m[5]),
                  expand32(v[6], m[6]), expand32(v[7], m[7]),
                  expand32(v[8], m[8]), expand32(v[9], m[9]),
                  expand32(v[10], m[10]), expand32(v[11], m[11]),
                  expand32(v[12], m[12]), expand32(v[13], m[13]),
                  expand32(v[14], m[14]), expand32(v[15], m[15]) };

    m512 xvec;
#if defined(HAVE_AVX512)
    xvec = _mm512_set_epi32(x[15], x[14], x[13], x[12],
                            x[11], x[10], x[9], x[8],
                            x[7], x[6], x[5], x[4],
                            x[3], x[2], x[1], x[0]);
#elif defined(HAVE_AVX2)
    xvec.lo = _mm256_set_epi32(x[7], x[6], x[5], x[4],
                               x[3], x[2], x[1], x[0]);
    xvec.hi = _mm256_set_epi32(x[15], x[14], x[13], x[12],
                               x[11], x[10], x[9], x[8]);
#else
    xvec.lo.lo = _mm_set_epi32(x[3], x[2], x[1], x[0]);
    xvec.lo.hi = _mm_set_epi32(x[7], x[6], x[5], x[4]);
    xvec.hi.lo = _mm_set_epi32(x[11], x[10], x[9], x[8]);
    xvec.hi.hi = _mm_set_epi32(x[15], x[14], x[13], x[12]);
#endif
    return xvec;
}
#endif

#if defined(ARCH_64_BIT)
static really_inline
m512 loadcompressed512_64bit(const void *ptr, m512 mvec) {
    // First, decompose our vectors into 64-bit chunks.
    u64a m[8];
    memcpy(m, &mvec, sizeof(mvec));

    u32 bits[8] = { popcount64(m[0]), popcount64(m[1]),
                    popcount64(m[2]), popcount64(m[3]),
                    popcount64(m[4]), popcount64(m[5]),
                    popcount64(m[6]), popcount64(m[7]) };
    u64a v[8];

    unpack_bits_64(v, (const u8 *)ptr, bits, 8);

    u64a x[8] = { expand64(v[0], m[0]), expand64(v[1], m[1]),
                  expand64(v[2], m[2]), expand64(v[3], m[3]),
                  expand64(v[4], m[4]), expand64(v[5], m[5]),
                  expand64(v[6], m[6]), expand64(v[7], m[7]) };

#if defined(HAVE_AVX512)
    m512 xvec = _mm512_set_epi64(x[7], x[6], x[5], x[4],
                                 x[3], x[2], x[1], x[0]);
#elif defined(HAVE_AVX2)
    m512 xvec = { .lo = _mm256_set_epi64x(x[3], x[2], x[1], x[0]),
                  .hi = _mm256_set_epi64x(x[7], x[6], x[5], x[4])};
#else
    m512 xvec = { .lo = { _mm_set_epi64x(x[1], x[0]),
                          _mm_set_epi64x(x[3], x[2]) },
                  .hi = { _mm_set_epi64x(x[5], x[4]),
                          _mm_set_epi64x(x[7], x[6]) } };
#endif
    return xvec;
}
#endif

void loadcompressed512(m512 *x, const void *ptr, const m512 *m,
                       UNUSED u32 bytes) {
#if defined(ARCH_64_BIT)
    *x = loadcompressed512_64bit(ptr, *m);
#else
    *x = loadcompressed512_32bit(ptr, *m);
#endif
}
