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
 * \brief Limex NFA: acceleration runtime.
 */

#include "limex_accel.h"

#include "accel.h"
#include "limex_internal.h"
#include "limex_limits.h"
#include "limex_shuffle.h"
#include "nfa_internal.h"
#include "shufti.h"
#include "truffle.h"
#include "ue2common.h"
#include "vermicelli.h"
#include "util/arch.h"
#include "util/bitutils.h"
#include "util/simd_utils.h"

static really_inline
size_t accelScanWrapper(const u8 *accelTable, const union AccelAux *aux,
                        const u8 *input, u32 idx, size_t i, size_t end) {
    assert(accelTable);
    assert(aux);

    DEBUG_PRINTF("shuffle returned %u -> aux %u\n", idx, accelTable[idx]);
    assert(idx < (1 << NFA_MAX_ACCEL_STATES));
    if (!idx) {
        return end;
    }

    u8 aux_idx = accelTable[idx];
    if (!aux_idx) {
        assert(aux[0].accel_type == ACCEL_NONE);
        DEBUG_PRINTF("no accel, bailing\n");
        return i;
    }

    aux = aux + aux_idx;
    const u8 *ptr = run_accel(aux, &input[i], &input[end]);
    assert(ptr >= &input[i]);
    size_t j = (size_t)(ptr - input);
    DEBUG_PRINTF("accel skipped %zu of %zu chars\n", (j - i), (end - i));
    DEBUG_PRINTF("returning j=%zu (i=%zu, end=%zu)\n", j, i, end);
    return j;
}

size_t doAccel32(u32 s, u32 accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end) {
    u32 idx = pext32(s, accel);
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}

#ifdef ARCH_64_BIT
size_t doAccel64(u64a s, u64a accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end) {
    u32 idx = pext64(s, accel);
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}
#else
size_t doAccel64(m128 s, m128 accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end) {
    u32 idx = pext64(movq(s), movq(accel));
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}
#endif

size_t doAccel128(const m128 *state, const struct LimExNFA128 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end) {
    u32 idx;
    m128 s = *state;
    DEBUG_PRINTF("using PSHUFB for 128-bit shuffle\n");
    m128 accelPerm = limex->accelPermute;
    m128 accelComp = limex->accelCompare;
    idx = packedExtract128(s, accelPerm, accelComp);
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}

size_t doAccel256(const m256 *state, const struct LimExNFA256 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end) {
    u32 idx;
    m256 s = *state;
    DEBUG_PRINTF("using PSHUFB for 256-bit shuffle\n");
    m256 accelPerm = limex->accelPermute;
    m256 accelComp = limex->accelCompare;
#if !defined(HAVE_AVX2)
    u32 idx1 = packedExtract128(s.lo, accelPerm.lo, accelComp.lo);
    u32 idx2 = packedExtract128(s.hi, accelPerm.hi, accelComp.hi);
    assert((idx1 & idx2) == 0); // should be no shared bits
    idx = idx1 | idx2;
#else
    idx = packedExtract256(s, accelPerm, accelComp);
#endif
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}

size_t doAccel384(const m384 *state, const struct LimExNFA384 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end) {
    u32 idx;
    m384 s = *state;
    DEBUG_PRINTF("using PSHUFB for 384-bit shuffle\n");
    m384 accelPerm = limex->accelPermute;
    m384 accelComp = limex->accelCompare;
    u32 idx1 = packedExtract128(s.lo, accelPerm.lo, accelComp.lo);
    u32 idx2 = packedExtract128(s.mid, accelPerm.mid, accelComp.mid);
    u32 idx3 = packedExtract128(s.hi, accelPerm.hi, accelComp.hi);
    assert((idx1 & idx2 & idx3) == 0); // should be no shared bits
    idx = idx1 | idx2 | idx3;
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}

size_t doAccel512(const m512 *state, const struct LimExNFA512 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end) {
    u32 idx;
    m512 s = *state;
    DEBUG_PRINTF("using PSHUFB for 512-bit shuffle\n");
    m512 accelPerm = limex->accelPermute;
    m512 accelComp = limex->accelCompare;
#if defined(HAVE_AVX512)
    idx = packedExtract512(s, accelPerm, accelComp);
#elif defined(HAVE_AVX2)
    u32 idx1 = packedExtract256(s.lo, accelPerm.lo, accelComp.lo);
    u32 idx2 = packedExtract256(s.hi, accelPerm.hi, accelComp.hi);
    assert((idx1 & idx2) == 0); // should be no shared bits
    idx = idx1 | idx2;
#else
    u32 idx1 = packedExtract128(s.lo.lo, accelPerm.lo.lo, accelComp.lo.lo);
    u32 idx2 = packedExtract128(s.lo.hi, accelPerm.lo.hi, accelComp.lo.hi);
    u32 idx3 = packedExtract128(s.hi.lo, accelPerm.hi.lo, accelComp.hi.lo);
    u32 idx4 = packedExtract128(s.hi.hi, accelPerm.hi.hi, accelComp.hi.hi);
    assert((idx1 & idx2 & idx3 & idx4) == 0); // should be no shared bits
    idx = idx1 | idx2 | idx3 | idx4;
#endif
    return accelScanWrapper(accelTable, aux, input, idx, i, end);
}
