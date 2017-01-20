/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 *
 * For the SIMD types (128 bits and above), we pass a pointer to the
 * implementation NFA structure instead of three masks: otherwise we spend all
 * our time building stack frames.
 */

#ifndef LIMEX_ACCEL_H
#define LIMEX_ACCEL_H

#include "util/simd_utils.h" // for m128 etc

union AccelAux;
struct LimExNFA64;
struct LimExNFA128;
struct LimExNFA256;
struct LimExNFA384;
struct LimExNFA512;

size_t doAccel32(u32 s, u32 accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end);

#ifdef ARCH_64_BIT
size_t doAccel64(u64a s, u64a accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end);
#else
size_t doAccel64(m128 s, m128 accel, const u8 *accelTable,
                 const union AccelAux *aux, const u8 *input, size_t i,
                 size_t end);
#endif

size_t doAccel128(const m128 *s, const struct LimExNFA128 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end);

size_t doAccel256(const m256 *s, const struct LimExNFA256 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end);

size_t doAccel384(const m384 *s, const struct LimExNFA384 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end);

size_t doAccel512(const m512 *s, const struct LimExNFA512 *limex,
                  const u8 *accelTable, const union AccelAux *aux,
                  const u8 *input, size_t i, size_t end);

#endif
