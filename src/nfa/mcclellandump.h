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

#ifndef MCCLELLAN_DUMP_H
#define MCCLELLAN_DUMP_H

#ifdef DUMP_SUPPORT

#include "rdfa.h"

#include <cstdio>
#include <string>

struct mcclellan;
struct mstate_aux;
struct NFA;
union AccelAux;

namespace ue2 {

void nfaExecMcClellan8_dump(const struct NFA *nfa, const std::string &base);
void nfaExecMcClellan16_dump(const struct NFA *nfa, const std::string &base);

/* These functions are shared with the Gough dump code. */

const mstate_aux *getAux(const NFA *n, dstate_id_t i);
void describeEdge(FILE *f, const u16 *t, u16 i);
void dumpAccelText(FILE *f, const union AccelAux *accel);
void dumpAccelDot(FILE *f, u16 i, const union AccelAux *accel);
void describeAlphabet(FILE *f, const mcclellan *m);
void dumpDotPreambleDfa(FILE *f);

} // namespace ue2

#endif // DUMP_SUPPORT

#endif // MCCLELLAN_DUMP_H
