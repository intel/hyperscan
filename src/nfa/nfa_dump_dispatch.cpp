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

#include "config.h"

#include "nfa_dump_api.h"

#include "nfa_internal.h"
#include "ue2common.h"

// Engine implementations.
#include "goughdump.h"
#include "castle_dump.h"
#include "lbr_dump.h"
#include "limex.h"
#include "mcclellandump.h"
#include "mpv_dump.h"

#ifndef DUMP_SUPPORT
#error "no dump support"
#endif

namespace ue2 {

#define DISPATCH_CASE(dc_ltype, dc_ftype, dc_subtype, dc_func_call) \
    case dc_ltype##_NFA_##dc_subtype:                               \
    nfaExec##dc_ftype##dc_subtype##dc_func_call;                    \
    break

// general framework calls

#define DISPATCH_BY_NFA_TYPE(dbnt_func)                       \
    DEBUG_PRINTF("dispatch for NFA type %u\n", nfa->type);    \
    switch (nfa->type) {                                      \
        DISPATCH_CASE(LIMEX,   LimEx,   32_1, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_2, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_3, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_4, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_5, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_6, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   32_7, dbnt_func);     \
        DISPATCH_CASE(LIMEX,   LimEx,   128_1, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_2, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_3, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_4, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_5, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_6, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   128_7, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_1, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_2, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_3, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_4, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_5, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_6, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   256_7, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_1, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_2, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_3, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_4, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_5, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_6, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   384_7, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_1, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_2, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_3, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_4, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_5, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_6, dbnt_func);    \
        DISPATCH_CASE(LIMEX,   LimEx,   512_7, dbnt_func);    \
        DISPATCH_CASE(MCCLELLAN, McClellan, 8, dbnt_func);    \
        DISPATCH_CASE(MCCLELLAN, McClellan, 16, dbnt_func);   \
        DISPATCH_CASE(GOUGH, Gough, 8, dbnt_func);            \
        DISPATCH_CASE(GOUGH, Gough, 16, dbnt_func);           \
        DISPATCH_CASE(MPV, Mpv, 0, dbnt_func);                \
        DISPATCH_CASE(LBR, Lbr, Dot, dbnt_func);              \
        DISPATCH_CASE(LBR, Lbr, Verm, dbnt_func);             \
        DISPATCH_CASE(LBR, Lbr, NVerm, dbnt_func);            \
        DISPATCH_CASE(LBR, Lbr, Shuf, dbnt_func);             \
        DISPATCH_CASE(LBR, Lbr, Truf, dbnt_func);             \
        DISPATCH_CASE(CASTLE, Castle, 0, dbnt_func);          \
    default:                                                  \
        assert(0);                                            \
    }

void nfaDumpDot(const struct NFA *nfa, FILE *dotFile) {
    DISPATCH_BY_NFA_TYPE(_dumpDot(nfa, dotFile));
}

void nfaDumpText(const struct NFA *nfa, FILE *txtFile) {
    DISPATCH_BY_NFA_TYPE(_dumpText(nfa, txtFile));
}

} // namespace ue2

