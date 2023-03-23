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
#include "mcsheng_dump.h"
#include "mpv_dump.h"
#include "shengdump.h"
#include "tamarama_dump.h"

#ifndef DUMP_SUPPORT
#error "no dump support"
#endif

namespace ue2 {

#define DISPATCH_CASE(dc_ltype, dc_ftype, dc_func_call)                        \
    case dc_ltype:                                                             \
        nfaExec##dc_ftype##dc_func_call;                                       \
    break

// general framework calls

#define DISPATCH_BY_NFA_TYPE(dbnt_func)                                        \
    DEBUG_PRINTF("dispatch for NFA type %u\n", nfa->type);                     \
    switch (nfa->type) {                                                       \
        DISPATCH_CASE(LIMEX_NFA_32, LimEx32, dbnt_func);                       \
        DISPATCH_CASE(LIMEX_NFA_64, LimEx64, dbnt_func);                       \
        DISPATCH_CASE(LIMEX_NFA_128, LimEx128, dbnt_func);                     \
        DISPATCH_CASE(LIMEX_NFA_256, LimEx256, dbnt_func);                     \
        DISPATCH_CASE(LIMEX_NFA_384, LimEx384, dbnt_func);                     \
        DISPATCH_CASE(LIMEX_NFA_512, LimEx512, dbnt_func);                     \
        DISPATCH_CASE(MCCLELLAN_NFA_8, McClellan8, dbnt_func);                 \
        DISPATCH_CASE(MCCLELLAN_NFA_16, McClellan16, dbnt_func);               \
        DISPATCH_CASE(GOUGH_NFA_8, Gough8, dbnt_func);                         \
        DISPATCH_CASE(GOUGH_NFA_16, Gough16, dbnt_func);                       \
        DISPATCH_CASE(MPV_NFA, Mpv, dbnt_func);                                \
        DISPATCH_CASE(LBR_NFA_DOT, LbrDot, dbnt_func);                         \
        DISPATCH_CASE(LBR_NFA_VERM, LbrVerm, dbnt_func);                       \
        DISPATCH_CASE(LBR_NFA_NVERM, LbrNVerm, dbnt_func);                     \
        DISPATCH_CASE(LBR_NFA_SHUF, LbrShuf, dbnt_func);                       \
        DISPATCH_CASE(LBR_NFA_TRUF, LbrTruf, dbnt_func);                       \
        DISPATCH_CASE(CASTLE_NFA, Castle, dbnt_func);                          \
        DISPATCH_CASE(SHENG_NFA, Sheng, dbnt_func);                            \
        DISPATCH_CASE(TAMARAMA_NFA, Tamarama, dbnt_func);                      \
        DISPATCH_CASE(MCSHENG_NFA_8, McSheng8, dbnt_func);                     \
        DISPATCH_CASE(MCSHENG_NFA_16, McSheng16, dbnt_func);                   \
        DISPATCH_CASE(SHENG_NFA_32, Sheng32, dbnt_func);                       \
        DISPATCH_CASE(SHENG_NFA_64, Sheng64, dbnt_func);                       \
        DISPATCH_CASE(MCSHENG_64_NFA_8, McSheng64_8, dbnt_func);               \
        DISPATCH_CASE(MCSHENG_64_NFA_16, McSheng64_16, dbnt_func);             \
    default:                                                                   \
        assert(0);                                                             \
    }

void nfaGenerateDumpFiles(const struct NFA *nfa, const std::string &base) {
    DISPATCH_BY_NFA_TYPE(_dump(nfa, base));
}

} // namespace ue2

