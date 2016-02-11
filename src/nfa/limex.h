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

#ifndef LIMEX_H
#define LIMEX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "nfa_api.h"

#if defined(DUMP_SUPPORT) && defined(__cplusplus)
#define GENERATE_NFA_DUMP_DECL(gf_name)                                        \
    } /* extern "C" */                                                         \
    namespace ue2 {                                                            \
    void gf_name##_dumpDot(const struct NFA *nfa, FILE *file);                 \
    void gf_name##_dumpText(const struct NFA *nfa, FILE *file);                \
    } /* namespace ue2 */                                                      \
    extern "C" {

#else
#define GENERATE_NFA_DUMP_DECL(gf_name)
#endif

#define GENERATE_NFA_DECL(gf_name)                                             \
    char gf_name##_testEOD(const struct NFA *nfa, const char *state,           \
                           const char *streamState, u64a offset,               \
                           NfaCallback callback, SomNfaCallback som_cb,        \
                           void *context);                                     \
    char gf_name##_Q(const struct NFA *n, struct mq *q, s64a end);             \
    char gf_name##_Q2(const struct NFA *n, struct mq *q, s64a end);            \
    char gf_name##_QR(const struct NFA *n, struct mq *q, ReportID report);     \
    char gf_name##_reportCurrent(const struct NFA *n, struct mq *q);           \
    char gf_name##_inAccept(const struct NFA *n, ReportID report,              \
                            struct mq *q);                                     \
    char gf_name##_queueInitState(const struct NFA *n, struct mq *q);          \
    char gf_name##_initCompressedState(const struct NFA *n, u64a offset,       \
                                       void *state, u8 key);                   \
    char gf_name##_B_Reverse(const struct NFA *n, u64a offset, const u8 *buf,  \
                             size_t buflen, const u8 *hbuf, size_t hlen,       \
                             NfaCallback cb, void *context);                   \
    char gf_name##_queueCompressState(const struct NFA *nfa,                   \
                                      const struct mq *q, s64a loc);           \
    char gf_name##_expandState(const struct NFA *nfa, void *dest,              \
                               const void *src, u64a offset, u8 key);          \
    enum nfa_zombie_status gf_name##_zombie_status(const struct NFA *nfa,      \
                                                   struct mq *q, s64a loc);    \
    GENERATE_NFA_DUMP_DECL(gf_name)

GENERATE_NFA_DECL(nfaExecLimEx32_1)
GENERATE_NFA_DECL(nfaExecLimEx32_2)
GENERATE_NFA_DECL(nfaExecLimEx32_3)
GENERATE_NFA_DECL(nfaExecLimEx32_4)
GENERATE_NFA_DECL(nfaExecLimEx32_5)
GENERATE_NFA_DECL(nfaExecLimEx32_6)
GENERATE_NFA_DECL(nfaExecLimEx32_7)
GENERATE_NFA_DECL(nfaExecLimEx128_1)
GENERATE_NFA_DECL(nfaExecLimEx128_2)
GENERATE_NFA_DECL(nfaExecLimEx128_3)
GENERATE_NFA_DECL(nfaExecLimEx128_4)
GENERATE_NFA_DECL(nfaExecLimEx128_5)
GENERATE_NFA_DECL(nfaExecLimEx128_6)
GENERATE_NFA_DECL(nfaExecLimEx128_7)
GENERATE_NFA_DECL(nfaExecLimEx256_1)
GENERATE_NFA_DECL(nfaExecLimEx256_2)
GENERATE_NFA_DECL(nfaExecLimEx256_3)
GENERATE_NFA_DECL(nfaExecLimEx256_4)
GENERATE_NFA_DECL(nfaExecLimEx256_5)
GENERATE_NFA_DECL(nfaExecLimEx256_6)
GENERATE_NFA_DECL(nfaExecLimEx256_7)
GENERATE_NFA_DECL(nfaExecLimEx384_1)
GENERATE_NFA_DECL(nfaExecLimEx384_2)
GENERATE_NFA_DECL(nfaExecLimEx384_3)
GENERATE_NFA_DECL(nfaExecLimEx384_4)
GENERATE_NFA_DECL(nfaExecLimEx384_5)
GENERATE_NFA_DECL(nfaExecLimEx384_6)
GENERATE_NFA_DECL(nfaExecLimEx384_7)
GENERATE_NFA_DECL(nfaExecLimEx512_1)
GENERATE_NFA_DECL(nfaExecLimEx512_2)
GENERATE_NFA_DECL(nfaExecLimEx512_3)
GENERATE_NFA_DECL(nfaExecLimEx512_4)
GENERATE_NFA_DECL(nfaExecLimEx512_5)
GENERATE_NFA_DECL(nfaExecLimEx512_6)
GENERATE_NFA_DECL(nfaExecLimEx512_7)

#undef GENERATE_NFA_DECL
#undef GENERATE_NFA_DUMP_DECL

#ifdef __cplusplus
}
#endif

#endif
