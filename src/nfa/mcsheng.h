/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef MCSHENG_H
#define MCSHENG_H

#include "callback.h"
#include "ue2common.h"

struct mq;
struct NFA;

/* 8-bit Sheng-McClellan hybrid */

char nfaExecMcSheng8_testEOD(const struct NFA *nfa, const char *state,
                             const char *streamState, u64a offset,
                             NfaCallback callback, void *context);
char nfaExecMcSheng8_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng8_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng8_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecMcSheng8_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecMcSheng8_inAccept(const struct NFA *n, ReportID report,
                              struct mq *q);
char nfaExecMcSheng8_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecMcSheng8_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecMcSheng8_initCompressedState(const struct NFA *n, u64a offset,
                                         void *state, u8 key);
char nfaExecMcSheng8_queueCompressState(const struct NFA *nfa,
                                        const struct mq *q, s64a loc);
char nfaExecMcSheng8_expandState(const struct NFA *nfa, void *dest,
                                 const void *src, u64a offset, u8 key);

#define nfaExecMcSheng8_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng8_zombie_status NFA_API_ZOMBIE_NO_IMPL

/* 16-bit Sheng-McClellan hybrid */

char nfaExecMcSheng16_testEOD(const struct NFA *nfa, const char *state,
                              const char *streamState, u64a offset,
                              NfaCallback callback, void *context);
char nfaExecMcSheng16_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng16_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng16_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecMcSheng16_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecMcSheng16_inAccept(const struct NFA *n, ReportID report,
                               struct mq *q);
char nfaExecMcSheng16_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecMcSheng16_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecMcSheng16_initCompressedState(const struct NFA *n, u64a offset,
                                          void *state, u8 key);
char nfaExecMcSheng16_queueCompressState(const struct NFA *nfa,
                                         const struct mq *q, s64a loc);
char nfaExecMcSheng16_expandState(const struct NFA *nfa, void *dest,
                                  const void *src, u64a offset, u8 key);

#define nfaExecMcSheng16_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng16_zombie_status NFA_API_ZOMBIE_NO_IMPL
#if defined(HAVE_AVX512VBMI)
/* 64-8 bit Sheng-McClellan hybrid  */
char nfaExecMcSheng64_8_testEOD(const struct NFA *nfa, const char *state,
                                const char *streamState, u64a offset,
                                NfaCallback callback, void *context);
char nfaExecMcSheng64_8_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng64_8_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng64_8_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecMcSheng64_8_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_8_inAccept(const struct NFA *n, ReportID report,
                                 struct mq *q);
char nfaExecMcSheng64_8_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_8_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_8_initCompressedState(const struct NFA *n, u64a offset,
                                            void *state, u8 key);
char nfaExecMcSheng64_8_queueCompressState(const struct NFA *nfa,
                                           const struct mq *q, s64a loc);
char nfaExecMcSheng64_8_expandState(const struct NFA *nfa, void *dest,
                                    const void *src, u64a offset, u8 key);

#define nfaExecMcSheng64_8_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_zombie_status NFA_API_ZOMBIE_NO_IMPL

/* 64-16 bit Sheng-McClellan hybrid  */
char nfaExecMcSheng64_16_testEOD(const struct NFA *nfa, const char *state,
                                 const char *streamState, u64a offset,
                                 NfaCallback callback, void *context);
char nfaExecMcSheng64_16_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng64_16_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecMcSheng64_16_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecMcSheng64_16_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_16_inAccept(const struct NFA *n, ReportID report,
                                  struct mq *q);
char nfaExecMcSheng64_16_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_16_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecMcSheng64_16_initCompressedState(const struct NFA *n, u64a offset,
                                             void *state, u8 key);
char nfaExecMcSheng64_16_queueCompressState(const struct NFA *nfa,
                                            const struct mq *q, s64a loc);
char nfaExecMcSheng64_16_expandState(const struct NFA *nfa, void *dest,
                                     const void *src, u64a offset, u8 key);
#define nfaExecMcSheng64_16_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_zombie_status NFA_API_ZOMBIE_NO_IMPL
#else // !HAVE_AVX512VBMI
#define nfaExecMcSheng64_8_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_zombie_status NFA_API_ZOMBIE_NO_IMPL
#define nfaExecMcSheng64_8_Q NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_Q2 NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_QR NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_inAccept NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_inAnyAccept NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_queueInitState NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_queueCompressState NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_expandState NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_initCompressedState NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_testEOD NFA_API_NO_IMPL
#define nfaExecMcSheng64_8_reportCurrent NFA_API_NO_IMPL

#define nfaExecMcSheng64_16_B_Reverse NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_zombie_status NFA_API_ZOMBIE_NO_IMPL
#define nfaExecMcSheng64_16_Q NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_Q2 NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_QR NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_inAccept NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_inAnyAccept NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_queueInitState NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_queueCompressState NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_expandState NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_initCompressedState NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_testEOD NFA_API_NO_IMPL
#define nfaExecMcSheng64_16_reportCurrent NFA_API_NO_IMPL

#endif //end of HAVE_AVX512VBMI

#endif
