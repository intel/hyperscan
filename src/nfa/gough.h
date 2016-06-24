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

#ifndef GOUGH_H
#define GOUGH_H

#include "callback.h"
#include "ue2common.h"

struct NFA;
struct mq;

// 8-bit Gough

char nfaExecGough8_testEOD(const struct NFA *nfa, const char *state,
                           const char *streamState, u64a offset,
                           NfaCallback callback, void *context);
char nfaExecGough8_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecGough8_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecGough8_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecGough8_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecGough8_inAccept(const struct NFA *n, ReportID report, struct mq *q);
char nfaExecGough8_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecGough8_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecGough8_initCompressedState(const struct NFA *n, u64a offset,
                                       void *state, u8 key);
char nfaExecGough8_queueCompressState(const struct NFA *nfa, const struct mq *q,
                                      s64a loc);
char nfaExecGough8_expandState(const struct NFA *nfa, void *dest,
                               const void *src, u64a offset, u8 key);

#define nfaExecGough8_B_Reverse NFA_API_NO_IMPL
#define nfaExecGough8_zombie_status NFA_API_ZOMBIE_NO_IMPL

// 16-bit Gough

char nfaExecGough16_testEOD(const struct NFA *nfa, const char *state,
                            const char *streamState, u64a offset,
                            NfaCallback callback, void *context);
char nfaExecGough16_Q(const struct NFA *n, struct mq *q, s64a end);
char nfaExecGough16_Q2(const struct NFA *n, struct mq *q, s64a end);
char nfaExecGough16_QR(const struct NFA *n, struct mq *q, ReportID report);
char nfaExecGough16_reportCurrent(const struct NFA *n, struct mq *q);
char nfaExecGough16_inAccept(const struct NFA *n, ReportID report, struct mq *q);
char nfaExecGough16_inAnyAccept(const struct NFA *n, struct mq *q);
char nfaExecGough16_queueInitState(const struct NFA *n, struct mq *q);
char nfaExecGough16_initCompressedState(const struct NFA *n, u64a offset,
                                        void *state, u8 key);
char nfaExecGough16_queueCompressState(const struct NFA *nfa,
                                       const struct mq *q, s64a loc);
char nfaExecGough16_expandState(const struct NFA *nfa, void *dest,
                                const void *src, u64a offset, u8 key);

#define nfaExecGough16_B_Reverse NFA_API_NO_IMPL
#define nfaExecGough16_zombie_status NFA_API_ZOMBIE_NO_IMPL

#endif
