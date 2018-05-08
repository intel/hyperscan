/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#ifndef SIG_H
#define SIG_H

#include <cstddef> // for size_t

#define STAGE_UNDEFINED 0
#define STAGE_UE2_COMPILE 1
#define STAGE_UE2_RUN 2
#define STAGE_PCRE_COMPILE 3
#define STAGE_PCRE_RUN 4
#define STAGE_GRAPH_PREPROCESS 5
#define STAGE_GRAPH_COMPILE 6
#define STAGE_GRAPH_RUN 7

#ifndef WIN32
#define TLS_VARIABLE __thread
#else
#define TLS_VARIABLE __declspec(thread)
#endif

extern TLS_VARIABLE volatile int debug_stage;
extern TLS_VARIABLE volatile int debug_expr;
extern TLS_VARIABLE const char * volatile debug_expr_ptr;
extern TLS_VARIABLE volatile int debug_corpus;
extern TLS_VARIABLE const char * volatile debug_corpus_ptr;
extern TLS_VARIABLE volatile size_t debug_corpus_len;

void installSignalHandler(void);

// Must be called by every thread.
void setSignalStack(void);

#endif
