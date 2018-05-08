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

#include "config.h"

#include "sig.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include <string>

#if defined(HAVE_SIGACTION) || defined(_WIN32)
#include <signal.h>
#endif

#ifdef HAVE_BACKTRACE
#include <execinfo.h>
#include <unistd.h>
#endif

#define BACKTRACE_BUFFER_SIZE 200

TLS_VARIABLE volatile int debug_stage = STAGE_UNDEFINED;
TLS_VARIABLE volatile int debug_expr = 0;
TLS_VARIABLE const char * volatile debug_expr_ptr = nullptr;
TLS_VARIABLE volatile int debug_corpus = 0;
TLS_VARIABLE const char * volatile debug_corpus_ptr = nullptr;
TLS_VARIABLE volatile size_t debug_corpus_len = 0;

extern std::string g_cmdline;

#if defined(_WIN32)
static void __cdecl sighandler(int signum) {
#elif defined(HAVE_SIGACTION)
static void sighandler(int signum) {
#endif
#if defined(HAVE_SIGACTION) || defined(_WIN32)
    /* NOTE: This signal handler is designed solely to provide more information
     * when a crash occurs in ue2collider -- it makes calls to signal-unsafe
     * functions like printf() and backtrace() by design, since we're already
     * in deep trouble and are going to exit anyway. */

    fflush(stdout);
    printf("signal %d\n", signum);
    printf("\nFailing cmdline was:\n%s\n\n", g_cmdline.c_str());
    printf("expression %d ", debug_expr);
    switch(debug_stage) {
    case STAGE_UE2_COMPILE:
        printf("ue2 compile\n");
        break;
    case STAGE_UE2_RUN:
        printf("corpus %d ue2 scan\n", debug_corpus);
        break;
    case STAGE_PCRE_COMPILE:
        printf("pcre compile\n");
        break;
    case STAGE_PCRE_RUN:
        printf("corpus %d pcre scan\n", debug_corpus);
        break;
    case STAGE_GRAPH_PREPROCESS:
        printf("graph preprocess\n");
        break;
    case STAGE_GRAPH_COMPILE:
        printf("graph compile\n");
        break;
    case STAGE_GRAPH_RUN:
        printf("corpus %d graph scan\n", debug_corpus);
        break;
    default:
    case STAGE_UNDEFINED:
        printf("unknown stage\n");
        break;
    }
    printf("\n");

    if (debug_expr_ptr) {
        printf("expression %p\n", debug_expr_ptr);
        printf("%d:%s\n\n", debug_expr, debug_expr_ptr);
    }

    if (debug_stage == STAGE_PCRE_RUN || debug_stage == STAGE_UE2_RUN) {
        printf("corpus %p len %zu\n", debug_corpus_ptr, debug_corpus_len);

        printf("%d:", debug_expr);
        for (size_t i = 0; i < debug_corpus_len && debug_corpus_ptr; i++) {
            unsigned char c = debug_corpus_ptr[i];
            if (c == '\n') {
                printf("\\n");
            } else if (c == '\t') {
                printf("\\t");
            } else if (c == '\r') {
                printf("\\r");
            } else if (0x20 <= c && c <= 0x7e && c != '\\') {
                printf("%c", c);
            } else {
                printf("\\x%02hhx", c);
            }
        }
        printf("\n\n");
    }

    fflush(stdout);

#ifdef HAVE_BACKTRACE
    static void *bt[BACKTRACE_BUFFER_SIZE];
    int count = backtrace(bt, BACKTRACE_BUFFER_SIZE);
    if (count) {
        backtrace_symbols_fd(bt, count, STDOUT_FILENO);
    } else {
        printf("(Call to backtrace() returns zero count.)\n");
    }
#else
    printf("(Backtrace unavailable on this platform.)\n");
#endif

    _exit(signum);
}
#endif // HAVE_SIGACTION

void installSignalHandler(void) {

#ifdef _WIN32
    signal(SIGABRT, sighandler);
    signal(SIGFPE, sighandler);
    signal(SIGILL, sighandler);
    signal(SIGSEGV, sighandler);
#elif defined(HAVE_SIGACTION)
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = sighandler;
    act.sa_flags = 0;
    sigemptyset(&act.sa_mask);
    sigaddset(&act.sa_mask, SIGSEGV);
    sigaddset(&act.sa_mask, SIGBUS);
    sigaddset(&act.sa_mask, SIGFPE);
    sigaddset(&act.sa_mask, SIGILL);
    sigaddset(&act.sa_mask, SIGABRT);
    sigaction(SIGBUS, &act, nullptr);
    sigaction(SIGFPE, &act, nullptr);
    sigaction(SIGILL, &act, nullptr);
    sigaction(SIGABRT, &act, nullptr);
    sigaction(SIGSEGV, &act, nullptr);
    setSignalStack();
#endif // HAVE_SIGACTION
}

#ifdef HAVE_SIGALTSTACK
static TLS_VARIABLE char alt_stack_loc[SIGSTKSZ];
#endif

void setSignalStack(void) {
#ifdef HAVE_SIGALTSTACK
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = sighandler;
    act.sa_flags = 0;
    stack_t alt_stack;
    memset(&alt_stack, 0, sizeof(alt_stack));
    alt_stack.ss_flags = 0;
    alt_stack.ss_size = SIGSTKSZ;
    alt_stack.ss_sp = alt_stack_loc;
    if (!sigaltstack(&alt_stack, nullptr)) {
        act.sa_flags |= SA_ONSTACK;
    }
    sigaction(SIGSEGV, &act, nullptr);
#endif
}

