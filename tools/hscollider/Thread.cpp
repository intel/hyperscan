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

#include "Thread.h"
#include "common.h"
#include "sig.h"

#include <cstdlib>
#include <iostream>

#ifndef _WIN32
static const size_t COLLIDER_THREAD_STACK_SIZE = 8192 * 1024;

void Thread::start() {
    // Some systems, notably Mac OS X, use a default stack size that is
    // smaller than what we want (particularly given that we're planning on
    // running PCRE, which recurses inside pcre_exec). We attempt to
    // increase it to 8MB.
    int ret;
    pthread_attr_t attr;
    ret = pthread_attr_init(&attr);
    if (ret) {
        std::cerr << "pthread_attr_init failed" << std::endl;
        exit(1);
    }

    size_t stacksize = 0;
    ret = pthread_attr_getstacksize(&attr, &stacksize);
    if (ret) {
        std::cerr << "Warning: can't query stack size with "
                     "pthread_attr_getstacksize" << std::endl;
        goto create_thread;
    }

    if (stacksize < COLLIDER_THREAD_STACK_SIZE) {
        ret = pthread_attr_setstacksize(&attr, COLLIDER_THREAD_STACK_SIZE);
        if (ret) {
            std::cerr << "Warning: pthread_attr_setstacksize failed, "
                         "unable to set stack size to "
                      << COLLIDER_THREAD_STACK_SIZE << " bytes." << std::endl;
            // Fall through: this isn't necessarily fatal (yet!)
        }
    }

create_thread:
    ret = pthread_create(&thread, &attr, &runThread, this);
    if (ret) {
        std::cerr << "pthread_create failed for thread id " << thread_id
                  << std::endl;
        exit(1);
    }
}

void Thread::join() { pthread_join(thread, nullptr); }

#else // windows

void Thread::start() { thread = std::thread(&runThread, this); }

void Thread::join() { thread.join(); }

#endif

// Dispatch
void *Thread::runThread(void *thr) {
    if (!no_signal_handler) {
        setSignalStack();
    }
    ((Thread *)thr)->run();
    return nullptr;
}


Thread::Thread(size_t num) : thread_id(num) {}

Thread::~Thread() {}
