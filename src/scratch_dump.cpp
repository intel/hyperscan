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

#include "config.h"

#include "scratch.h"
#include "scratch_dump.h"
#include "hs_internal.h"
#include "ue2common.h"
#include "util/multibit_build.h"
#include "nfa/nfa_api_queue.h"
#include "rose/rose_internal.h"

#include <cassert>
#include <cstdio>

#ifdef DUMP_SUPPORT

using namespace std;

namespace ue2 {

void dumpScratch(const struct hs_scratch *s, FILE *f) {
    assert(s);

    fprintf(f, "Scratch space required : %u bytes\n", s->scratchSize);
    fprintf(f, "  hs_scratch structure : %zu bytes\n", sizeof(*s));
    fprintf(f, "    tctxt structure    : %zu bytes\n", sizeof(s->tctxt));
    fprintf(f, "  queues               : %zu bytes\n",
            s->queueCount * sizeof(struct mq));
    fprintf(f, "  bStateSize           : %u bytes\n", s->bStateSize);
    fprintf(f, "  active queue array   : %u bytes\n", s->activeQueueArraySize);
    fprintf(f, "  qmpq                 : %zu bytes\n",
            s->queueCount * sizeof(struct queue_match));
    fprintf(f, "  delay info           : %u bytes\n",
            s->delay_fatbit_size * DELAY_SLOT_COUNT);
}

} // namespace ue2

#endif // DUMP_SUPPORT
