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

#ifndef UTIL_SCATTER_RUNTIME_H
#define UTIL_SCATTER_RUNTIME_H

#include "scatter.h"

#include "uniform_ops.h"

#define SCATTER_DEF(t)                                                        \
static really_inline                                                          \
void scatter_##t(void *out, const struct scatter_unit_##t *plan, u32 count) { \
    for (u32 i = 0; i < count; i++) {                                         \
        const struct scatter_unit_##t *item = plan + i;                       \
        DEBUG_PRINTF("storing %llu into offset %u\n", (u64a)item->val, \
                     item->offset);                                     \
        storeu_##t((char *)out + item->offset, item->val);              \
    }                                                                         \
}

SCATTER_DEF(u64a)
SCATTER_DEF(u32)
SCATTER_DEF(u16)
SCATTER_DEF(u8)

#undef SCATTER_DEF

static really_inline
void scatter(void *out, const void *base, const struct scatter_full_plan *p) {
#define RUN_SUB(t)                                      \
    if (p->s_##t##_offset) {                            \
        assert(p->s_##t##_count);                       \
        const struct scatter_unit_##t *pp               \
            = (const void *)(b + p->s_##t##_offset);    \
        scatter_##t(out, pp, p->s_##t##_count);         \
    }

    const char *b = base;

    RUN_SUB(u64a);
    RUN_SUB(u32);
    RUN_SUB(u16);
    RUN_SUB(u8);

#undef RUN_SUB
}

#endif
