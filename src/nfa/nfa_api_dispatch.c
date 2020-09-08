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

/** \file
    \brief Dispatches NFA engine API calls to the appropriate engines
*/
#include "nfa_api.h"

#include "nfa_api_queue.h"
#include "nfa_internal.h"
#include "ue2common.h"

// Engine implementations.
#include "castle.h"
#include "gough.h"
#include "lbr.h"
#include "limex.h"
#include "mcclellan.h"
#include "mcsheng.h"
#include "mpv.h"
#include "sheng.h"
#include "tamarama.h"

#define DISPATCH_CASE(dc_ltype, dc_ftype, dc_func_call)                        \
    case dc_ltype:                                                             \
        return nfaExec##dc_ftype##dc_func_call;                                \
    break

// general framework calls

#define DISPATCH_BY_NFA_TYPE(dbnt_func)                                        \
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

char nfaCheckFinalState(const struct NFA *nfa, const char *state,
                        const char *streamState, u64a offset,
                        NfaCallback callback, void *context) {
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    // Caller should avoid calling us if we can never produce matches.
    assert(nfaAcceptsEod(nfa));

    DISPATCH_BY_NFA_TYPE(_testEOD(nfa, state, streamState, offset, callback,
                                  context));
    return 0;
}

char nfaQueueInitState(const struct NFA *nfa, struct mq *q) {
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    DISPATCH_BY_NFA_TYPE(_queueInitState(nfa, q));
    return 0;
}

static really_inline
char nfaQueueExec_i(const struct NFA *nfa, struct mq *q, s64a end) {
    DISPATCH_BY_NFA_TYPE(_Q(nfa, q, end));
    return 0;
}

static really_inline
char nfaQueueExec2_i(const struct NFA *nfa, struct mq *q, s64a end) {
    DISPATCH_BY_NFA_TYPE(_Q2(nfa, q, end));
    return 0;
}

char nfaQueueExec_raw(const struct NFA *nfa, struct mq *q, s64a end) {
    return nfaQueueExec_i(nfa, q, end);
}

char nfaQueueExec2_raw(const struct NFA *nfa, struct mq *q, s64a end) {
    return nfaQueueExec2_i(nfa, q, end);
}

static really_inline
char nfaQueueExecRose_i(const struct NFA *nfa, struct mq *q, ReportID report) {
    DISPATCH_BY_NFA_TYPE(_QR(nfa, q, report));
    return 0;
}

/** Returns 0 if this NFA cannot possibly match (due to width constraints etc)
 * and the caller should return 0. May also edit the queue. */
static really_inline
char nfaQueueCanMatch(const struct NFA *nfa, struct mq *q, s64a end,
                      char *q_trimmed) {
    assert(q_trimmed);
    assert(q->end - q->cur >= 2);
    assert(end >= 0);

    DEBUG_PRINTF("q->offset=%llu, end=%lld\n", q->offset, end);
    DEBUG_PRINTF("maxBiAnchoredWidth=%u, maxOffset=%u\n",
                 nfa->maxBiAnchoredWidth, nfa->maxOffset);

    if (nfa->maxBiAnchoredWidth &&
            (end + q->offset > nfa->maxBiAnchoredWidth)) {
        DEBUG_PRINTF("stream too long: o %llu l %zu max: %hhu\n", q->offset,
                     q->length, nfa->maxBiAnchoredWidth);
        return 0;
    }

    if (nfa->maxOffset) {
        if (q->offset >= nfa->maxOffset) {
            DEBUG_PRINTF("stream is past maxOffset\n");
            return 0;
        }

        if (q->offset + end > nfa->maxOffset) {
            s64a maxEnd = nfa->maxOffset - q->offset;
            DEBUG_PRINTF("me %lld off %llu len = %lld\n", maxEnd,
                         q->offset, end);
            while (q->end > q->cur
                   && q->items[q->end - 1].location > maxEnd) {
                *q_trimmed = 1;
                DEBUG_PRINTF("killing item %u %lld %u\n", q->end,
                              q->items[q->end - 1].location,
                              q->items[q->end - 1].type);
                q->items[q->end - 1].location = maxEnd;
                q->items[q->end - 1].type = MQE_END;
                if (q->end - q->cur < 2
                     ||q->items[q->end - 2].location <= maxEnd) {
                    break;
                }
                q->end--;
            }

            if (q->end - q->cur < 2) { /* nothing left on q */
                DEBUG_PRINTF("queue empty\n");
                return 0;
            }
        }

#ifdef DEBUG
        if (*q_trimmed) {
            debugQueue(q);
        }
#endif
    }

    return 1;
}

char nfaQueueExec(const struct NFA *nfa, struct mq *q, s64a end) {
    DEBUG_PRINTF("nfa=%p end=%lld\n", nfa, end);
#ifdef DEBUG
    debugQueue(q);
#endif

    assert(q && q->context && q->state);
    assert(end >= 0);
    assert(q->cur < q->end);
    assert(q->end <= MAX_MQE_LEN);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));
    assert(end < q->items[q->end - 1].location
           || q->items[q->end - 1].type == MQE_END);

    if (q->items[q->cur].location > end) {
        return 1;
    }

    char q_trimmed = 0;

    assert(end <= (s64a)q->length || !q->hlength);
    /* due to reverse accel in block mode some queues may work on a truncated
     * buffer */
    if (end > (s64a)q->length) {
        end = q->length;
        q_trimmed = 1;
    }

    if (!nfaQueueCanMatch(nfa, q, end, &q_trimmed)) {
        if (q->report_current) {
            nfaReportCurrentMatches(nfa, q);
            q->report_current = 0;
        }

        return 0;
    }

    char rv = nfaQueueExec_i(nfa, q, end);

#ifdef DEBUG
    debugQueue(q);
#endif

    assert(!q->report_current);
    DEBUG_PRINTF("returned rv=%d, q_trimmed=%d\n", rv, q_trimmed);
    return rv && !q_trimmed;
}

char nfaQueueExecToMatch(const struct NFA *nfa, struct mq *q, s64a end) {
    DEBUG_PRINTF("nfa=%p end=%lld\n", nfa, end);
#ifdef DEBUG
    debugQueue(q);
#endif

    assert(q);
    assert(end >= 0);
    assert(q->state);
    assert(q->cur < q->end);
    assert(q->end <= MAX_MQE_LEN);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));
    assert(end < q->items[q->end - 1].location
           || q->items[q->end - 1].type == MQE_END);

    char q_trimmed_ra = 0;
    assert(end <= (s64a)q->length || !q->hlength);
    /* due to reverse accel in block mode some queues may work on a truncated
     * buffer */
    if (q->items[q->cur].location > end) {
        return 1;
    }

    if (end > (s64a)q->length) {
        end = q->length;
        q_trimmed_ra = 1;
    }

    char q_trimmed = 0;
    if (!nfaQueueCanMatch(nfa, q, end, &q_trimmed)) {
        if (q->report_current) {
            nfaReportCurrentMatches(nfa, q);
            q->report_current = 0;
        }

        return 0;
    }

    char rv = nfaQueueExec2_i(nfa, q, end);
    assert(!q->report_current);
    DEBUG_PRINTF("returned rv=%d, q_trimmed=%d\n", rv, q_trimmed);
    if (rv == MO_MATCHES_PENDING) {
        if (q_trimmed) {
            // We need to "fix" the queue so that subsequent operations must
            // trim it as well.
            assert(q->end > 0);
            assert(nfa->maxOffset);
            q->items[q->end - 1].location = nfa->maxOffset + 1;
        }
        return rv;
    }
    return rv && !q_trimmed && !q_trimmed_ra;
}

char nfaReportCurrentMatches(const struct NFA *nfa, struct mq *q) {
    DISPATCH_BY_NFA_TYPE(_reportCurrent(nfa, q));
    return 0;
}

char nfaInAcceptState(const struct NFA *nfa, ReportID report, struct mq *q) {
    DISPATCH_BY_NFA_TYPE(_inAccept(nfa, report, q));
    return 0;
}

char nfaInAnyAcceptState(const struct NFA *nfa, struct mq *q) {
    DISPATCH_BY_NFA_TYPE(_inAnyAccept(nfa, q));
    return 0;
}

char nfaQueueExecRose(const struct NFA *nfa, struct mq *q, ReportID r) {
    DEBUG_PRINTF("nfa=%p\n", nfa);
#ifdef DEBUG
    debugQueue(q);
#endif

    assert(q && !q->context && q->state);
    assert(q->cur <= q->end);
    assert(q->end <= MAX_MQE_LEN);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));
    assert(!q->report_current);

    return nfaQueueExecRose_i(nfa, q, r);
}

char nfaBlockExecReverse(const struct NFA *nfa, u64a offset, const u8 *buf,
                         size_t buflen, const u8 *hbuf, size_t hlen,
                         NfaCallback callback, void *context) {
    assert(nfa);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    DISPATCH_BY_NFA_TYPE(_B_Reverse(nfa, offset, buf, buflen, hbuf, hlen,
                                    callback, context));
    return 0;
}

char nfaQueueCompressState(const struct NFA *nfa, const struct mq *q,
                           s64a loc) {
    assert(nfa && q);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    DISPATCH_BY_NFA_TYPE(_queueCompressState(nfa, q, loc));
    return 0;
}

char nfaExpandState(const struct NFA *nfa, void *dest, const void *src,
                    u64a offset, u8 key) {
    assert(nfa && dest && src);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    DISPATCH_BY_NFA_TYPE(_expandState(nfa, dest, src, offset, key));
    return 0;
}

char nfaInitCompressedState(const struct NFA *nfa, u64a offset, void *state,
                            u8 key) {
    assert(nfa && state);
    assert(ISALIGNED_CL(nfa) && ISALIGNED_CL(getImplNfa(nfa)));

    DISPATCH_BY_NFA_TYPE(_initCompressedState(nfa, offset, state, key));
    return 0;
}

enum nfa_zombie_status nfaGetZombieStatus(const struct NFA *nfa, struct mq *q,
                                          s64a loc) {
    DISPATCH_BY_NFA_TYPE(_zombie_status(nfa, q, loc));
    return NFA_ZOMBIE_NO;
}
