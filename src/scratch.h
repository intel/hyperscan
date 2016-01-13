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

/** \file
 * \brief Scratch and associated data structures.
 *
 * This header gets pulled into many places (many deep, slow to compile
 * places). Try to keep the included headers under control.
 */

#ifndef SCRATCH_H_DA6D4FC06FF410
#define SCRATCH_H_DA6D4FC06FF410

#include "ue2common.h"
#include "rose/rose_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

UNUSED static const u32 SCRATCH_MAGIC = 0x544F4259;
#define FDR_TEMP_BUF_SIZE 200

struct fatbit;
struct hs_scratch;
struct RoseEngine;
struct mq;

struct queue_match {
    /** \brief used to store the current location of an (suf|out)fix match in
     * the current buffer.
     *
     * As (suf|out)fixes always run in the main buffer and never in history
     * this number will always be positive (matches at 0 belong to previous
     * write). Hence we can get away with a size_t rather than the usual s64a
     * for a location. */
    size_t loc;

    u32 queue; /**< queue index. */
};

struct catchup_pq {
    struct queue_match *qm;
    u32 qm_size; /**< current size of the priority queue */
};


/** \brief Value indicating a stream is active (not broken). */
#define NOT_BROKEN       0

/** \brief Value indicating that the user has requested that matching be
 * terminated. */
#define BROKEN_FROM_USER 1

/** \brief Value indicating that all possible matches on this stream have been
 * raised (i.e. all its exhaustion keys are on.) */
#define BROKEN_EXHAUSTED 2

/** \brief Core information about the current scan, used everywhere. */
struct core_info {
    void *userContext; /**< user-supplied context */

    /** \brief user-supplied match callback */
    int (*userCallback)(unsigned int id, unsigned long long from,
                        unsigned long long to, unsigned int flags, void *ctx);

    const struct RoseEngine *rose;
    char *state; /**< full stream state */
    char *exhaustionVector; /**< pointer to evec for this stream */
    char broken;  /**< user told us to stop, or exhausted */
    const u8 *buf; /**< main scan buffer */
    size_t len; /**< length of main scan buffer in bytes */
    const u8 *hbuf; /**< history buffer */
    size_t hlen; /**< length of history buffer in bytes. */
    u64a buf_offset; /**< stream offset, for the base of the buffer */
};

/** \brief Rose state information. */
struct RoseContext {
    u8 mpv_inactive;
    u64a groups;
    u64a lit_offset_adjust; /**< offset to add to matches coming from hwlm */
    u64a delayLastEndOffset; /**< end of the last match from FDR used by delay
                              * code */
    u64a lastEndOffset; /**< end of the last match from FDR/anchored DFAs used
                         * by history code. anchored DFA matches update this
                         * when they are inserted into the literal match
                         * stream */
    u64a lastMatchOffset; /**< last match offset report up out of rose;
                           * used _only_ for debugging, asserts */
    u64a minMatchOffset; /**< the earliest offset that we are still allowed to
                          * report */
    u64a minNonMpvMatchOffset; /**< the earliest offset that non-mpv engines are
                                * still allowed to report */
    u64a next_mpv_offset; /**< earliest offset that the MPV can next report a
                           * match, cleared if top events arrive */
    RoseCallback cb;
    RoseCallbackSom cb_som;
    u32 filledDelayedSlots;
    u32 curr_anchored_loc;   /**< last read/written row */
    u32 curr_row_offset; /**< last read/written entry */
    u32 curr_qi;    /**< currently executing main queue index during
                     * \ref nfaQueueExec */
};

struct match_deduper {
    struct fatbit *log[2]; /**< even, odd logs */
    struct fatbit *som_log[2]; /**< even, odd fatbit logs for som */
    u64a *som_start_log[2]; /**< even, odd start offset logs for som */
    u32 log_size;
    u64a current_report_offset;
    u8 som_log_dirty;
};

/** \brief Hyperscan scratch region header.
 *
 * NOTE: there is no requirement that scratch is 16-byte aligned, as it is
 * allocated by a malloc equivalent, possibly supplied by the user.
 */
struct ALIGN_CL_DIRECTIVE hs_scratch {
    u32 magic;
    char *scratch_alloc; /* user allocated scratch object */
    u32 queueCount;
    u32 bStateSize; /**< sizeof block mode states */
    u32 tStateSize; /**< sizeof transient rose states */
    u32 fullStateSize; /**< size of uncompressed nfa state */
    struct RoseContext tctxt;
    char *bstate; /**< block mode states */
    char *tstate; /**< state for transient roses */
    char *qNfaState; /**< queued NFA temp state */
    void *nfaContext; /**< use for your NFAContextNNN struct */
    void *nfaContextSom; /**< use for your NFAContextNNN struct by som_runtime */
    char *fullState; /**< uncompressed NFA state */
    struct mq *queues;
    struct fatbit *aqa; /**< active queue array; fatbit of queues that are valid
                         * & active */
    struct fatbit **delay_slots;
    struct fatbit **am_log;
    struct fatbit **al_log;
    u64a am_log_sum;
    u64a al_log_sum;
    struct catchup_pq catchup_pq;
    struct core_info core_info;
    struct match_deduper deduper;
    u32 anchored_region_len;
    u32 anchored_region_width;
    u32 anchored_literal_region_len;
    u32 anchored_literal_count;
    u32 delay_count;
    u32 scratchSize;
    u8 ALIGN_DIRECTIVE fdr_temp_buf[FDR_TEMP_BUF_SIZE];
    u32 handledKeyCount;
    struct fatbit *handled_roles; /**< fatbit of ROLES (not states) already
                                   * handled by this literal */
    u64a *som_store; /**< array of som locations */
    u64a *som_attempted_store; /**< array of som locations for fail stores */
    struct fatbit *som_set_now; /**< fatbit, true if the som location was set
                                 * based on a match at the current offset */
    struct fatbit *som_attempted_set; /**< fatbit, true if the som location
                            * would have been set at the current offset if the
                            * location had been writable */
    u64a som_set_now_offset; /**< offset at which som_set_now represents */
    u32 som_store_count;
};

static really_inline
struct hs_scratch *tctxtToScratch(struct RoseContext *tctxt) {
    return (struct hs_scratch *)
        ((char *)tctxt - offsetof(struct hs_scratch, tctxt));
}

static really_inline
struct fatbit **getAnchoredLog(struct hs_scratch *scratch) {
    return scratch->am_log;
}

/* array of fatbit ptr; TODO: why not an array of fatbits? */
static really_inline
struct fatbit **getAnchoredLiteralLog(struct hs_scratch *scratch) {
    return scratch->al_log;
}

static really_inline
struct fatbit **getDelaySlots(struct hs_scratch *scratch) {
    return scratch->delay_slots;
}

static really_inline
char told_to_stop_matching(const struct hs_scratch *scratch) {
    return scratch->core_info.broken == BROKEN_FROM_USER;
}

static really_inline
char can_stop_matching(const struct hs_scratch *scratch) {
    return scratch->core_info.broken != NOT_BROKEN;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SCRATCH_H_DA6D4FC06FF410 */

