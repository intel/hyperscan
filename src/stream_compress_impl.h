/*
 * Copyright (c) 2017-2018, Intel Corporation
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

#include "util/join.h"

#define COPY_FIELD(x) COPY(&x, sizeof(x))
#define COPY_LEFTFIXES JOIN(sc_left_, FN_SUFFIX)
#define COPY_SOM_INFO JOIN(sc_som_, FN_SUFFIX)

static
size_t COPY_LEFTFIXES(const struct RoseEngine *rose, size_t currOffset,
                      STREAM_QUAL struct hs_stream *stream,
                      BUF_QUAL char *buf, UNUSED size_t buf_size) {
    if (!rose->activeLeftIterOffset) {
        return currOffset;
    }

    const struct RoseStateOffsets *so = &rose->stateOffsets;
    STREAM_QUAL char *stream_body
        = ((STREAM_QUAL char *)stream) + sizeof(struct hs_stream);

    /* Note: in the expand case the active left array has already been copied
     * into the stream. */
    const u8 *ara = (const u8 *)(stream_body + so->activeLeftArray);
    const u32 arCount = rose->activeLeftCount;
    const struct LeftNfaInfo *left_table = getLeftTable(rose);

    /* We only want to look at non-transient leftfixes */
    const struct mmbit_sparse_iter *it = getActiveLeftIter(rose);
    struct mmbit_sparse_state si_state[MAX_SPARSE_ITER_STATES];
    u32 dummy;
    u32 ri = mmbit_sparse_iter_begin(ara, arCount, &dummy, it, si_state);
    for (; ri != MMB_INVALID;
         ri = mmbit_sparse_iter_next(ara, arCount, ri, &dummy, it, si_state)) {
        u32 qi = ri + rose->leftfixBeginQueue;
        UNUSED const struct LeftNfaInfo *left = left_table + ri;
        const struct NfaInfo *nfa_info = getNfaInfoByQueue(rose, qi);
        const struct NFA *nfa = getNfaByInfo(rose, nfa_info);

        COPY(stream_body + nfa_info->stateOffset, nfa->streamStateSize);
        /* copy the one whole byte for active leftfixes as well */
        assert(left->lagIndex != ROSE_OFFSET_INVALID);
        COPY(stream_body + so->leftfixLagTable + left->lagIndex, 1);
    }

    return currOffset;
}

static
size_t COPY_SOM_INFO(const struct RoseEngine *rose, size_t currOffset,
                     STREAM_QUAL struct hs_stream *stream,
                     BUF_QUAL char *buf, UNUSED size_t buf_size) {
    const struct RoseStateOffsets *so = &rose->stateOffsets;

    if (!so->somLocation) {
        assert(!so->somValid);
        assert(!so->somWritable);
        return currOffset;
    }

    STREAM_QUAL char *stream_body
        = ((STREAM_QUAL char *)stream) + sizeof(struct hs_stream);

    assert(so->somValid);
    assert(so->somWritable);

    COPY_MULTIBIT(stream_body + so->somWritable, rose->somLocationCount);
    COPY_MULTIBIT(stream_body + so->somValid, rose->somLocationCount);

    /* Copy only the som slots which contain valid values. */
    /* Note: in the expand case the som valid array has been copied in. */
    const u8 *svalid = (const u8 *)(stream_body + so->somValid);
    u32 s_count = rose->somLocationCount;
    u32 s_width = rose->somHorizon;
    for (u32 slot = mmbit_iterate(svalid, s_count, MMB_INVALID);
         slot != MMB_INVALID; slot = mmbit_iterate(svalid, s_count, slot)) {
        COPY(stream_body + so->somLocation + slot * s_width, s_width);
    }

    return currOffset;
}

static
size_t JOIN(sc_, FN_SUFFIX)(const struct RoseEngine *rose,
                            STREAM_QUAL struct hs_stream *stream,
                            BUF_QUAL char *buf, UNUSED size_t buf_size) {
    size_t currOffset = 0;
    const struct RoseStateOffsets *so = &rose->stateOffsets;

    STREAM_QUAL char *stream_body
        = ((STREAM_QUAL char *)stream) + sizeof(struct hs_stream);

    COPY_FIELD(stream->offset);
    ASSIGN(stream->rose, rose);

    COPY(stream_body + ROSE_STATE_OFFSET_STATUS_FLAGS, 1);
    COPY_MULTIBIT(stream_body + ROSE_STATE_OFFSET_ROLE_MMBIT, rose->rolesWithStateCount);

    /* stream is valid in compress/size, and stream->offset has been set already
     * on the expand side */
    u64a offset = stream->offset;
    u32 history = MIN((u32)offset, rose->historyRequired);

    /* copy the active mmbits */
    COPY_MULTIBIT(stream_body + so->activeLeafArray, rose->activeArrayCount);
    COPY_MULTIBIT(stream_body + so->activeLeftArray, rose->activeLeftCount);

    COPY(stream_body + so->longLitState, so->longLitState_size);

    /* Leftlag table will be handled later, for active leftfixes */

    /* anchored table state is not required once we are deep in the stream */
    if (offset <= rose->anchoredDistance) {
        COPY(stream_body + so->anchorState, rose->anchorStateSize);
    }

    COPY(stream_body + so->groups, so->groups_size);

    /* copy the real bits of history */
    UNUSED u32 hend = so->history + rose->historyRequired;
    COPY(stream_body + hend - history, history);

    /* copy the exhaustion multibit */
    COPY_MULTIBIT(stream_body + so->exhausted, rose->ekeyCount);

    /* copy the logical multibit */
    COPY_MULTIBIT(stream_body + so->logicalVec,
                  rose->lkeyCount + rose->lopCount);

    /* copy the combination multibit */
    COPY_MULTIBIT(stream_body + so->combVec, rose->ckeyCount);

    /* copy nfa stream state for endfixes */
    /* Note: in the expand case the active array has already been copied into
     * the stream. */
    const u8 *aa = (const u8 *)(stream_body + so->activeLeafArray);
    u32 aaCount = rose->activeArrayCount;
    for (u32 qi = mmbit_iterate(aa, aaCount, MMB_INVALID); qi != MMB_INVALID;
         qi = mmbit_iterate(aa, aaCount, qi)) {
        DEBUG_PRINTF("saving stream state for qi=%u\n", qi);
        const struct NfaInfo *nfa_info = getNfaInfoByQueue(rose, qi);
        const struct NFA *nfa = getNfaByInfo(rose, nfa_info);
        COPY(stream_body + nfa_info->stateOffset, nfa->streamStateSize);
    }

    /* copy nfa stream state for leftfixes */
    currOffset = COPY_LEFTFIXES(rose, currOffset, stream, buf, buf_size);
    if (!currOffset) {
        return 0;
    }

    currOffset = COPY_SOM_INFO(rose, currOffset, stream, buf, buf_size);
    if (!currOffset) {
        return 0;
    }

    return currOffset;
}

#undef ASSIGN
#undef COPY
#undef COPY_FIELD
#undef COPT_LEFTFIXES
#undef COPY_MULTIBIT
#undef COPY_SOM_INFO
#undef FN_SUFFIX
#undef BUF_QUAL
#undef STREAM_QUAL
