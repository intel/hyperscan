/*
 * Copyright (c) 2018-2020, Intel Corporation
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
 * \brief Chimera: main runtime.
 */

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hs.h"
#include "hs_internal.h"
#include "ue2common.h"
#include "ch_database.h"
#include "ch_internal.h"
#include "ch_scratch.h"
#include "util/multibit.h"
#include "util/unicode_def.h"

typedef struct queue_item PQ_T;

static
char PQ_COMP(PQ_T *pqc_items, int a, int b) {
    if ((pqc_items)[a].to != (pqc_items)[b].to) {
        return (pqc_items)[a].to < (pqc_items)[b].to;
    } else if ((pqc_items)[a].from != (pqc_items)[b].from) {
        return (pqc_items)[a].from < (pqc_items)[b].from;
    } else {
        return (pqc_items)[a].id < (pqc_items)[b].id;
    }
}

static
char PQ_COMP_B(PQ_T *pqc_items, int a, PQ_T b_fixed) {
    if ((pqc_items)[a].to != (b_fixed).to) {
        return (pqc_items)[a].to < (b_fixed).to;
    } else if ((pqc_items)[a].from != (b_fixed).from) {
        return (pqc_items)[a].from < (b_fixed).from;
    } else {
        return (pqc_items)[a].id < b_fixed.id;
    }
}

#include "util/pqueue.h"

static really_inline
void pq_insert_with(struct match_pq *pq, int from, int to, u32 id) {
    DEBUG_PRINTF("inserting pattern%u in pq at %u\n", id, to);
    struct queue_item temp = {
        .from = from,
        .to = to,
        .id = id,
    };

    pq_insert(pq->item, pq->size, temp);
    ++pq->size;
}

static really_inline
void pq_pop_nice(struct match_pq *pq) {
    pq_pop(pq->item, pq->size);
    pq->size--;
}

/** dummy event handler for use when user does not provide one */
static
int HS_CDECL null_onEvent(UNUSED unsigned id, UNUSED unsigned long long from,
                 UNUSED unsigned long long to, UNUSED unsigned flags,
                 UNUSED unsigned size, UNUSED const ch_capture_t *captured,
                 UNUSED void *ctxt) {
    return 0;
}

/** \brief Chimera runtime context. */
struct HybridContext {
    const char *data; //!< buffer being scanned
    u32 length; //!< length of data buffer
    u32 valid_utf8_highwater; //!< UTF-8 has been validated up to here.
    const struct ch_bytecode *db;
    struct ch_scratch *scratch;
    struct match_pq *pq;
    /** \brief user-supplied match callback */
    int (HS_CDECL *match_callback)(unsigned int id, unsigned long long from,
                          unsigned long long to, unsigned int flags,
                          unsigned int size, const ch_capture_t *capture,
                          void *ctx);
    /** \brief user-supplied error callback */
    int (HS_CDECL *error_callback)(ch_error_event_t error_type, unsigned int id,
                          void *info, void *ctx);
    /** \brief user-supplied context */
    void *context;
};

// Internal PCRE func.
extern int _pcre_valid_utf(const unsigned char *, int, int *);

/** UTF-8 validity check. Returns >0 if the given region of the data is valid
 * UTF-8, 0 otherwise. */
static
char isValidUTF8(struct HybridContext *hyctx, u32 end) {
    assert(hyctx);

    if (hyctx->valid_utf8_highwater >= end) {
        return 1; // Already validated.
    }

    const unsigned char *data =
        (const unsigned char *)hyctx->data + hyctx->valid_utf8_highwater;
    int validate_len = end - hyctx->valid_utf8_highwater;

    DEBUG_PRINTF("validating %d bytes\n", validate_len);

    int erroroffset = 0;
    if (_pcre_valid_utf(data, validate_len, &erroroffset)) {
        DEBUG_PRINTF("UTF8 invalid at offset %d\n", erroroffset);
        return 0;
    }

    hyctx->valid_utf8_highwater = end;
    return 1;
}

static
const pcre *getPcre(const struct ch_pattern *pattern) {
    const char *ptr = (const char *)pattern;
    const pcre *p = (const pcre *)(ptr + ROUNDUP_N(sizeof(*pattern), 8));
    assert(ISALIGNED_N(p, 8));
    return p;
}

/** \brief Fill the Chimera groups array from a pcre_exec ovector. */
static
void fillGroupsFromOvector(ch_capture_t *groups, int numPairs, int *ovector) {
    assert(groups);
    assert(ISALIGNED_N(groups, alignof(ch_capture_t)));

    DEBUG_PRINTF("filling %d groups (@ %p) from pcre ovector\n",
                 numPairs, groups);

    for (int i = 0; i < numPairs * 2; i += 2) {
        if (ovector[i] == -1) {
            groups->flags = CH_CAPTURE_FLAG_INACTIVE;
        } else {
            groups->flags = CH_CAPTURE_FLAG_ACTIVE;
            assert(ovector[i] <= ovector[i + 1]);
            groups->from = ovector[i];
            groups->to = ovector[i + 1];
        }
        ++groups;
    }
}

static
ch_error_t handlePcreNonMatch(const struct ch_pattern *pattern, int rv,
                              ch_error_event_handler onError,
                              void *userContext) {
    assert(rv < 0);

    if (rv == PCRE_ERROR_NOMATCH) {
        DEBUG_PRINTF("no match found by libpcre\n");
        return CH_SUCCESS;
    } else if (rv == PCRE_ERROR_MATCHLIMIT) {
        DEBUG_PRINTF("pcre hit match limit\n");
        if (onError) {
            return onError(CH_ERROR_MATCHLIMIT, pattern->id, NULL,
                           userContext);
        }
        return CH_SUCCESS;
    } else if (rv == PCRE_ERROR_RECURSIONLIMIT) {
        DEBUG_PRINTF("pcre hit recursion limit\n");
        if (onError) {
            return onError(CH_ERROR_RECURSIONLIMIT, pattern->id, NULL,
                           userContext);
        }
        return CH_SUCCESS;
    }

    // All other errors not handled above are fatal.
    return CH_FAIL_INTERNAL;
}

static
ch_error_t scanPcre(struct HybridContext *hyctx, UNUSED unsigned int length,
                    unsigned int offset, u32 id) {
    const char *data = hyctx->data;
    unsigned int full_length = hyctx->length;
    ch_error_event_handler onError = hyctx->error_callback;
    void *userContext = hyctx->context;

    const struct ch_pattern *pattern = getPattern(hyctx->db, id);
    const pcre *p = getPcre(pattern);

    // Set up the PCRE extra block.
    const pcre_extra *extra = &pattern->extra;

    int startoffset = offset;

    int *ovector = hyctx->scratch->ovector;
    int ovectorSize = (hyctx->scratch->maxCaptureGroups + 1) * 3;
    assert(ovectorSize >= 2);

    DEBUG_PRINTF("scanning %u bytes, pattern %u, startoffset %d\n",
                 length, id, startoffset);

    int options = 0;
    if (pattern->flags & CHIMERA_PATTERN_FLAG_UTF8) {
        // We do our own UTF-8 validation.
        options |= PCRE_NO_UTF8_CHECK;
        if (!isValidUTF8(hyctx, full_length)) {
            return handlePcreNonMatch(pattern, PCRE_ERROR_BADUTF8, onError,
                                      userContext);
        }
    }

    int rv = pcre_exec(p, extra, data, full_length, startoffset, options,
                       ovector, ovectorSize);

    DEBUG_PRINTF("pcre return code is %d\n", rv);

    // Handle all non-match or error cases, all of which involve us
    // terminating the loop.
    if (rv < 0) {
        return handlePcreNonMatch(pattern, rv, onError, userContext);
    }

    // We've found a match, and we should always have room for at least the
    // start and end offsets in our ovector. Pass this info to the user.
    assert(rv >= 1);
    assert(rv < ovectorSize);
    int from = ovector[0];
    int to = ovector[1];
    DEBUG_PRINTF("match %d -> %d\n", from, to);

    struct ch_patterndata *pd = hyctx->scratch->patternData + id;

    if (hyctx->db->flags & CHIMERA_FLAG_GROUPS) {
        fillGroupsFromOvector(pd->match, rv, ovector);
    } else {
        rv = 0;
    }
    pd->groupCount = (u32)rv;

    // Insert new matched item to the queue
    pq_insert_with(hyctx->pq, from, to, id);

    // Next scan starts at the first codepoint after the match.  It's
    // possible that we have a vacuous match, in which case we must step
    // past it to ensure that we always progress.
    if (from != to) {
        startoffset = to;
    } else if (pattern->flags & CHIMERA_PATTERN_FLAG_UTF8) {
        startoffset = to + 1;
        while (startoffset < (int)full_length &&
               ((data[startoffset] & 0xc0) == UTF_CONT_BYTE_HEADER)) {
            ++startoffset;
        }
    } else {
        startoffset = to + 1;
    }

    pd->scanStart = startoffset;
    DEBUG_PRINTF("new offset %u\n", pd->scanStart);

    return CH_SUCCESS;
}

static
ch_error_t catchupPcre(struct HybridContext *hyctx, unsigned int id,
                       unsigned long long from, unsigned long long to) {
    ch_match_event_handler onEvent = hyctx->match_callback;
    void *userContext = hyctx->context;
    DEBUG_PRINTF("priority queue size %u\n", hyctx->pq->size);
    while (hyctx->pq->size) {
        u32 num_item = hyctx->pq->size;
        struct queue_item *item = pq_top(hyctx->pq->item);
        size_t top_from = item->from;
        size_t top_to = item->to;
        u32 top_id = item->id;

        if (top_to > to) {
            pq_insert_with(hyctx->pq, from, to, id);
            break;
        }
        pq_pop_nice(hyctx->pq);

        const struct ch_pattern *pattern = getPattern(hyctx->db, top_id);
        struct ch_patterndata *pd = hyctx->scratch->patternData + top_id;

        // Report match for pattern
        DEBUG_PRINTF("trigger match@%zu\n", top_to);
        ch_callback_t cbrv =
            onEvent(pattern->id, top_from, top_to, 0 /* flags */,
                    pd->groupCount, pd->match, userContext);

        if (cbrv == CH_CALLBACK_TERMINATE) {
            DEBUG_PRINTF("user callback told us to terminate scanning\n");
            return CH_SCAN_TERMINATED;
        } else if (cbrv == CH_CALLBACK_SKIP_PATTERN) {
            DEBUG_PRINTF("user callback told us to skip this pattern\n");
            pd->scanStart = hyctx->length;
        }

        if (top_id == id) {
            break;
        }

        // Push a new match to replace the old one
        unsigned int start = pd->scanStart;
        unsigned int len = hyctx->length - pd->scanStart;
        if (hyctx->length >= pd->scanStart &&
            !(pattern->flags & CHIMERA_PATTERN_FLAG_SINGLEMATCH)) {
            DEBUG_PRINTF("get a new match item\n");
            int ret = scanPcre(hyctx, len, start, top_id);

            if (ret == CH_CALLBACK_TERMINATE) {
                DEBUG_PRINTF("user callback told us to terminate scanning\n");
                return CH_SCAN_TERMINATED;
            } else if (ret == CH_CALLBACK_SKIP_PATTERN) {
                DEBUG_PRINTF("user callback told us to skip this pattern\n");
                pd->scanStart = hyctx->length;
                ret = CH_SUCCESS;
            } else if (ret == CH_FAIL_INTERNAL) {
                return ret;
            }

            // No further match is found
            if (hyctx->pq->size == num_item - 1) {
                pd->scanStart = hyctx->length;
            }
        }
    }

    return CH_SUCCESS;
}

/** \brief Callback used for internal Hyperscan multi-matcher. */
static
int HS_CDECL multiCallback(unsigned int id, unsigned long long from,
                  unsigned long long to, UNUSED unsigned int flags,
                  void *ctx) {
    assert(ctx);
    struct HybridContext *hyctx = ctx;

    DEBUG_PRINTF("match for ID %u at offset %llu\n", id, to);
    assert(id < hyctx->db->patternCount);

    const struct ch_pattern *pattern = getPattern(hyctx->db, id);
    struct ch_patterndata *pd = hyctx->scratch->patternData + id;
    char needConfirm = pattern->fixedWidth == ~0U;

    if (needConfirm &&
        mmbit_isset(hyctx->scratch->active, hyctx->db->patternCount, id)) {
        if ((hyctx->db->flags & CHIMERA_FLAG_ALL_CONFIRM) &&
             mmbit_all(hyctx->scratch->active, hyctx->db->patternCount)) {
            return 1;
        }
        return 0;
    }
    // Store the fact that we've seen this bit.
    char already = mmbit_set(hyctx->scratch->active,
                             hyctx->db->patternCount, id);
    DEBUG_PRINTF("match from %u to %llu\n", pd->scanStart, to);

    if (!already) {
        pd->scanStart = 0;
    } else if (to < pd->scanStart + pattern->minWidth) {
        return 0;
    } else if (pattern->flags & CHIMERA_PATTERN_FLAG_SINGLEMATCH) {
        if ((hyctx->db->flags & CHIMERA_FLAG_ALL_SINGLE) &&
             mmbit_all(hyctx->scratch->active, hyctx->db->patternCount)) {
            return 1;
        }
        // Note: we may have unordered match from Hyperscan,
        // thus possibly get to < pd->scanStart.
        return 0;
    }

    int ret = HS_SUCCESS;
    unsigned int start = pd->scanStart;
    unsigned int len = hyctx->length - pd->scanStart;
    assert(hyctx->length >= pd->scanStart);
    const char *data = hyctx->data;
    if (needConfirm) {
        DEBUG_PRINTF("run confirm for the first time\n");
        ret = scanPcre(hyctx, len, start, id);
        hyctx->scratch->ret = ret;
        if (ret == CH_CALLBACK_TERMINATE) {
            DEBUG_PRINTF("user callback told us to terminate scanning\n");
            return HS_SCAN_TERMINATED;
        } else if (ret == CH_CALLBACK_SKIP_PATTERN) {
            DEBUG_PRINTF("user callback told us to skip this pattern\n");
            pd->scanStart = hyctx->length;
            ret = HS_SUCCESS;
            hyctx->scratch->ret = ret;
        } else if (ret == CH_FAIL_INTERNAL) {
            return ret;
        }
    } else {
        if (already) {
            DEBUG_PRINTF("catch up with new matches\n");
            ret = catchupPcre(hyctx, id, from, to);

            hyctx->scratch->ret = ret;
            if (pd->scanStart >= hyctx->length) {
                return ret;
            }
        }
        int startoffset = 0;
        // Next scan starts at the first codepoint after the match.  It's
        // possible that we have a vacuous match, in which case we must step
        // past it to ensure that we always progress.
        if (from != to) {
            startoffset = to;
        } else if (pattern->flags & CHIMERA_PATTERN_FLAG_UTF8) {
            startoffset = to + 1;
            while (startoffset < (int)hyctx->length &&
                   ((data[startoffset] & 0xc0) == UTF_CONT_BYTE_HEADER)) {
                ++startoffset;
            }
        } else {
            startoffset = to + 1;
        }
        pd->scanStart = startoffset;
        int rv = 0;
        if (hyctx->db->flags & CHIMERA_FLAG_GROUPS) {
            ch_capture_t *groups = pd->match;
            groups->flags = CH_CAPTURE_FLAG_ACTIVE;
            groups->from = from;
            groups->to = to;
            rv = 1;
        }
        pd->groupCount = (u32)rv;
        pq_insert_with(hyctx->pq, from, to, id);
    }

    return ret;
}

static
hs_error_t scanHyperscan(struct HybridContext *hyctx, const char *data,
                         unsigned int length) {
    DEBUG_PRINTF("scanning %u bytes with Hyperscan\n", length);
    const struct ch_bytecode *hydb = hyctx->db;
    const hs_database_t *db = getHyperscanDatabase(hydb);
    hs_scratch_t *scratch = hyctx->scratch->multi_scratch;

    hs_error_t err = hs_scan(db, data, length, 0, scratch, multiCallback,
                             hyctx);

    return err;
}

/** \brief Init match priority queue.
 *
 * Add a first match offset for each pattern that is not supported by Hyperscan
 * with prefiltering.
 */
static really_inline
ch_error_t initQueue(struct HybridContext *hyctx, struct match_pq *pq) {
    const struct ch_bytecode *db = hyctx->db;

    u8 *active = hyctx->scratch->active;
    mmbit_clear(active, db->patternCount);

    // Init match queue size
    pq->size = 0;

    unsigned int length = hyctx->length;
    const u32 *unguarded = getUnguarded(db);
    for (u32 i = 0; i < db->unguardedCount; i++) {
        u32 patternId = unguarded[i];
        DEBUG_PRINTF("switch on unguarded pcre %u\n", patternId);
        mmbit_set(active, db->patternCount, patternId);

        DEBUG_PRINTF("get a new match item\n");
        int ret = scanPcre(hyctx, length, 0, patternId);

        struct ch_patterndata *pd = hyctx->scratch->patternData + patternId;
        if (ret == CH_CALLBACK_TERMINATE) {
            DEBUG_PRINTF("user callback told us to terminate scanning\n");
            return CH_SCAN_TERMINATED;
        } else if (ret == CH_CALLBACK_SKIP_PATTERN) {
            DEBUG_PRINTF("user callback told us to skip this pattern\n");
            pd->scanStart = length;
            ret = CH_SUCCESS;
        } else if (ret == CH_FAIL_INTERNAL) {
            return ret;
        }
    }

    return CH_SUCCESS;
}

static really_inline
ch_error_t ch_scan_i(const ch_database_t *hydb,
                     const char *data, unsigned int length,
                     UNUSED unsigned int flags,
                     ch_scratch_t *scratch,
                     ch_match_event_handler onEvent,
                     ch_error_event_handler onError,
                     void *userContext) {
    if (unlikely(!hydb || !scratch || !data)) {
        DEBUG_PRINTF("args invalid\n");
        return CH_INVALID;
    }
    ch_error_t ret = hydbIsValid(hydb);
    if (ret != CH_SUCCESS) {
        DEBUG_PRINTF("database invalid\n");
        return ret;
    }

    if (!ISALIGNED_CL(scratch)) {
        DEBUG_PRINTF("bad alignment %p\n", scratch);
        return CH_INVALID;
    }

    if (scratch->magic != CH_SCRATCH_MAGIC) {
        DEBUG_PRINTF("scratch invalid\n");
        return CH_INVALID;
    }

    if (unlikely(markScratchInUse(scratch))) {
        return CH_SCRATCH_IN_USE;
    }

    // Hyperscan underlying scratch and database validity will be checked by
    // the hs_scan() call, so no need to do it here.

    // PCRE takes the data region length in as an int, so this limits our block
    // size to INT_MAX.
    if (length > INT_MAX) {
        DEBUG_PRINTF("length invalid\n");
        unmarkScratchInUse(scratch);
        return CH_INVALID;
    }

    const struct ch_bytecode *db = ch_get_bytecode(hydb);

    scratch->pq.size = 0;
    scratch->ret = CH_SUCCESS;

    // Firstly, we run Hyperscan in block mode and add its matches into the
    // active list for subsequent confirmation with pcre.
    struct HybridContext hyctx = {
        .data = data,
        .length = length,
        .valid_utf8_highwater = 0,
        .db = db,
        .scratch = scratch,
        .pq = &scratch->pq,
        .match_callback = onEvent ? onEvent : null_onEvent,
        .error_callback = onError,
        .context = userContext
    };

    // Init priority queue.
    ret = initQueue(&hyctx, &scratch->pq);
    if (ret != CH_SUCCESS) {
        DEBUG_PRINTF("Chimera returned error %d\n", ret);
        unmarkScratchInUse(scratch);
        return ret;
    }

    if (!(db->flags & CHIMERA_FLAG_NO_MULTIMATCH)) {
        ret = scanHyperscan(&hyctx, data, length);
        // Errors from pcre scan.
        if (scratch->ret == CH_CALLBACK_TERMINATE) {
            DEBUG_PRINTF("Pcre terminates scan\n");
            unmarkScratchInUse(scratch);
            return CH_SCAN_TERMINATED;
        } else if (scratch->ret != CH_SUCCESS) {
            DEBUG_PRINTF("Pcre internal error\n");
            unmarkScratchInUse(scratch);
            return scratch->ret;
        }
        // Errors from Hyperscan scan. Note Chimera could terminate
        // Hyperscan callback on purpose so this is not counted as an error.
        if (ret != HS_SUCCESS && ret != HS_SCAN_TERMINATED) {
            assert(scratch->ret == CH_SUCCESS);
            DEBUG_PRINTF("Hyperscan returned error %d\n", ret);
            unmarkScratchInUse(scratch);
            return ret;
        }
    }

    DEBUG_PRINTF("Flush priority queue\n");
    // Catch up with PCRE and make up id and offsets as we don't really care
    // about their values
    ret = catchupPcre(&hyctx, ~0U, length, length);
    if (ret != CH_SUCCESS) {
        DEBUG_PRINTF("PCRE catch up returned error %d\n", ret);
        unmarkScratchInUse(scratch);
        return ret;
    }

    unmarkScratchInUse(scratch);
    return CH_SUCCESS;
}

HS_PUBLIC_API
ch_error_t HS_CDECL ch_scan(const ch_database_t *hydb, const char *data,
                            unsigned int length, unsigned int flags,
                            ch_scratch_t *scratch,
                            ch_match_event_handler onEvent,
                            ch_error_event_handler onError, void *userContext) {
    ch_error_t ret = ch_scan_i(hydb, data, length, flags, scratch, onEvent,
                               onError, userContext);

    return ret;
}

HS_PUBLIC_API
const char * HS_CDECL ch_version(void) {
    return HS_VERSION_STRING;
}
