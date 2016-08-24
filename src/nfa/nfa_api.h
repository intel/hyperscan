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
 * \brief Declarations for the main NFA Engine API.
 *
 * This file provides the internal API for all runtime engines ("NFAs", even if
 * they're not strictly NFA implementations).
 */

#ifndef NFA_API_H
#define NFA_API_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "callback.h"
#include "ue2common.h"

struct mq;
struct NFA;

/**
 * Indicates if an nfa is a zombie. Note: that there were plans for a more
 * nuanced view of zombiehood but this never eventuated.
 */
enum nfa_zombie_status {
    NFA_ZOMBIE_NO, /**< nfa is not a zombie and will respond to top events */
    NFA_ZOMBIE_ALWAYS_YES /**< nfa is a zombie and will always be a zombie */
};

/**
 * Compresses an engine's state.
 * The expanded state (@ref mq::state, @ref mq::streamState) is reduced purely
 * to a corresponding compressed stream state (@ref mq::streamState).
 *
 * @param nfa engine the state belongs to
 * @param q queue for the engine. The final compressed stream stream is placed
 *        in the location indicated by @ref mq::streamState
 * @param loc the location corresponding to the engine's current state
 */
char nfaQueueCompressState(const struct NFA *nfa, const struct mq *q, s64a loc);

/**
 * Expands an engine's compressed stream state, into its scratch space
 * representation. This is required before an engine starts operating over its
 * queue.
 *
 * @param nfa engine the state belongs to
 * @param dest location in scratch for decompressed state
 * @param src compressed stream state
 * @param offset the current stream offset.
 * @param key byte corresponding to the location where the compressed state was
 *        created.
 */
char nfaExpandState(const struct NFA *nfa, void *dest, const void *src,
                    u64a offset, u8 key);

/**
 * Gives us a properly initialised dead state suitable for later @ref
 * nfaQueueExec calls.
 */
char nfaQueueInitState(const struct NFA *nfa, struct mq *q);

/**
 * Initialise the state, applying a TOP appropriate for the offset. If the
 * NFA becomes inactive, return zero. Otherwise, write out its compressed
 * representation to `state' and return non-zero.
 *
 * @param nfa engine the state belongs to
 * @param offset offset in the stream (relative to start of stream)
 * @param state pointer indicating where the state is to be written
 * @param key byte corresponding to the location where the compressed state is
 *        to be created.
 */
char nfaInitCompressedState(const struct NFA *nfa, u64a offset, void *state,
                            u8 key);

/**
 * Process the queued commands on the given NFA.
 *
 * @param nfa the NFA to execute
 * @param q the queued commands. It must start with some variant of start and
 *        end with some variant of end. The location field of the events must
 *        be monotonically increasing.
 * @param end stop processing command queue when we reach this point
 *
 * @return non-zero if the nfa is still active, if the nfa is not active the
 *         state data is undefined
 *
 * Note: this function can not process events from the past: the location field
 * of each event must be >= current offset.
 */
char nfaQueueExec(const struct NFA *nfa, struct mq *q, s64a end);

/**
 * Main execution function that doesn't perform the checks and optimisations of
 * nfaQueueExec() and just dispatches directly to the nfa implementations. It is
 * intended to be used by the Tamarama engine.
 */
char nfaQueueExec_raw(const struct NFA *nfa, struct mq *q, s64a end);

/** Return value indicating that the engine is dead. */
#define MO_DEAD 0

/** Return value indicating that the engine is alive. */
#define MO_ALIVE 1

/** Return value from @ref nfaQueueExecToMatch indicating that engine progress
 * stopped as a match state was reached. */
#define MO_MATCHES_PENDING 2

/**
 * Process the queued commands on the given nfa up to end or the first match.
 * This function will only fire the callback in response to an report_current
 * being set and accepts at the starting offset, in all other situations accepts
 * will result in the queue pausing with a return value of
 * @ref MO_MATCHES_PENDING.
 *
 * @param nfa the NFA to execute
 * @param q the queued commands. It must start with some variant of start and
 *        end with some variant of end. The location field of the events must
 *        be monotonically increasing. If not all the data was processed during
 *        the call, the queue is updated to reflect the remaining work.
 * @param end stop processing command queue when we reach this point
 *
 * @return @ref MO_ALIVE if the nfa is still active with no matches pending,
 *         and @ref MO_MATCHES_PENDING if there are matches pending, 0 if not
 *         alive
 *
 * Note: if it can be determined that the stream can never match, the nfa
 * may be reported as dead even if not all the data was scanned
 *
 * Note: if the nfa is not alive the state data is undefined
 *
 * Note: this function can not process events from the past: the location field
 * of each event must be >= current offset.
 */
char nfaQueueExecToMatch(const struct NFA *nfa, struct mq *q, s64a end);

/**
 * Main execution function that doesn't perform the checks and optimisations of
 * nfaQueueExecToMatch() and just dispatches directly to the nfa
 * implementations. It is intended to be used by the Tamarama engine.
 */
char nfaQueueExec2_raw(const struct NFA *nfa, struct mq *q, s64a end);

/**
 * Report matches at the current queue location.
 *
 * @param nfa the NFA to execute
 * @param q the queued commands. It must start with some variant of start and
 *         end with some variant of end. The location field of the events must
 *         be monotonically increasing.
 *
 * Note: the queue MUST be located at position where @ref nfaQueueExecToMatch
 *       returned @ref MO_MATCHES_PENDING.
 *
 * Note: the return value of this call is undefined, and should be ignored.
 */
char nfaReportCurrentMatches(const struct NFA *nfa, struct mq *q);

/**
 * Returns non-zero if the NFA is in an accept state with the given report ID.
 */
char nfaInAcceptState(const struct NFA *nfa, ReportID report, struct mq *q);

/**
 * Returns non-zero if the NFA is in any accept state regardless of report
 * ID.
 */
char nfaInAnyAcceptState(const struct NFA *nfa, struct mq *q);

/**
 * Process the queued commands on the given NFA up to end or the first match.
 *
 * Note: This version is meant for rose prefix/infix NFAs:
 *  - never uses a callback
 *  - loading of state at a point in history is not special cased
 *
 * @param nfa the NFA to execute
 * @param q the queued commands. It must start with some variant of start and
 *        end with some variant of end. The location field of the events must
 *        be monotonically increasing. If not all the data was processed during
 *        the call, the queue is updated to reflect the remaining work.
 * @param report we are interested in. If the given report will be raised at
 *        the end location, the function returns @ref MO_MATCHES_PENDING. If no
 *        match information is desired, MO_INVALID_IDX should be passed in.
 * @return @ref MO_ALIVE if the nfa is still active with no matches pending,
 *         and @ref MO_MATCHES_PENDING if there are matches pending, 0 if not
 *         alive
 *
 * Note: if it can be determined that the stream can never match, the nfa
 * may be reported as dead even if not all the data was scanned
 *
 * Note: if the NFA is not active the state data is undefined.
 */
char nfaQueueExecRose(const struct NFA *nfa, struct mq *q, ReportID report);

/**
 * Runs an NFA in reverse from (buf + buflen) to buf and then from (hbuf + hlen)
 * to hbuf (main buffer and history buffer).
 *
 * Note: provides the match location as the "end" offset when the callback is
 * called.
 *
 * @param nfa engine to run
 * @param offset base offset of buf
 * @param buf main buffer
 * @param buflen length of buf
 * @param hbuf history buf
 * @param hlen length of hbuf
 * @param callback the callback to call for each match raised
 * @param context context pointer passed to each callback
 */
char nfaBlockExecReverse(const struct NFA *nfa, u64a offset, const u8 *buf,
                         size_t buflen, const u8 *hbuf, size_t hlen,
                         NfaCallback callback, void *context);

/**
 * Check whether the given NFA's state indicates that it is in one or more
 * final (accept at end of data) state. If so, call the callback for each
 * match.
 *
 * @param nfa the NFA to execute
 * @param state current state associated with this NFA
 * @param streamState stream version of the state associated with this NFA
 *        (including br region)
 * @param offset the offset to return (via the callback) with each match
 * @param callback the callback to call for each match raised
 * @param context context pointer passed to each callback
 *
 * @return @ref MO_HALT_MATCHING if the user instructed us to halt, otherwise
 *         @ref MO_CONTINUE_MATCHING.
 */
char nfaCheckFinalState(const struct NFA *nfa, const char *state,
                        const char *streamState, u64a offset,
                        NfaCallback callback, void *context);

/**
 * Indicates if an engine is a zombie.
 *
 * @param nfa engine to consider
 * @param q queue corresponding to the engine
 * @param loc current location in the buffer for an engine
 */
enum nfa_zombie_status nfaGetZombieStatus(const struct NFA *nfa, struct mq *q,
                                          s64a loc);
#ifdef __cplusplus
}       /* extern "C" */
#endif

#endif
