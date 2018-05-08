/*
 * Copyright (c) 2018, Intel Corporation
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

#ifndef CH_RUNTIME_H_
#define CH_RUNTIME_H_

#include <stdlib.h>

/**
 * @file
 * @brief The Chimera runtime API definition.
 *
 * Chimera is a hybrid of Hyperscan and PCRE regular expression engine.
 *
 * This header contains functions for using compiled Chimera databases for
 * scanning data at runtime.
 */

#include "hs_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct ch_scratch;

/**
 * A Chimera scratch space.
 */
typedef struct ch_scratch ch_scratch_t;

/**
 * Callback return value used to tell the Chimera matcher what to do after
 * processing this match.
 */
typedef int ch_callback_t;

/**
 * @defgroup CH_CALLBACK ch_callback_t values
 *
 * @{
 */

/**
 * Continue matching.
 */
#define CH_CALLBACK_CONTINUE     0

/**
 * Terminate matching.
 */
#define CH_CALLBACK_TERMINATE    1

/**
 * Skip remaining matches for this ID and continue.
 */
#define CH_CALLBACK_SKIP_PATTERN 2


/** @} */


/**
 * Type used to differentiate the errors raised with the @ref
 * ch_error_event_handler callback.
 */
typedef int ch_error_event_t;

/**
 * @defgroup CH_ERROR_EVENT ch_error_event_t values
 *
 * @{
 */

/**
 * PCRE hits its match limit and reports PCRE_ERROR_MATCHLIMIT.
 */
#define CH_ERROR_MATCHLIMIT      1

/**
 * PCRE hits its recursion limit and reports PCRE_ERROR_RECURSIONLIMIT.
 */
#define CH_ERROR_RECURSIONLIMIT  2

/** @} */

/**
 * Structure representing a captured subexpression within a match. An array of
 * these structures corresponding to capture groups in order is passed to the
 * callback on match, with active structures identified by the
 * CH_CAPTURE_FLAG_ACTIVE flag.
 */
typedef struct ch_capture {
    /**
     * The flags indicating if this structure is active.
     */
    unsigned int flags;

    /**
     * offset at which this capture group begins.
     */
    unsigned long long from; /*< offset at which this capture group begins. */

    /**
     * offset at which this capture group ends.
     */
    unsigned long long to;
} ch_capture_t;

/**
 * @defgroup CH_CAPTURE ch_capture_t flags
 *
 * These flags are used in @ref ch_capture_t::flags to indicate if this
 * structure is active.
 *
 * @{
 */

/**
 * Flag indicating that a particular capture group is inactive, used in @ref
 * ch_capture_t::flags.
 */
#define CH_CAPTURE_FLAG_INACTIVE      0

/**
 * Flag indicating that a particular capture group is active, used in @ref
 * ch_capture_t::flags.
 */
#define CH_CAPTURE_FLAG_ACTIVE      1

/** @} */

/**
 * Definition of the match event callback function type.
 *
 * A callback function matching the defined type must be provided by the
 * application calling the @ref ch_scan()
 *
 * This callback function will be invoked whenever a match is located in the
 * target data during the execution of a scan. The details of the match are
 * passed in as parameters to the callback function, and the callback function
 * should return a value indicating whether or not matching should continue on
 * the target data. If no callbacks are desired from a scan call, NULL may be
 * provided in order to suppress match production.
 *
 * @param id
 *      The ID number of the expression that matched. If the expression was a
 *      single expression compiled with @ref ch_compile(), this value will be
 *      zero.
 *
 * @param from
 *      The offset of the first byte that matches the expression.
 *
 * @param to
 *      The offset after the last byte that matches the expression.
 *
 * @param flags
 *      This is provided for future use and is unused at present.
 *
 * @param size
 *      The number of valid entries pointed to by the captured parameter.
 *
 * @param captured
 *      A pointer to an array of @ref ch_capture_t structures that
 *      contain the start and end offsets of entire pattern match and
 *      each captured subexpression.
 *
 * @param ctx
 *      The pointer supplied by the user to the @ref ch_scan() function.
 *
 * @return
 *      The callback can return @ref CH_CALLBACK_TERMINATE to stop matching.
 *      Otherwise, a return value of @ref CH_CALLBACK_CONTINUE will continue,
 *      with the current pattern if configured to produce multiple matches per
 *      pattern, while a return value of @ref CH_CALLBACK_SKIP_PATTERN will
 *      cease matching this pattern but continue matching the next pattern.
 */
typedef ch_callback_t (HS_CDECL *ch_match_event_handler)(unsigned int id,
                                                unsigned long long from,
                                                unsigned long long to,
                                                unsigned int flags,
                                                unsigned int size,
                                                const ch_capture_t *captured,
                                                void *ctx);

/**
 * Definition of the Chimera error event callback function type.
 *
 * A callback function matching the defined type may be provided by the
 * application calling the @ref ch_scan function. This callback function
 * will be invoked when an error event occurs during matching; this indicates
 * that some matches for a given expression may not be reported.
 *
 * @param error_type
 *      The type of error event that occurred. Currently these errors
 *      correspond to resource limits on PCRE backtracking
 *      @ref CH_ERROR_MATCHLIMIT and @ref CH_ERROR_RECURSIONLIMIT.
 *
 * @param id
 *      The ID number of the expression that matched.
 *
 * @param info
 *      Event-specific data, for future use. Currently unused.
 *
 * @param ctx
 *      The context pointer supplied by the user to the @ref ch_scan
 *      function.
 *
 * @return
 *      The callback can return @ref CH_CALLBACK_SKIP_PATTERN to cease matching
 *      this pattern but continue matching the next pattern. Otherwise, we stop
 *      matching for all patterns with @ref CH_CALLBACK_TERMINATE.
 */
 typedef ch_callback_t (HS_CDECL *ch_error_event_handler)(
                                                 ch_error_event_t error_type,
                                                 unsigned int id, void *info,
                                                 void *ctx);

/**
 * The block regular expression scanner.
 *
 * This is the function call in which the actual pattern matching takes place
 * for block-mode pattern databases.
 *
 * @param db
 *      A compiled pattern database.
 *
 * @param data
 *      Pointer to the data to be scanned.
 *
 * @param length
 *      The number of bytes to scan.
 *
 * @param flags
 *      Flags modifying the behaviour of this function. This parameter is
 *      provided for future use and is unused at present.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref ch_alloc_scratch() for this
 *      database.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param onError
 *      Pointer to a error event callback function. If a NULL pointer is given,
 *      @ref CH_ERROR_MATCHLIMIT and @ref CH_ERROR_RECURSIONLIMIT errors will
 *      be ignored and match will continue.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function.
 *
 * @return
 *      Returns @ref CH_SUCCESS on success; @ref CH_SCAN_TERMINATED if the
 *      match callback indicated that scanning should stop; other values on
 *      error.
 */
ch_error_t HS_CDECL ch_scan(const ch_database_t *db, const char *data,
                            unsigned int length, unsigned int flags,
                            ch_scratch_t *scratch,
                            ch_match_event_handler onEvent,
                            ch_error_event_handler onError,
                            void *context);

/**
 * Allocate a "scratch" space for use by Chimera.
 *
 * This is required for runtime use, and one scratch space per thread, or
 * concurrent caller, is required. Any allocator callback set by @ref
 * ch_set_scratch_allocator() or @ref ch_set_allocator() will be used by this
 * function.
 *
 * @param db
 *      The database, as produced by @ref ch_compile().
 *
 * @param scratch
 *      On first allocation, a pointer to NULL should be provided so a new
 *      scratch can be allocated. If a scratch block has been previously
 *      allocated, then a pointer to it should be passed back in to see if it
 *      is valid for this database block. If a new scratch block is required,
 *      the original will be freed and the new one returned, otherwise the
 *      previous scratch block will be returned. On success, the scratch block
 *      will be suitable for use with the provided database in addition to any
 *      databases that original scratch space was suitable for.
 *
 * @return
 *      @ref CH_SUCCESS on successful allocation; @ref CH_NOMEM if the
 *      allocation fails.  Other errors may be returned if invalid parameters
 *      are specified.
 */
ch_error_t HS_CDECL ch_alloc_scratch(const ch_database_t *db,
                                     ch_scratch_t **scratch);

/**
 * Allocate a scratch space that is a clone of an existing scratch space.
 *
 * This is useful when multiple concurrent threads will be using the same set
 * of compiled databases, and another scratch space is required. Any allocator
 * callback set by @ref ch_set_scratch_allocator() or @ref ch_set_allocator()
 * will be used by this function.
 *
 * @param src
 *      The existing @ref ch_scratch_t to be cloned.
 *
 * @param dest
 *      A pointer to the new scratch space will be returned here.
 *
 * @return
 *      @ref CH_SUCCESS on success; @ref CH_NOMEM if the allocation fails.
 *      Other errors may be returned if invalid parameters are specified.
 */
ch_error_t HS_CDECL ch_clone_scratch(const ch_scratch_t *src,
                                     ch_scratch_t **dest);

/**
 * Provides the size of the given scratch space.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref ch_alloc_scratch() or @ref
 *      ch_clone_scratch().
 *
 * @param scratch_size
 *      On success, the size of the scratch space in bytes is placed in this
 *      parameter.
 *
 * @return
 *      @ref CH_SUCCESS on success, other values on failure.
 */
ch_error_t HS_CDECL ch_scratch_size(const ch_scratch_t *scratch,
                                    size_t *scratch_size);

/**
 * Free a scratch block previously allocated by @ref ch_alloc_scratch() or @ref
 * ch_clone_scratch().
 *
 * The free callback set by @ref ch_set_scratch_allocator() or @ref
 * ch_set_allocator() will be used by this function.
 *
 * @param scratch
 *      The scratch block to be freed. NULL may also be safely provided.
 *
 * @return
 *      @ref CH_SUCCESS on success, other values on failure.
 */
ch_error_t HS_CDECL ch_free_scratch(ch_scratch_t *scratch);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CH_RUNTIME_H_ */
