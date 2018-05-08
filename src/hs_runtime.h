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

#ifndef HS_RUNTIME_H_
#define HS_RUNTIME_H_

#include <stdlib.h>

/**
 * @file
 * @brief The Hyperscan runtime API definition.
 *
 * Hyperscan is a high speed regular expression engine.
 *
 * This header contains functions for using compiled Hyperscan databases for
 * scanning data at runtime.
 */

#include "hs_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Definition of the stream identifier type.
 */
struct hs_stream;

/**
 * The stream identifier returned by @ref hs_open_stream().
 */
typedef struct hs_stream hs_stream_t;

struct hs_scratch;

/**
 * A Hyperscan scratch space.
 */
typedef struct hs_scratch hs_scratch_t;

/**
 * Definition of the match event callback function type.
 *
 * A callback function matching the defined type must be provided by the
 * application calling the @ref hs_scan(), @ref hs_scan_vector() or @ref
 * hs_scan_stream() functions (or other streaming calls which can produce
 * matches).
 *
 * This callback function will be invoked whenever a match is located in the
 * target data during the execution of a scan. The details of the match are
 * passed in as parameters to the callback function, and the callback function
 * should return a value indicating whether or not matching should continue on
 * the target data. If no callbacks are desired from a scan call, NULL may be
 * provided in order to suppress match production.
 *
 * This callback function should not attempt to call Hyperscan API functions on
 * the same stream nor should it attempt to reuse the scratch space allocated
 * for the API calls that caused it to be triggered. Making another call to the
 * Hyperscan library with completely independent parameters should work (for
 * example, scanning a different database in a new stream and with new scratch
 * space), but reusing data structures like stream state and/or scratch space
 * will produce undefined behavior.
 *
 * @param id
 *      The ID number of the expression that matched. If the expression was a
 *      single expression compiled with @ref hs_compile(), this value will be
 *      zero.
 *
 * @param from
 *      - If a start of match flag is enabled for the current pattern, this
 *        argument will be set to the start of match for the pattern assuming
 *        that that start of match value lies within the current 'start of match
 *        horizon' chosen by one of the SOM_HORIZON mode flags.

 *      - If the start of match value lies outside this horizon (possible only
 *        when the SOM_HORIZON value is not @ref HS_MODE_SOM_HORIZON_LARGE),
 *        the @p from value will be set to @ref HS_OFFSET_PAST_HORIZON.

 *      - This argument will be set to zero if the Start of Match flag is not
 *        enabled for the given pattern.
 *
 * @param to
 *      The offset after the last byte that matches the expression.
 *
 * @param flags
 *      This is provided for future use and is unused at present.
 *
 * @param context
 *      The pointer supplied by the user to the @ref hs_scan(), @ref
 *      hs_scan_vector() or @ref hs_scan_stream() function.
 *
 * @return
 *      Non-zero if the matching should cease, else zero. If scanning is
 *      performed in streaming mode and a non-zero value is returned, any
 *      subsequent calls to @ref hs_scan_stream() for that stream will
 *      immediately return with @ref HS_SCAN_TERMINATED.
 */
typedef int (HS_CDECL *match_event_handler)(unsigned int id,
                                            unsigned long long from,
                                            unsigned long long to,
                                            unsigned int flags,
                                            void *context);

/**
 * Open and initialise a stream.
 *
 * @param db
 *      A compiled pattern database.
 *
 * @param flags
 *      Flags modifying the behaviour of the stream. This parameter is provided
 *      for future use and is unused at present.
 *
 * @param stream
 *      On success, a pointer to the generated @ref hs_stream_t will be
 *      returned; NULL on failure.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_open_stream(const hs_database_t *db, unsigned int flags,
                                   hs_stream_t **stream);

/**
 * Write data to be scanned to the opened stream.
 *
 * This is the function call in which the actual pattern matching takes place
 * as data is written to the stream. Matches will be returned via the @ref
 * match_event_handler callback supplied.
 *
 * @param id
 *      The stream ID (returned by @ref hs_open_stream()) to which the data
 *      will be written.
 *
 * @param data
 *      Pointer to the data to be scanned.
 *
 * @param length
 *      The number of bytes to scan.
 *
 * @param flags
 *      Flags modifying the behaviour of the stream. This parameter is provided
 *      for future use and is unused at present.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch().
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param ctxt
 *      The user defined pointer which will be passed to the callback function
 *      when a match occurs.
 *
 * @return
 *      Returns @ref HS_SUCCESS on success; @ref HS_SCAN_TERMINATED if the
 *      match callback indicated that scanning should stop; other values on
 *      error.
 */
hs_error_t HS_CDECL hs_scan_stream(hs_stream_t *id, const char *data,
                                   unsigned int length, unsigned int flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *ctxt);

/**
 * Close a stream.
 *
 * This function completes matching on the given stream and frees the memory
 * associated with the stream state. After this call, the stream pointed to by
 * @p id is invalid and can no longer be used. To reuse the stream state after
 * completion, rather than closing it, the @ref hs_reset_stream function can be
 * used.
 *
 * This function must be called for any stream created with @ref
 * hs_open_stream(), even if scanning has been terminated by a non-zero return
 * from the match callback function.
 *
 * Note: This operation may result in matches being returned (via calls to the
 * match event callback) for expressions anchored to the end of the data stream
 * (for example, via the use of the `$` meta-character). If these matches are
 * not desired, NULL may be provided as the @ref match_event_handler callback.
 *
 * If NULL is provided as the @ref match_event_handler callback, it is
 * permissible to provide a NULL scratch.
 *
 * @param id
 *      The stream ID returned by @ref hs_open_stream().
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch(). This is
 *      allowed to be NULL only if the @p onEvent callback is also NULL.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param ctxt
 *      The user defined pointer which will be passed to the callback function
 *      when a match occurs.
 *
 * @return
 *      Returns @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_close_stream(hs_stream_t *id, hs_scratch_t *scratch,
                                    match_event_handler onEvent, void *ctxt);

/**
 * Reset a stream to an initial state.
 *
 * Conceptually, this is equivalent to performing @ref hs_close_stream() on the
 * given stream, followed by a @ref hs_open_stream(). This new stream replaces
 * the original stream in memory, avoiding the overhead of freeing the old
 * stream and allocating the new one.
 *
 * Note: This operation may result in matches being returned (via calls to the
 * match event callback) for expressions anchored to the end of the original
 * data stream (for example, via the use of the `$` meta-character). If these
 * matches are not desired, NULL may be provided as the @ref match_event_handler
 * callback.
 *
 * Note: the stream will also be tied to the same database.
 *
 * @param id
 *      The stream (as created by @ref hs_open_stream()) to be replaced.
 *
 * @param flags
 *      Flags modifying the behaviour of the stream. This parameter is provided
 *      for future use and is unused at present.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch(). This is
 *      allowed to be NULL only if the @p onEvent callback is also NULL.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function
 *      when a match occurs.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_reset_stream(hs_stream_t *id, unsigned int flags,
                                    hs_scratch_t *scratch,
                                    match_event_handler onEvent, void *context);

/**
 * Duplicate the given stream. The new stream will have the same state as the
 * original including the current stream offset.
 *
 * @param to_id
 *      On success, a pointer to the new, copied @ref hs_stream_t will be
 *      returned; NULL on failure.
 *
 * @param from_id
 *      The stream (as created by @ref hs_open_stream()) to be copied.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_copy_stream(hs_stream_t **to_id,
                                   const hs_stream_t *from_id);

/**
 * Duplicate the given 'from' stream state onto the 'to' stream. The 'to' stream
 * will first be reset (reporting any EOD matches if a non-NULL @p onEvent
 * callback handler is provided).
 *
 * Note: the 'to' stream and the 'from' stream must be open against the same
 * database.
 *
 * @param to_id
 *      On success, a pointer to the new, copied @ref hs_stream_t will be
 *      returned; NULL on failure.
 *
 * @param from_id
 *      The stream (as created by @ref hs_open_stream()) to be copied.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch(). This is
 *      allowed to be NULL only if the @p onEvent callback is also NULL.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function
 *      when a match occurs.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_reset_and_copy_stream(hs_stream_t *to_id,
                                             const hs_stream_t *from_id,
                                             hs_scratch_t *scratch,
                                             match_event_handler onEvent,
                                             void *context);

/**
 * Creates a compressed representation of the provided stream in the buffer
 * provided. This compressed representation can be converted back into a stream
 * state by using @ref hs_expand_stream() or @ref hs_reset_and_expand_stream().
 * The size of the compressed representation will be placed into @p used_space.
 *
 * If there is not sufficient space in the buffer to hold the compressed
 * representation, @ref HS_INSUFFICIENT_SPACE will be returned and @p used_space
 * will be populated with the amount of space required.
 *
 * Note: this function does not close the provided stream, you may continue to
 * use the stream or to free it with @ref hs_close_stream().
 *
 * @param stream
 *      The stream (as created by @ref hs_open_stream()) to be compressed.
 *
 * @param buf
 *      Buffer to write the compressed representation into. Note: if the call is
 *      just being used to determine the amount of space required, it is allowed
 *      to pass NULL here and @p buf_space as 0.
 *
 * @param buf_space
 *      The number of bytes in @p buf. If buf_space is too small, the call will
 *      fail with @ref HS_INSUFFICIENT_SPACE.
 *
 * @param used_space
 *      Pointer to where the amount of used space will be written to. The used
 *      buffer space is always less than or equal to @p buf_space. If the call
 *      fails with @ref HS_INSUFFICIENT_SPACE, this pointer will be used to
 *      write out the amount of buffer space required.
 *
 * @return
 *      @ref HS_SUCCESS on success, @ref HS_INSUFFICIENT_SPACE if the provided
 *      buffer is too small.
 */
hs_error_t HS_CDECL hs_compress_stream(const hs_stream_t *stream, char *buf,
                                       size_t buf_space, size_t *used_space);

/**
 * Decompresses a compressed representation created by @ref hs_compress_stream()
 * into a new stream.
 *
 * Note: @p buf must correspond to a complete compressed representation created
 * by @ref hs_compress_stream() of a stream that was opened against @p db. It is
 * not always possible to detect misuse of this API and behaviour is undefined
 * if these properties are not satisfied.
 *
 * @param db
 *      The compiled pattern database that the compressed stream was opened
 *      against.
 *
 * @param stream
 *      On success, a pointer to the expanded @ref hs_stream_t will be
 *      returned; NULL on failure.
 *
 * @param buf
 *      A compressed representation of a stream. These compressed forms are
 *      created by @ref hs_compress_stream().
 *
 * @param buf_size
 *      The size in bytes of the compressed representation.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_expand_stream(const hs_database_t *db,
                                     hs_stream_t **stream, const char *buf,
                                     size_t buf_size);

/**
 * Decompresses a compressed representation created by @ref hs_compress_stream()
 * on top of the 'to' stream. The 'to' stream will first be reset (reporting
 * any EOD matches if a non-NULL @p onEvent callback handler is provided).
 *
 * Note: the 'to' stream must be opened against the same database as the
 * compressed stream.
 *
 * Note: @p buf must correspond to a complete compressed representation created
 * by @ref hs_compress_stream() of a stream that was opened against @p db. It is
 * not always possible to detect misuse of this API and behaviour is undefined
 * if these properties are not satisfied.
 *
 * @param to_stream
 *      A pointer to a valid stream state. A pointer to the expanded @ref
 *      hs_stream_t will be returned; NULL on failure.
 *
 * @param buf
 *      A compressed representation of a stream. These compressed forms are
 *      created by @ref hs_compress_stream().
 *
 * @param buf_size
 *      The size in bytes of the compressed representation.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch(). This is
 *      allowed to be NULL only if the @p onEvent callback is also NULL.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function
 *      when a match occurs.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_reset_and_expand_stream(hs_stream_t *to_stream,
                                               const char *buf, size_t buf_size,
                                               hs_scratch_t *scratch,
                                               match_event_handler onEvent,
                                               void *context);

/**
 * The block (non-streaming) regular expression scanner.
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
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch() for this
 *      database.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function.
 *
 * @return
 *      Returns @ref HS_SUCCESS on success; @ref HS_SCAN_TERMINATED if the
 *      match callback indicated that scanning should stop; other values on
 *      error.
 */
hs_error_t HS_CDECL hs_scan(const hs_database_t *db, const char *data,
                            unsigned int length, unsigned int flags,
                            hs_scratch_t *scratch, match_event_handler onEvent,
                            void *context);

/**
 * The vectored regular expression scanner.
 *
 * This is the function call in which the actual pattern matching takes place
 * for vectoring-mode pattern databases.
 *
 * @param db
 *      A compiled pattern database.
 *
 * @param data
 *      An array of pointers to the data blocks to be scanned.
 *
 * @param length
 *      An array of lengths (in bytes) of each data block to scan.
 *
 * @param count
 *      Number of data blocks to scan. This should correspond to the size of
 *      of the @p data and @p length arrays.
 *
 * @param flags
 *      Flags modifying the behaviour of this function. This parameter is
 *      provided for future use and is unused at present.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch() for
 *      this database.
 *
 * @param onEvent
 *      Pointer to a match event callback function. If a NULL pointer is given,
 *      no matches will be returned.
 *
 * @param context
 *      The user defined pointer which will be passed to the callback function.
 *
 * @return
 *      Returns @ref HS_SUCCESS on success; @ref HS_SCAN_TERMINATED if the match
 *      callback indicated that scanning should stop; other values on error.
 */
hs_error_t HS_CDECL hs_scan_vector(const hs_database_t *db,
                                   const char *const *data,
                                   const unsigned int *length,
                                   unsigned int count, unsigned int flags,
                                   hs_scratch_t *scratch,
                                   match_event_handler onEvent, void *context);

/**
 * Allocate a "scratch" space for use by Hyperscan.
 *
 * This is required for runtime use, and one scratch space per thread, or
 * concurrent caller, is required. Any allocator callback set by @ref
 * hs_set_scratch_allocator() or @ref hs_set_allocator() will be used by this
 * function.
 *
 * @param db
 *      The database, as produced by @ref hs_compile().
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
 *      @ref HS_SUCCESS on successful allocation; @ref HS_NOMEM if the
 *      allocation fails.  Other errors may be returned if invalid parameters
 *      are specified.
 */
hs_error_t HS_CDECL hs_alloc_scratch(const hs_database_t *db,
                                     hs_scratch_t **scratch);

/**
 * Allocate a scratch space that is a clone of an existing scratch space.
 *
 * This is useful when multiple concurrent threads will be using the same set
 * of compiled databases, and another scratch space is required. Any allocator
 * callback set by @ref hs_set_scratch_allocator() or @ref hs_set_allocator()
 * will be used by this function.
 *
 * @param src
 *      The existing @ref hs_scratch_t to be cloned.
 *
 * @param dest
 *      A pointer to the new scratch space will be returned here.
 *
 * @return
 *      @ref HS_SUCCESS on success; @ref HS_NOMEM if the allocation fails.
 *      Other errors may be returned if invalid parameters are specified.
 */
hs_error_t HS_CDECL hs_clone_scratch(const hs_scratch_t *src,
                                     hs_scratch_t **dest);

/**
 * Provides the size of the given scratch space.
 *
 * @param scratch
 *      A per-thread scratch space allocated by @ref hs_alloc_scratch() or @ref
 *      hs_clone_scratch().
 *
 * @param scratch_size
 *      On success, the size of the scratch space in bytes is placed in this
 *      parameter.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_scratch_size(const hs_scratch_t *scratch,
                                    size_t *scratch_size);

/**
 * Free a scratch block previously allocated by @ref hs_alloc_scratch() or @ref
 * hs_clone_scratch().
 *
 * The free callback set by @ref hs_set_scratch_allocator() or @ref
 * hs_set_allocator() will be used by this function.
 *
 * @param scratch
 *      The scratch block to be freed. NULL may also be safely provided.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_free_scratch(hs_scratch_t *scratch);

/**
 * Callback 'from' return value, indicating that the start of this match was
 * too early to be tracked with the requested SOM_HORIZON precision.
 */
#define HS_OFFSET_PAST_HORIZON    (~0ULL)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HS_RUNTIME_H_ */
