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

#ifndef HS_COMPILE_H_
#define HS_COMPILE_H_

/**
 * @file
 * @brief The Hyperscan compiler API definition.
 *
 * Hyperscan is a high speed regular expression engine.
 *
 * This header contains functions for compiling regular expressions into
 * Hyperscan databases that can be used by the Hyperscan runtime.
 */

#include "hs_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * A type containing error details that is returned by the compile calls (@ref
 * hs_compile(), @ref hs_compile_multi() and @ref hs_compile_ext_multi()) on
 * failure. The caller may inspect the values returned in this type to
 * determine the cause of failure.
 *
 * Common errors generated during the compile process include:
 *
 *    - *Invalid parameter*
 *
 *      An invalid argument was specified in the compile call.
 *
 *    - *Unrecognised flag*
 *
 *      An unrecognised value was passed in the flags argument.
 *
 *    - *Pattern matches empty buffer*
 *
 *      By default, Hyperscan only supports patterns that will *always*
 *      consume at least one byte of input. Patterns that do not have this
 *      property (such as `/(abc)?/`) will produce this error unless
 *      the @ref HS_FLAG_ALLOWEMPTY flag is supplied. Note that such
 *      patterns will produce a match for *every* byte when scanned.
 *
 *    - *Embedded anchors not supported*
 *
 *      Hyperscan only supports the use of anchor meta-characters (such as
 *      `^` and `$`) in patterns where they could *only* match
 *      at the start or end of a buffer. A pattern containing an embedded
 *      anchor, such as `/abc^def/`, can never match, as there is no
 *      way for `abc` to precede the start of the data stream.
 *
 *    - *Bounded repeat is too large*
 *
 *      The pattern contains a repeated construct with very large finite
 *      bounds.
 *
 *    - *Unsupported component type*
 *
 *      An unsupported PCRE construct was used in the pattern.
 *
 *    - *Unable to generate bytecode*
 *
 *      This error indicates that Hyperscan was unable to compile a pattern
 *      that is syntactically valid. The most common cause is a pattern that is
 *      very long and complex or contains a large repeated subpattern.
 *
 *    - *Unable to allocate memory*
 *
 *      The library was unable to allocate temporary storage used during
 *      compilation time.
 *
 *    - *Allocator returned misaligned memory*
 *
 *      The memory allocator (either malloc() or the allocator set with @ref
 *      hs_set_allocator()) did not correctly return memory suitably aligned
 *      for the largest representable data type on this platform.
 *
 *    - *Internal error*
 *
 *      An unexpected error occurred: if this error is reported, please contact
 *      the Hyperscan team with a description of the situation.
 */
typedef struct hs_compile_error {
    /**
     * A human-readable error message describing the error.
     */
    char *message;

    /**
     * The zero-based number of the expression that caused the error (if this
     * can be determined). If the error is not specific to an expression, then
     * this value will be less than zero.
     */
    int expression;
} hs_compile_error_t;

/**
 * A type containing information on the target platform which may optionally be
 * provided to the compile calls (@ref hs_compile(), @ref hs_compile_multi(),
 * @ref hs_compile_ext_multi()).
 *
 * A hs_platform_info structure may be populated for the current platform by
 * using the @ref hs_populate_platform() call.
 */
typedef struct hs_platform_info {
    /**
     * Information about the target platform which may be used to guide the
     * optimisation process of the compile.
     *
     * Use of this field does not limit the processors that the resulting
     * database can run on, but may impact the performance of the resulting
     * database.
     */
    unsigned int tune;

    /**
     * Relevant CPU features available on the target platform
     *
     * This value may be produced by combining HS_CPU_FEATURE_* flags (such as
     * @ref HS_CPU_FEATURES_AVX2). Multiple CPU features may be or'ed together
     * to produce the value.
     */
    unsigned long long cpu_features;

    /**
     * Reserved for future use.
     */
    unsigned long long reserved1;

    /**
     * Reserved for future use.
     */
    unsigned long long reserved2;
} hs_platform_info_t;

/**
 * A type containing information related to an expression that is returned by
 * @ref hs_expression_info() or @ref hs_expression_ext_info.
 */
typedef struct hs_expr_info {
    /**
     * The minimum length in bytes of a match for the pattern.
     *
     * Note: in some cases when using advanced features to suppress matches
     * (such as extended parameters or the @ref HS_FLAG_SINGLEMATCH flag) this
     * may represent a conservative lower bound for the true minimum length of
     * a match.
     */
    unsigned int min_width;

    /**
     * The maximum length in bytes of a match for the pattern. If the pattern
     * has an unbounded maximum length, this will be set to the maximum value
     * of an unsigned int (UINT_MAX).
     *
     * Note: in some cases when using advanced features to suppress matches
     * (such as extended parameters or the @ref HS_FLAG_SINGLEMATCH flag) this
     * may represent a conservative upper bound for the true maximum length of
     * a match.
     */
    unsigned int max_width;

    /**
     * Whether this expression can produce matches that are not returned in
     * order, such as those produced by assertions. Zero if false, non-zero if
     * true.
     */
    char unordered_matches;

    /**
     * Whether this expression can produce matches at end of data (EOD). In
     * streaming mode, EOD matches are raised during @ref hs_close_stream(),
     * since it is only when @ref hs_close_stream() is called that the EOD
     * location is known. Zero if false, non-zero if true.
     *
     * Note: trailing `\b` word boundary assertions may also result in EOD
     * matches as end-of-data can act as a word boundary.
     */
    char matches_at_eod;

    /**
     * Whether this expression can *only* produce matches at end of data (EOD).
     * In streaming mode, all matches for this expression are raised during
     * @ref hs_close_stream(). Zero if false, non-zero if true.
     */
    char matches_only_at_eod;
} hs_expr_info_t;

/**
 * A structure containing additional parameters related to an expression,
 * passed in at build time to @ref hs_compile_ext_multi() or @ref
 * hs_expression_ext_info.
 *
 * These parameters allow the set of matches produced by a pattern to be
 * constrained at compile time, rather than relying on the application to
 * process unwanted matches at runtime.
 */
typedef struct hs_expr_ext {
    /**
     * Flags governing which parts of this structure are to be used by the
     * compiler. See @ref HS_EXT_FLAG.
     */
    unsigned long long flags;

    /**
     * The minimum end offset in the data stream at which this expression
     * should match successfully. To use this parameter, set the
     * @ref HS_EXT_FLAG_MIN_OFFSET flag in the hs_expr_ext::flags field.
     */
    unsigned long long min_offset;

    /**
     * The maximum end offset in the data stream at which this expression
     * should match successfully. To use this parameter, set the
     * @ref HS_EXT_FLAG_MAX_OFFSET flag in the hs_expr_ext::flags field.
     */
    unsigned long long max_offset;

    /**
     * The minimum match length (from start to end) required to successfully
     * match this expression. To use this parameter, set the
     * @ref HS_EXT_FLAG_MIN_LENGTH flag in the hs_expr_ext::flags field.
     */
    unsigned long long min_length;

    /**
     * Allow patterns to approximately match within this edit distance. To use
     * this parameter, set the @ref HS_EXT_FLAG_EDIT_DISTANCE flag in the
     * hs_expr_ext::flags field.
     */
    unsigned edit_distance;

    /**
     * Allow patterns to approximately match within this Hamming distance. To
     * use this parameter, set the @ref HS_EXT_FLAG_HAMMING_DISTANCE flag in the
     * hs_expr_ext::flags field.
     */
    unsigned hamming_distance;
} hs_expr_ext_t;

/**
 * @defgroup HS_EXT_FLAG hs_expr_ext_t flags
 *
 * These flags are used in @ref hs_expr_ext_t::flags to indicate which fields
 * are used.
 *
 * @{
 */

/** Flag indicating that the hs_expr_ext::min_offset field is used. */
#define HS_EXT_FLAG_MIN_OFFSET      1ULL

/** Flag indicating that the hs_expr_ext::max_offset field is used. */
#define HS_EXT_FLAG_MAX_OFFSET      2ULL

/** Flag indicating that the hs_expr_ext::min_length field is used. */
#define HS_EXT_FLAG_MIN_LENGTH      4ULL

/** Flag indicating that the hs_expr_ext::edit_distance field is used. */
#define HS_EXT_FLAG_EDIT_DISTANCE   8ULL

/** Flag indicating that the hs_expr_ext::hamming_distance field is used. */
#define HS_EXT_FLAG_HAMMING_DISTANCE 16ULL

/** @} */

/**
 * The basic regular expression compiler.
 *
 * This is the function call with which an expression is compiled into a
 * Hyperscan database which can be passed to the runtime functions (such as
 * @ref hs_scan(), @ref hs_open_stream(), etc.)
 *
 * @param expression
 *      The NULL-terminated expression to parse. Note that this string must
 *      represent ONLY the pattern to be matched, with no delimiters or flags;
 *      any global flags should be specified with the @p flags argument. For
 *      example, the expression `/abc?def/i` should be compiled by providing
 *      `abc?def` as the @p expression, and @ref HS_FLAG_CASELESS as the @a
 *      flags.
 *
 * @param flags
 *      Flags which modify the behaviour of the expression. Multiple flags may
 *      be used by ORing them together. Valid values are:
 *       - HS_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - HS_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - HS_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - HS_FLAG_SINGLEMATCH - Only one match will be generated for the
 *                               expression per stream.
 *       - HS_FLAG_ALLOWEMPTY - Allow expressions which can match against an
 *                              empty string, such as `.*`.
 *       - HS_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - HS_FLAG_UCP - Use Unicode properties for character classes.
 *       - HS_FLAG_PREFILTER - Compile pattern in prefiltering mode.
 *       - HS_FLAG_SOM_LEFTMOST - Report the leftmost start of match offset
 *                                when a match is found.
 *
 * @param mode
 *      Compiler mode flags that affect the database as a whole. One of @ref
 *      HS_MODE_STREAM or @ref HS_MODE_BLOCK or @ref HS_MODE_VECTORED must be
 *      supplied, to select between the generation of a streaming, block or
 *      vectored database. In addition, other flags (beginning with HS_MODE_)
 *      may be supplied to enable specific features. See @ref HS_MODE_FLAG for
 *      more details.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref hs_free_database() function.
 *
 * @param error
 *      If the compile fails, a pointer to a @ref hs_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      hs_free_compile_error() function.
 *
 * @return
 *      @ref HS_SUCCESS is returned on successful compilation; @ref
 *      HS_COMPILER_ERROR on failure, with details provided in the error
 *      parameter.
 */
hs_error_t HS_CDECL hs_compile(const char *expression, unsigned int flags,
                               unsigned int mode,
                               const hs_platform_info_t *platform,
                               hs_database_t **db, hs_compile_error_t **error);

/**
 * The multiple regular expression compiler.
 *
 * This is the function call with which a set of expressions is compiled into a
 * database which can be passed to the runtime functions (such as @ref
 * hs_scan(), @ref hs_open_stream(), etc.) Each expression can be labelled with
 * a unique integer which is passed into the match callback to identify the
 * pattern that has matched.
 *
 * @param expressions
 *      Array of NULL-terminated expressions to compile. Note that (as for @ref
 *      hs_compile()) these strings must contain only the pattern to be
 *      matched, with no delimiters or flags. For example, the expression
 *      `/abc?def/i` should be compiled by providing `abc?def` as the first
 *      string in the @p expressions array, and @ref HS_FLAG_CASELESS as the
 *      first value in the @p flags array.
 *
 * @param flags
 *      Array of flags which modify the behaviour of each expression. Multiple
 *      flags may be used by ORing them together.  Specifying the NULL pointer
 *      in place of an array will set the flags value for all patterns to zero.
 *      Valid values are:
 *       - HS_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - HS_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - HS_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - HS_FLAG_SINGLEMATCH - Only one match will be generated by patterns
 *                               with this match id per stream.
 *       - HS_FLAG_ALLOWEMPTY - Allow expressions which can match against an
 *                              empty string, such as `.*`.
 *       - HS_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - HS_FLAG_UCP - Use Unicode properties for character classes.
 *       - HS_FLAG_PREFILTER - Compile pattern in prefiltering mode.
 *       - HS_FLAG_SOM_LEFTMOST - Report the leftmost start of match offset
 *                                when a match is found.
 *
 * @param ids
 *      An array of integers specifying the ID number to be associated with the
 *      corresponding pattern in the expressions array. Specifying the NULL
 *      pointer in place of an array will set the ID value for all patterns to
 *      zero.
 *
 * @param elements
 *      The number of elements in the input arrays.
 *
 * @param mode
 *      Compiler mode flags that affect the database as a whole. One of @ref
 *      HS_MODE_STREAM or @ref HS_MODE_BLOCK or @ref HS_MODE_VECTORED must be
 *      supplied, to select between the generation of a streaming, block or
 *      vectored database. In addition, other flags (beginning with HS_MODE_)
 *      may be supplied to enable specific features. See @ref HS_MODE_FLAG for
 *      more details.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref hs_free_database() function.
 *
 * @param error
 *      If the compile fails, a pointer to a @ref hs_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      hs_free_compile_error() function.
 *
 * @return
 *      @ref HS_SUCCESS is returned on successful compilation; @ref
 *      HS_COMPILER_ERROR on failure, with details provided in the @p error
 *      parameter.
 *
 */
hs_error_t HS_CDECL hs_compile_multi(const char *const *expressions,
                                     const unsigned int *flags,
                                     const unsigned int *ids,
                                     unsigned int elements, unsigned int mode,
                                     const hs_platform_info_t *platform,
                                     hs_database_t **db,
                                     hs_compile_error_t **error);

/**
 * The multiple regular expression compiler with extended parameter support.
 *
 * This function call compiles a group of expressions into a database in the
 * same way as @ref hs_compile_multi(), but allows additional parameters to be
 * specified via an @ref hs_expr_ext_t structure per expression.
 *
 * @param expressions
 *      Array of NULL-terminated expressions to compile. Note that (as for @ref
 *      hs_compile()) these strings must contain only the pattern to be
 *      matched, with no delimiters or flags. For example, the expression
 *      `/abc?def/i` should be compiled by providing `abc?def` as the first
 *      string in the @p expressions array, and @ref HS_FLAG_CASELESS as the
 *      first value in the @p flags array.
 *
 * @param flags
 *      Array of flags which modify the behaviour of each expression. Multiple
 *      flags may be used by ORing them together. Specifying the NULL pointer
 *      in place of an array will set the flags value for all patterns to zero.
 *      Valid values are:
 *       - HS_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - HS_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - HS_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - HS_FLAG_SINGLEMATCH - Only one match will be generated by patterns
 *                               with this match id per stream.
 *       - HS_FLAG_ALLOWEMPTY - Allow expressions which can match against an
 *                              empty string, such as `.*`.
 *       - HS_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - HS_FLAG_UCP - Use Unicode properties for character classes.
 *       - HS_FLAG_PREFILTER - Compile pattern in prefiltering mode.
 *       - HS_FLAG_SOM_LEFTMOST - Report the leftmost start of match offset
 *                                when a match is found.
 *
 * @param ids
 *      An array of integers specifying the ID number to be associated with the
 *      corresponding pattern in the expressions array. Specifying the NULL
 *      pointer in place of an array will set the ID value for all patterns to
 *      zero.
 *
 * @param ext
 *      An array of pointers to filled @ref hs_expr_ext_t structures that
 *      define extended behaviour for each pattern. NULL may be specified if no
 *      extended behaviour is needed for an individual pattern, or in place of
 *      the whole array if it is not needed for any expressions. Memory used by
 *      these structures must be both allocated and freed by the caller.
 *
 * @param elements
 *      The number of elements in the input arrays.
 *
 * @param mode
 *      Compiler mode flags that affect the database as a whole. One of @ref
 *      HS_MODE_STREAM, @ref HS_MODE_BLOCK or @ref HS_MODE_VECTORED must be
 *      supplied, to select between the generation of a streaming, block or
 *      vectored database. In addition, other flags (beginning with HS_MODE_)
 *      may be supplied to enable specific features. See @ref HS_MODE_FLAG for
 *      more details.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref hs_free_database() function.
 *
 * @param error
 *      If the compile fails, a pointer to a @ref hs_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      hs_free_compile_error() function.
 *
 * @return
 *      @ref HS_SUCCESS is returned on successful compilation; @ref
 *      HS_COMPILER_ERROR on failure, with details provided in the @p error
 *      parameter.
 *
 */
hs_error_t HS_CDECL hs_compile_ext_multi(const char *const *expressions,
                                const unsigned int *flags,
                                const unsigned int *ids,
                                const hs_expr_ext_t *const *ext,
                                unsigned int elements, unsigned int mode,
                                const hs_platform_info_t *platform,
                                hs_database_t **db, hs_compile_error_t **error);

/**
 * Free an error structure generated by @ref hs_compile(), @ref
 * hs_compile_multi() or @ref hs_compile_ext_multi().
 *
 * @param error
 *      The @ref hs_compile_error_t to be freed. NULL may also be safely
 *      provided.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_free_compile_error(hs_compile_error_t *error);

/**
 * Utility function providing information about a regular expression. The
 * information provided in @ref hs_expr_info_t includes the minimum and maximum
 * width of a pattern match.
 *
 * Note: successful analysis of an expression with this function does not imply
 * that compilation of the same expression (via @ref hs_compile(), @ref
 * hs_compile_multi() or @ref hs_compile_ext_multi()) would succeed. This
 * function may return @ref HS_SUCCESS for regular expressions that Hyperscan
 * cannot compile.
 *
 * Note: some per-pattern flags (such as @ref HS_FLAG_ALLOWEMPTY, @ref
 * HS_FLAG_SOM_LEFTMOST) are accepted by this call, but as they do not affect
 * the properties returned in the @ref hs_expr_info_t structure, they will not
 * affect the outcome of this function.
 *
 * @param expression
 *      The NULL-terminated expression to parse. Note that this string must
 *      represent ONLY the pattern to be matched, with no delimiters or flags;
 *      any global flags should be specified with the @p flags argument.  For
 *      example, the expression `/abc?def/i` should be compiled by providing
 *      `abc?def` as the @p expression, and @ref HS_FLAG_CASELESS as the @a
 *      flags.
 *
 * @param flags
 *      Flags which modify the behaviour of the expression. Multiple flags may
 *      be used by ORing them together. Valid values are:
 *       - HS_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - HS_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - HS_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - HS_FLAG_SINGLEMATCH - Only one match will be generated by the
 *                               expression per stream.
 *       - HS_FLAG_ALLOWEMPTY - Allow expressions which can match against an
 *                              empty string, such as `.*`.
 *       - HS_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - HS_FLAG_UCP - Use Unicode properties for character classes.
 *       - HS_FLAG_PREFILTER - Compile pattern in prefiltering mode.
 *       - HS_FLAG_SOM_LEFTMOST - Report the leftmost start of match offset
 *                                when a match is found.
 *
 * @param info
 *      On success, a pointer to the pattern information will be returned in
 *      this parameter, or NULL on failure. This structure is allocated using
 *      the allocator supplied in @ref hs_set_allocator() (or malloc() if no
 *      allocator was set) and should be freed by the caller.
 *
 * @param error
 *      If the call fails, a pointer to a @ref hs_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      hs_free_compile_error() function.
 *
 * @return
 *      @ref HS_SUCCESS is returned on successful compilation; @ref
 *      HS_COMPILER_ERROR on failure, with details provided in the error
 *      parameter.
 */
hs_error_t HS_CDECL hs_expression_info(const char *expression,
                                       unsigned int flags,
                                       hs_expr_info_t **info,
                                       hs_compile_error_t **error);

/**
 * Utility function providing information about a regular expression, with
 * extended parameter support. The information provided in @ref hs_expr_info_t
 * includes the minimum and maximum width of a pattern match.
 *
 * Note: successful analysis of an expression with this function does not imply
 * that compilation of the same expression (via @ref hs_compile(), @ref
 * hs_compile_multi() or @ref hs_compile_ext_multi()) would succeed. This
 * function may return @ref HS_SUCCESS for regular expressions that Hyperscan
 * cannot compile.
 *
 * Note: some per-pattern flags (such as @ref HS_FLAG_ALLOWEMPTY, @ref
 * HS_FLAG_SOM_LEFTMOST) are accepted by this call, but as they do not affect
 * the properties returned in the @ref hs_expr_info_t structure, they will not
 * affect the outcome of this function.
 *
 * @param expression
 *      The NULL-terminated expression to parse. Note that this string must
 *      represent ONLY the pattern to be matched, with no delimiters or flags;
 *      any global flags should be specified with the @p flags argument.  For
 *      example, the expression `/abc?def/i` should be compiled by providing
 *      `abc?def` as the @p expression, and @ref HS_FLAG_CASELESS as the @a
 *      flags.
 *
 * @param flags
 *      Flags which modify the behaviour of the expression. Multiple flags may
 *      be used by ORing them together. Valid values are:
 *       - HS_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - HS_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - HS_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - HS_FLAG_SINGLEMATCH - Only one match will be generated by the
 *                               expression per stream.
 *       - HS_FLAG_ALLOWEMPTY - Allow expressions which can match against an
 *                              empty string, such as `.*`.
 *       - HS_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - HS_FLAG_UCP - Use Unicode properties for character classes.
 *       - HS_FLAG_PREFILTER - Compile pattern in prefiltering mode.
 *       - HS_FLAG_SOM_LEFTMOST - Report the leftmost start of match offset
 *                                when a match is found.
 *
 * @param ext
 *      A pointer to a filled @ref hs_expr_ext_t structure that defines
 *      extended behaviour for this pattern. NULL may be specified if no
 *      extended parameters are needed.
 *
 * @param info
 *      On success, a pointer to the pattern information will be returned in
 *      this parameter, or NULL on failure. This structure is allocated using
 *      the allocator supplied in @ref hs_set_allocator() (or malloc() if no
 *      allocator was set) and should be freed by the caller.
 *
 * @param error
 *      If the call fails, a pointer to a @ref hs_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      hs_free_compile_error() function.
 *
 * @return
 *      @ref HS_SUCCESS is returned on successful compilation; @ref
 *      HS_COMPILER_ERROR on failure, with details provided in the error
 *      parameter.
 */
hs_error_t HS_CDECL hs_expression_ext_info(const char *expression,
                                           unsigned int flags,
                                           const hs_expr_ext_t *ext,
                                           hs_expr_info_t **info,
                                           hs_compile_error_t **error);

/**
 * Populates the platform information based on the current host.
 *
 * @param platform
 *      On success, the pointed to structure is populated based on the current
 *      host.
 *
 * @return
 *      @ref HS_SUCCESS on success, other values on failure.
 */
hs_error_t HS_CDECL hs_populate_platform(hs_platform_info_t *platform);

/**
 * @defgroup HS_PATTERN_FLAG Pattern flags
 *
 * @{
 */

/**
 * Compile flag: Set case-insensitive matching.
 *
 * This flag sets the expression to be matched case-insensitively by default.
 * The expression may still use PCRE tokens (notably `(?i)` and
 * `(?-i)`) to switch case-insensitive matching on and off.
 */
#define HS_FLAG_CASELESS        1

/**
 * Compile flag: Matching a `.` will not exclude newlines.
 *
 * This flag sets any instances of the `.` token to match newline characters as
 * well as all other characters. The PCRE specification states that the `.`
 * token does not match newline characters by default, so without this flag the
 * `.` token will not cross line boundaries.
 */
#define HS_FLAG_DOTALL          2

/**
 * Compile flag: Set multi-line anchoring.
 *
 * This flag instructs the expression to make the `^` and `$` tokens match
 * newline characters as well as the start and end of the stream. If this flag
 * is not specified, the `^` token will only ever match at the start of a
 * stream, and the `$` token will only ever match at the end of a stream within
 * the guidelines of the PCRE specification.
 */
#define HS_FLAG_MULTILINE       4

/**
 * Compile flag: Set single-match only mode.
 *
 * This flag sets the expression's match ID to match at most once. In streaming
 * mode, this means that the expression will return only a single match over
 * the lifetime of the stream, rather than reporting every match as per
 * standard Hyperscan semantics. In block mode or vectored mode, only the first
 * match for each invocation of @ref hs_scan() or @ref hs_scan_vector() will be
 * returned.
 *
 * If multiple expressions in the database share the same match ID, then they
 * either must all specify @ref HS_FLAG_SINGLEMATCH or none of them specify
 * @ref HS_FLAG_SINGLEMATCH. If a group of expressions sharing a match ID
 * specify the flag, then at most one match with the match ID will be generated
 * per stream.
 *
 * Note: The use of this flag in combination with @ref HS_FLAG_SOM_LEFTMOST
 * is not currently supported.
 */
#define HS_FLAG_SINGLEMATCH     8

/**
 * Compile flag: Allow expressions that can match against empty buffers.
 *
 * This flag instructs the compiler to allow expressions that can match against
 * empty buffers, such as `.?`, `.*`, `(a|)`. Since Hyperscan can return every
 * possible match for an expression, such expressions generally execute very
 * slowly; the default behaviour is to return an error when an attempt to
 * compile one is made. Using this flag will force the compiler to allow such
 * an expression.
 */
#define HS_FLAG_ALLOWEMPTY      16

/**
 * Compile flag: Enable UTF-8 mode for this expression.
 *
 * This flag instructs Hyperscan to treat the pattern as a sequence of UTF-8
 * characters. The results of scanning invalid UTF-8 sequences with a Hyperscan
 * library that has been compiled with one or more patterns using this flag are
 * undefined.
 */
#define HS_FLAG_UTF8            32

/**
 * Compile flag: Enable Unicode property support for this expression.
 *
 * This flag instructs Hyperscan to use Unicode properties, rather than the
 * default ASCII interpretations, for character mnemonics like `\w` and `\s` as
 * well as the POSIX character classes. It is only meaningful in conjunction
 * with @ref HS_FLAG_UTF8.
 */
#define HS_FLAG_UCP             64

/**
 * Compile flag: Enable prefiltering mode for this expression.
 *
 * This flag instructs Hyperscan to compile an "approximate" version of this
 * pattern for use in a prefiltering application, even if Hyperscan does not
 * support the pattern in normal operation.
 *
 * The set of matches returned when this flag is used is guaranteed to be a
 * superset of the matches specified by the non-prefiltering expression.
 *
 * If the pattern contains pattern constructs not supported by Hyperscan (such
 * as zero-width assertions, back-references or conditional references) these
 * constructs will be replaced internally with broader constructs that may
 * match more often.
 *
 * Furthermore, in prefiltering mode Hyperscan may simplify a pattern that
 * would otherwise return a "Pattern too large" error at compile time, or for
 * performance reasons (subject to the matching guarantee above).
 *
 * It is generally expected that the application will subsequently confirm
 * prefilter matches with another regular expression matcher that can provide
 * exact matches for the pattern.
 *
 * Note: The use of this flag in combination with @ref HS_FLAG_SOM_LEFTMOST
 * is not currently supported.
 */
#define HS_FLAG_PREFILTER       128

/**
 * Compile flag: Enable leftmost start of match reporting.
 *
 * This flag instructs Hyperscan to report the leftmost possible start of match
 * offset when a match is reported for this expression. (By default, no start
 * of match is returned.)
 *
 * Enabling this behaviour may reduce performance and increase stream state
 * requirements in streaming mode.
 */
#define HS_FLAG_SOM_LEFTMOST    256

/**
 * Compile flag: Logical combination.
 *
 * This flag instructs Hyperscan to parse this expression as logical
 * combination syntax.
 * Logical constraints consist of operands, operators and parentheses.
 * The operands are expression indices, and operators can be
 * '!'(NOT), '&'(AND) or '|'(OR).
 * For example:
 *     (101&102&103)|(104&!105)
 *     ((301|302)&303)&(304|305)
 */
#define HS_FLAG_COMBINATION     512

/**
 * Compile flag: Don't do any match reporting.
 *
 * This flag instructs Hyperscan to ignore match reporting for this expression.
 * It is designed to be used on the sub-expressions in logical combinations.
 */
#define HS_FLAG_QUIET           1024

/** @} */

/**
 * @defgroup HS_CPU_FEATURES_FLAG CPU feature support flags
 *
 * @{
 */

/**
 * CPU features flag - Intel(R) Advanced Vector Extensions 2 (Intel(R) AVX2)
 *
 * Setting this flag indicates that the target platform supports AVX2
 * instructions.
 */
#define HS_CPU_FEATURES_AVX2             (1ULL << 2)

/**
 * CPU features flag - Intel(R) Advanced Vector Extensions 512 (Intel(R) AVX512)
 *
 * Setting this flag indicates that the target platform supports AVX512
 * instructions, specifically AVX-512BW. Using AVX512 implies the use of AVX2.
 */
#define HS_CPU_FEATURES_AVX512           (1ULL << 3)

/** @} */

/**
 * @defgroup HS_TUNE_FLAG Tuning flags
 *
 * @{
 */

/**
 * Tuning Parameter - Generic
 *
 * This indicates that the compiled database should not be tuned for any
 * particular target platform.
 */
#define HS_TUNE_FAMILY_GENERIC 0

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Sandy Bridge
 *
 * This indicates that the compiled database should be tuned for the
 * Sandy Bridge microarchitecture.
 */
#define HS_TUNE_FAMILY_SNB 1

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Ivy Bridge
 *
 * This indicates that the compiled database should be tuned for the
 * Ivy Bridge microarchitecture.
 */
#define HS_TUNE_FAMILY_IVB 2

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Haswell
 *
 * This indicates that the compiled database should be tuned for the
 * Haswell microarchitecture.
 */
#define HS_TUNE_FAMILY_HSW 3

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Silvermont
 *
 * This indicates that the compiled database should be tuned for the
 * Silvermont microarchitecture.
 */
#define HS_TUNE_FAMILY_SLM 4

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Broadwell
 *
 * This indicates that the compiled database should be tuned for the
 * Broadwell microarchitecture.
 */
#define HS_TUNE_FAMILY_BDW 5

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Skylake
 *
 * This indicates that the compiled database should be tuned for the
 * Skylake microarchitecture.
 */
#define HS_TUNE_FAMILY_SKL 6

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Skylake Server
 *
 * This indicates that the compiled database should be tuned for the
 * Skylake Server microarchitecture.
 */
#define HS_TUNE_FAMILY_SKX 7

/**
 * Tuning Parameter - Intel(R) microarchitecture code name Goldmont
 *
 * This indicates that the compiled database should be tuned for the
 * Goldmont microarchitecture.
 */
#define HS_TUNE_FAMILY_GLM 8

/** @} */

/**
 * @defgroup HS_MODE_FLAG Compile mode flags
 *
 * The mode flags are used as values for the mode parameter of the various
 * compile calls (@ref hs_compile(), @ref hs_compile_multi() and @ref
 * hs_compile_ext_multi()).
 *
 * A mode value can be built by ORing these flag values together; the only
 * required flag is one of @ref HS_MODE_BLOCK, @ref HS_MODE_STREAM or @ref
 * HS_MODE_VECTORED. Other flags may be added to enable support for additional
 * features.
 *
 *  @{
 */

/**
 * Compiler mode flag: Block scan (non-streaming) database.
 */
#define HS_MODE_BLOCK           1

/**
 * Compiler mode flag: Alias for @ref HS_MODE_BLOCK.
 */
#define HS_MODE_NOSTREAM        1

/**
 * Compiler mode flag: Streaming database.
 */
#define HS_MODE_STREAM          2

/**
 * Compiler mode flag: Vectored scanning database.
 */
#define HS_MODE_VECTORED        4

/**
 * Compiler mode flag: use full precision to track start of match offsets in
 * stream state.
 *
 * This mode will use the most stream state per pattern, but will always return
 * an accurate start of match offset regardless of how far back in the past it
 * was found.
 *
 * One of the SOM_HORIZON modes must be selected to use the @ref
 * HS_FLAG_SOM_LEFTMOST expression flag.
 */
#define HS_MODE_SOM_HORIZON_LARGE   (1U << 24)

/**
 * Compiler mode flag: use medium precision to track start of match offsets in
 * stream state.
 *
 * This mode will use less stream state than @ref HS_MODE_SOM_HORIZON_LARGE and
 * will limit start of match accuracy to offsets within 2^32 bytes of the
 * end of match offset reported.
 *
 * One of the SOM_HORIZON modes must be selected to use the @ref
 * HS_FLAG_SOM_LEFTMOST expression flag.
 */
#define HS_MODE_SOM_HORIZON_MEDIUM  (1U << 25)

/**
 * Compiler mode flag: use limited precision to track start of match offsets in
 * stream state.
 *
 * This mode will use less stream state than @ref HS_MODE_SOM_HORIZON_LARGE and
 * will limit start of match accuracy to offsets within 2^16 bytes of the
 * end of match offset reported.
 *
 * One of the SOM_HORIZON modes must be selected to use the @ref
 * HS_FLAG_SOM_LEFTMOST expression flag.
 */
#define HS_MODE_SOM_HORIZON_SMALL   (1U << 26)

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HS_COMPILE_H_ */
