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

#ifndef CH_COMPILE_H_
#define CH_COMPILE_H_

/**
 * @file
 * @brief The Chimera compiler API definition.
 *
 * Chimera is a hybrid solution of Hyperscan and PCRE.
 *
 * This header contains functions for compiling regular expressions into
 * Chimera databases that can be used by the Chimera runtime.
 */

#include "ch_common.h"
#include "hs_compile.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * A type containing error details that is returned by the compile calls (@ref
 * ch_compile() and @ref ch_compile_multi() on failure. The caller may inspect
 * the values returned in this type to determine the cause of failure.
 */
typedef struct ch_compile_error {
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
} ch_compile_error_t;

/**
 * The basic regular expression compiler.
 *
 * This is the function call with which an expression is compiled into a
 * Chimera database which can be passed to the runtime function (
 * @ref ch_scan())
 *
 * @param expression
 *      The NULL-terminated expression to parse. Note that this string must
 *      represent ONLY the pattern to be matched, with no delimiters or flags;
 *      any global flags should be specified with the @a flags argument. For
 *      example, the expression `/abc?def/i` should be compiled by providing
 *      `abc?def` as the @a expression, and @ref CH_FLAG_CASELESS as the @a
 *      flags.
 *
 * @param flags
 *      Flags which modify the behaviour of the expression. Multiple flags may
 *      be used by ORing them together. Valid values are:
 *       - CH_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - CH_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - CH_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - CH_FLAG_SINGLEMATCH - Only one match will be generated for the
 *                               expression per stream.
 *       - CH_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - CH_FLAG_UCP - Use Unicode properties for character classes.
 *
 * @param mode
 *      Compiler mode flag that affect the database as a whole for capturing
 *      groups. One of  CH_MODE_NOGROUPS or  CH_MODE_GROUPS must be supplied.
 *      See @ref CH_MODE_FLAG for more details.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref ch_free_database() function.
 *
 * @param compile_error
 *      If the compile fails, a pointer to a @ref ch_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      ch_free_compile_error() function.
 *
 * @return
 *      @ref CH_SUCCESS is returned on successful compilation; @ref
 *      CH_COMPILER_ERROR on failure, with details provided in the error
 *      parameter.
 */
ch_error_t HS_CDECL ch_compile(const char *expression, unsigned int flags,
                               unsigned int mode,
                               const hs_platform_info_t *platform,
                               ch_database_t **db,
                               ch_compile_error_t **compile_error);

/**
 * The multiple regular expression compiler.
 *
 * This is the function call with which a set of expressions is compiled into a
 * database which can be passed to the runtime function (@ref ch_scan()).
 * Each expression can be labelled with a unique integer which is passed into
 * the match callback to identify the pattern that has matched.
 *
 * @param expressions
 *      Array of NULL-terminated expressions to compile. Note that (as for @ref
 *      ch_compile()) these strings must contain only the pattern to be
 *      matched, with no delimiters or flags. For example, the expression
 *      `/abc?def/i` should be compiled by providing `abc?def` as the first
 *      string in the @a expressions array, and @ref CH_FLAG_CASELESS as the
 *      first value in the @a flags array.
 *
 * @param flags
 *      Array of flags which modify the behaviour of each expression. Multiple
 *      flags may be used by ORing them together.  Specifying the NULL pointer
 *      in place of an array will set the flags value for all patterns to zero.
 *      Valid values are:
 *       - CH_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - CH_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - CH_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - CH_FLAG_SINGLEMATCH - Only one match will be generated by patterns
 *                               with this match id per stream.
 *       - CH_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - CH_FLAG_UCP - Use Unicode properties for character classes.
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
 *      Compiler mode flag that affect the database as a whole for capturing
 *      groups. One of  CH_MODE_NOGROUPS or  CH_MODE_GROUPS must be supplied.
 *      See @ref CH_MODE_FLAG for more details.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref ch_free_database() function.
 *
 * @param compile_error
 *      If the compile fails, a pointer to a @ref ch_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      ch_free_compile_error() function.
 *
 * @return
 *      @ref CH_SUCCESS is returned on successful compilation; @ref
 *      CH_COMPILER_ERROR on failure, with details provided in the @a error
 *      parameter.
 *
 */
ch_error_t HS_CDECL ch_compile_multi(const char *const *expressions,
                                     const unsigned int *flags,
                                     const unsigned int *ids,
                                     unsigned int elements, unsigned int mode,
                                     const hs_platform_info_t *platform,
                                     ch_database_t **db,
                                     ch_compile_error_t **compile_error);

/**
 * The multiple regular expression compiler with extended match limits support.
 *
 * This is the function call with which a set of expressions is compiled into a
 * database in the same way as @ref ch_compile_multi(), but allows additional
 * parameters to be specified via match_limit and match_limit_recursion to
 * define match limits for PCRE runtime.
 *
 * @param expressions
 *      Array of NULL-terminated expressions to compile. Note that (as for @ref
 *      ch_compile()) these strings must contain only the pattern to be
 *      matched, with no delimiters or flags. For example, the expression
 *      `/abc?def/i` should be compiled by providing `abc?def` as the first
 *      string in the @a expressions array, and @ref CH_FLAG_CASELESS as the
 *      first value in the @a flags array.
 *
 * @param flags
 *      Array of flags which modify the behaviour of each expression. Multiple
 *      flags may be used by ORing them together.  Specifying the NULL pointer
 *      in place of an array will set the flags value for all patterns to zero.
 *      Valid values are:
 *       - CH_FLAG_CASELESS - Matching will be performed case-insensitively.
 *       - CH_FLAG_DOTALL - Matching a `.` will not exclude newlines.
 *       - CH_FLAG_MULTILINE - `^` and `$` anchors match any newlines in data.
 *       - CH_FLAG_SINGLEMATCH - Only one match will be generated by patterns
 *                               with this match id per stream.
 *       - CH_FLAG_UTF8 - Treat this pattern as a sequence of UTF-8 characters.
 *       - CH_FLAG_UCP - Use Unicode properties for character classes.
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
 *      Compiler mode flag that affect the database as a whole for capturing
 *      groups. One of  CH_MODE_NOGROUPS or  CH_MODE_GROUPS must be supplied.
 *      See @ref CH_MODE_FLAG for more details.
 *
 * @param match_limit
 *      A limit from pcre_extra on the amount of match function called in PCRE
 *      to limit backtracking that can take place.
 *
 * @param match_limit_recursion
 *      A limit from pcre_extra on the recursion depth of match function
 *      in PCRE.
 *
 * @param platform
 *      If not NULL, the platform structure is used to determine the target
 *      platform for the database. If NULL, a database suitable for running
 *      on the current host platform is produced.
 *
 * @param db
 *      On success, a pointer to the generated database will be returned in
 *      this parameter, or NULL on failure. The caller is responsible for
 *      deallocating the buffer using the @ref ch_free_database() function.
 *
 * @param compile_error
 *      If the compile fails, a pointer to a @ref ch_compile_error_t will be
 *      returned, providing details of the error condition. The caller is
 *      responsible for deallocating the buffer using the @ref
 *      ch_free_compile_error() function.
 *
 * @return
 *      @ref CH_SUCCESS is returned on successful compilation; @ref
 *      CH_COMPILER_ERROR on failure, with details provided in the @a error
 *      parameter.
 *
 */
ch_error_t HS_CDECL ch_compile_ext_multi(const char *const *expressions,
                                         const unsigned int *flags,
                                         const unsigned int *ids,
                                         unsigned int elements,
                                         unsigned int mode,
                                         unsigned long int match_limit,
                                         unsigned long int match_limit_recursion,
                                         const hs_platform_info_t *platform,
                                         ch_database_t **db,
                                         ch_compile_error_t **compile_error);

/**
 * Free an error structure generated by @ref ch_compile(), @ref
 * ch_compile_multi().
 *
 * @param error
 *      The @ref ch_compile_error_t to be freed. NULL may also be safely
 *      provided.
 *
 * @return
 *      @ref CH_SUCCESS on success, other values on failure.
 */
ch_error_t HS_CDECL ch_free_compile_error(ch_compile_error_t *error);

/**
 * @defgroup CH_PATTERN_FLAG Pattern flags
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
#define CH_FLAG_CASELESS        1

/**
 * Compile flag: Matching a `.` will not exclude newlines.
 *
 * This flag sets any instances of the `.` token to match newline characters as
 * well as all other characters. The PCRE specification states that the `.`
 * token does not match newline characters by default, so without this flag the
 * `.` token will not cross line boundaries.
 */
#define CH_FLAG_DOTALL          2

/**
 * Compile flag: Set multi-line anchoring.
 *
 * This flag instructs the expression to make the `^` and `$` tokens match
 * newline characters as well as the start and end of the stream. If this flag
 * is not specified, the `^` token will only ever match at the start of a
 * stream, and the `$` token will only ever match at the end of a stream within
 * the guidelines of the PCRE specification.
 */
#define CH_FLAG_MULTILINE       4

/**
 * Compile flag: Set single-match only mode.
 *
 * This flag sets the expression's match ID to match at most once, only the
 * first match for each invocation of @ref ch_scan() will be returned.
 *
 */
#define CH_FLAG_SINGLEMATCH     8

/**
 * Compile flag: Enable UTF-8 mode for this expression.
 *
 * This flag instructs Chimera to treat the pattern as a sequence of UTF-8
 * characters. The results of scanning invalid UTF-8 sequences with a Chimera
 * library that has been compiled with one or more patterns using this flag are
 * undefined.
 */
#define CH_FLAG_UTF8            32

/**
 * Compile flag: Enable Unicode property support for this expression.
 *
 * This flag instructs Chimera to use Unicode properties, rather than the
 * default ASCII interpretations, for character mnemonics like `\w` and `\s` as
 * well as the POSIX character classes. It is only meaningful in conjunction
 * with @ref CH_FLAG_UTF8.
 */
#define CH_FLAG_UCP             64

/** @} */

/**
 * @defgroup CH_MODE_FLAG Compile mode flags
 *
 * The mode flags are used as values for the mode parameter of the various
 * compile calls (@ref ch_compile(), @ref ch_compile_multi().
 *
 * By default, the matcher will only supply the start and end offsets of the
 * match when the match callback is called. Using mode flag @ref CH_MODE_GROUPS
 * will also fill the `captured' array with the start and end offsets of all
 * the capturing groups specified by the pattern that has matched.
 *
 * @{
 */

/**
 * Compiler mode flag: Disable capturing groups.
 */
#define CH_MODE_NOGROUPS        0

/**
 * Compiler mode flag: Enable capturing groups.
 */
#define CH_MODE_GROUPS          1048576

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CH_COMPILE_H_ */
