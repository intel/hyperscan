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
 * \brief Compiler front-end, including public API calls for compilation.
 */
#include "allocator.h"
#include "ue2common.h"
#include "grey.h"
#include "hs_compile.h"
#include "hs_internal.h"
#include "database.h"
#include "compiler/compiler.h"
#include "compiler/error.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_expr_info.h"
#include "parser/Parser.h"
#include "parser/parse_error.h"
#include "parser/prefilter.h"
#include "parser/unsupported.h"
#include "util/compile_error.h"
#include "util/cpuid_flags.h"
#include "util/cpuid_inline.h"
#include "util/depth.h"
#include "util/popcount.h"
#include "util/target_info.h"

#include <cassert>
#include <cstddef>
#include <cstring>
#include <limits.h>
#include <string>
#include <vector>

using namespace std;
using namespace ue2;

/** \brief Cheap check that no unexpected mode flags are on. */
static
bool validModeFlags(unsigned int mode) {
    static const unsigned allModeFlags = HS_MODE_BLOCK
                                       | HS_MODE_STREAM
                                       | HS_MODE_VECTORED
                                       | HS_MODE_SOM_HORIZON_LARGE
                                       | HS_MODE_SOM_HORIZON_MEDIUM
                                       | HS_MODE_SOM_HORIZON_SMALL;

    return !(mode & ~allModeFlags);
}

/** \brief Validate mode flags. */
static
bool checkMode(unsigned int mode, hs_compile_error **comp_error) {
    // First, check that only bits with meaning are on.
    if (!validModeFlags(mode)) {
        *comp_error = generateCompileError("Invalid parameter: "
                "unrecognised mode flags.", -1);
        return false;
    }

    // Our mode must be ONE of (block, streaming, vectored).
    unsigned checkmode
        = mode & (HS_MODE_STREAM | HS_MODE_BLOCK | HS_MODE_VECTORED);
    if (popcount32(checkmode) != 1) {
        *comp_error = generateCompileError(
            "Invalid parameter: mode must have one "
            "(and only one) of HS_MODE_BLOCK, HS_MODE_STREAM or "
            "HS_MODE_VECTORED set.",
            -1);
        return false;
    }

    // If you specify SOM precision, you must be in streaming mode and you only
    // get to have one.
    unsigned somMode = mode & (HS_MODE_SOM_HORIZON_LARGE |
                               HS_MODE_SOM_HORIZON_MEDIUM |
                               HS_MODE_SOM_HORIZON_SMALL);
    if (somMode) {
        if (!(mode & HS_MODE_STREAM)) {
            *comp_error = generateCompileError("Invalid parameter: the "
                    "HS_MODE_SOM_HORIZON_ mode flags may only be set in "
                    "streaming mode.", -1);
            return false;

        }
        if ((somMode & (somMode - 1)) != 0) {
            *comp_error = generateCompileError("Invalid parameter: only one "
                    "HS_MODE_SOM_HORIZON_ mode flag can be set.", -1);
            return false;
        }
    }

    return true;
}

static
bool checkPlatform(const hs_platform_info *p, hs_compile_error **comp_error) {
    static constexpr u32 HS_TUNE_LAST = HS_TUNE_FAMILY_ICX;
    static constexpr u32 HS_CPU_FEATURES_ALL =
        HS_CPU_FEATURES_AVX2 | HS_CPU_FEATURES_AVX512 |
        HS_CPU_FEATURES_AVX512VBMI;

    if (!p) {
        return true;
    }

    if (p->cpu_features & ~HS_CPU_FEATURES_ALL) {
        *comp_error = generateCompileError("Invalid cpu features specified in "
                                           "the platform information.", -1);
        return false;
   }

    if (p->tune > HS_TUNE_LAST) {
        *comp_error = generateCompileError("Invalid tuning value specified in "
                                           "the platform information.", -1);
        return false;
    }

    return true;
}

/** \brief Convert from SOM mode to bytes of precision. */
static
unsigned getSomPrecision(unsigned mode) {
    if (mode & HS_MODE_VECTORED) {
        /* always assume full precision for vectoring */
        return 8;
    }

    if (mode & HS_MODE_SOM_HORIZON_LARGE) {
        return 8;
    } else if (mode & HS_MODE_SOM_HORIZON_MEDIUM) {
        return 4;
    } else if (mode & HS_MODE_SOM_HORIZON_SMALL) {
        return 2;
    }
    return 0;
}

namespace ue2 {

hs_error_t
hs_compile_multi_int(const char *const *expressions, const unsigned *flags,
                     const unsigned *ids, const hs_expr_ext *const *ext,
                     unsigned elements, unsigned mode,
                     const hs_platform_info_t *platform, hs_database_t **db,
                     hs_compile_error_t **comp_error, const Grey &g) {
    // Check the args: note that it's OK for flags, ids or ext to be null.
    if (!comp_error) {
        if (db) {
            *db = nullptr;
        }
        // nowhere to write the string, but we can still report an error code
        return HS_COMPILER_ERROR;
    }
    if (!db) {
        *comp_error = generateCompileError("Invalid parameter: db is NULL", -1);
        return HS_COMPILER_ERROR;
    }
    if (!expressions) {
        *db = nullptr;
        *comp_error
            = generateCompileError("Invalid parameter: expressions is NULL",
                                   -1);
        return HS_COMPILER_ERROR;
    }
    if (elements == 0) {
        *db = nullptr;
        *comp_error = generateCompileError("Invalid parameter: elements is zero", -1);
        return HS_COMPILER_ERROR;
    }

#if defined(FAT_RUNTIME)
    if (!check_ssse3()) {
        *db = nullptr;
        *comp_error = generateCompileError("Unsupported architecture", -1);
        return HS_ARCH_ERROR;
    }
#endif

    if (!checkMode(mode, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkMode.
        return HS_COMPILER_ERROR;
    }

    if (!checkPlatform(platform, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkPlatform.
        return HS_COMPILER_ERROR;
    }

    if (elements > g.limitPatternCount) {
        *db = nullptr;
        *comp_error = generateCompileError("Number of patterns too large", -1);
        return HS_COMPILER_ERROR;
    }

    // This function is simply a wrapper around both the parser and compiler
    bool isStreaming = mode & (HS_MODE_STREAM | HS_MODE_VECTORED);
    bool isVectored = mode & HS_MODE_VECTORED;
    unsigned somPrecision = getSomPrecision(mode);

    target_t target_info = platform ? target_t(*platform)
                                    : get_current_target();

    try {
        CompileContext cc(isStreaming, isVectored, target_info, g);
        NG ng(cc, elements, somPrecision);

        for (unsigned int i = 0; i < elements; i++) {
            // Add this expression to the compiler
            try {
                addExpression(ng, i, expressions[i], flags ? flags[i] : 0,
                              ext ? ext[i] : nullptr, ids ? ids[i] : 0);
            } catch (CompileError &e) {
                /* Caught a parse error:
                 * throw it upstream as a CompileError with a specific index */
                e.setExpressionIndex(i);
                throw; /* do not slice */
            }
        }

        // Check sub-expression ids
        ng.rm.pl.validateSubIDs(ids, expressions, flags, elements);
        // Renumber and assign lkey to reports
        ng.rm.logicalKeyRenumber();

        unsigned length = 0;
        struct hs_database *out = build(ng, &length, 0);

        assert(out);    // should have thrown exception on error
        assert(length);

        *db = out;
        *comp_error = nullptr;

        return HS_SUCCESS;
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *db = nullptr;
        *comp_error = generateCompileError(e.reason,
                                           e.hasIndex ? (int)e.index : -1);
        return HS_COMPILER_ERROR;
    }
    catch (const std::bad_alloc &) {
        *db = nullptr;
        *comp_error = const_cast<hs_compile_error_t *>(&hs_enomem);
        return HS_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal error, unexpected exception");
        *db = nullptr;
        *comp_error = const_cast<hs_compile_error_t *>(&hs_einternal);
        return HS_COMPILER_ERROR;
    }
}

hs_error_t
hs_compile_lit_multi_int(const char *const *expressions, const unsigned *flags,
                         const unsigned *ids, const hs_expr_ext *const *ext,
                         const size_t *lens, unsigned elements, unsigned mode,
                         const hs_platform_info_t *platform, hs_database_t **db,
                         hs_compile_error_t **comp_error, const Grey &g) {
    // Check the args: note that it's OK for flags, ids or ext to be null.
    if (!comp_error) {
        if (db) {
            *db = nullptr;
        }
        // nowhere to write the string, but we can still report an error code
        return HS_COMPILER_ERROR;
    }
    if (!db) {
        *comp_error = generateCompileError("Invalid parameter: db is NULL", -1);
        return HS_COMPILER_ERROR;
    }
    if (!expressions) {
        *db = nullptr;
        *comp_error
            = generateCompileError("Invalid parameter: expressions is NULL",
                                   -1);
        return HS_COMPILER_ERROR;
    }
    if (!lens) {
        *db = nullptr;
        *comp_error = generateCompileError("Invalid parameter: len is NULL", -1);
        return HS_COMPILER_ERROR;
    }
    if (elements == 0) {
        *db = nullptr;
        *comp_error = generateCompileError("Invalid parameter: elements is zero", -1);
        return HS_COMPILER_ERROR;
    }

#if defined(FAT_RUNTIME)
    if (!check_ssse3()) {
        *db = nullptr;
        *comp_error = generateCompileError("Unsupported architecture", -1);
        return HS_ARCH_ERROR;
    }
#endif

    if (!checkMode(mode, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkMode.
        return HS_COMPILER_ERROR;
    }

    if (!checkPlatform(platform, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkPlattform.
        return HS_COMPILER_ERROR;
    }

    if (elements > g.limitPatternCount) {
        *db = nullptr;
        *comp_error = generateCompileError("Number of patterns too large", -1);
        return HS_COMPILER_ERROR;
    }

    // This function is simply a wrapper around both the parser and compiler
    bool isStreaming = mode & (HS_MODE_STREAM | HS_MODE_VECTORED);
    bool isVectored = mode & HS_MODE_VECTORED;
    unsigned somPrecision = getSomPrecision(mode);

    target_t target_info = platform ? target_t(*platform)
                                    : get_current_target();

    try {
        CompileContext cc(isStreaming, isVectored, target_info, g);
        NG ng(cc, elements, somPrecision);

        for (unsigned int i = 0; i < elements; i++) {
            // Add this expression to the compiler
            try {
                addLitExpression(ng, i, expressions[i], flags ? flags[i] : 0,
                                 ext ? ext[i] : nullptr, ids ? ids[i] : 0,
                                 lens[i]);
            } catch (CompileError &e) {
                /* Caught a parse error;
                 * throw it upstream as a CompileError with a specific index */
                e.setExpressionIndex(i);
                throw; /* do not slice */
            }
        }

        // Check sub-expression ids
        ng.rm.pl.validateSubIDs(ids, expressions, flags, elements);
        // Renumber and assign lkey to reports
        ng.rm.logicalKeyRenumber();

        unsigned length = 0;
        struct hs_database *out = build(ng, &length, 1);

        assert(out);    //should have thrown exception on error
        assert(length);

        *db = out;
        *comp_error = nullptr;

        return HS_SUCCESS;
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *db = nullptr;
        *comp_error = generateCompileError(e.reason,
                                           e.hasIndex ? (int)e.index : -1);
        return HS_COMPILER_ERROR;
    }
    catch (const std::bad_alloc &) {
        *db = nullptr;
        *comp_error = const_cast<hs_compile_error_t *>(&hs_enomem);
        return HS_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal errror, unexpected exception");
        *db = nullptr;
        *comp_error = const_cast<hs_compile_error_t *>(&hs_einternal);
        return HS_COMPILER_ERROR;
    }
}

} // namespace ue2

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_compile(const char *expression, unsigned flags,
                               unsigned mode,
                               const hs_platform_info_t *platform,
                               hs_database_t **db, hs_compile_error_t **error) {
    if (expression == nullptr) {
        *db = nullptr;
        *error = generateCompileError("Invalid parameter: expression is NULL",
                                      -1);
        return HS_COMPILER_ERROR;
    }

    unsigned id = 0; // single expressions get zero as an ID
    const hs_expr_ext * const *ext = nullptr; // unused for this call.

    return hs_compile_multi_int(&expression, &flags, &id, ext, 1, mode,
                                platform, db, error, Grey());
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_compile_multi(const char *const *expressions,
                                     const unsigned *flags, const unsigned *ids,
                                     unsigned elements, unsigned mode,
                                     const hs_platform_info_t *platform,
                                     hs_database_t **db,
                                     hs_compile_error_t **error) {
    const hs_expr_ext * const *ext = nullptr; // unused for this call.
    return hs_compile_multi_int(expressions, flags, ids, ext, elements, mode,
                                platform, db, error, Grey());
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_compile_ext_multi(const char * const *expressions,
                                     const unsigned *flags, const unsigned *ids,
                                     const hs_expr_ext * const *ext,
                                     unsigned elements, unsigned mode,
                                     const hs_platform_info_t *platform,
                                     hs_database_t **db,
                                     hs_compile_error_t **error) {
    return hs_compile_multi_int(expressions, flags, ids, ext, elements, mode,
                                platform, db, error, Grey());
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_compile_lit(const char *expression, unsigned flags,
                                   const size_t len, unsigned mode,
                                   const hs_platform_info_t *platform,
                                   hs_database_t **db,
                                   hs_compile_error_t **error) {
    if (expression == nullptr) {
        *db = nullptr;
        *error = generateCompileError("Invalid parameter: expression is NULL",
                                      -1);
        return HS_COMPILER_ERROR;
    }

    unsigned id = 0; // single expressions get zero as an ID
    const hs_expr_ext * const *ext = nullptr; // unused for this call.

    return hs_compile_lit_multi_int(&expression, &flags, &id, ext, &len, 1,
                                    mode, platform, db, error, Grey());
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_compile_lit_multi(const char * const *expressions,
                                         const unsigned *flags,
                                         const unsigned *ids,
                                         const size_t *lens,
                                         unsigned elements, unsigned mode,
                                         const hs_platform_info_t *platform,
                                         hs_database_t **db,
                                         hs_compile_error_t **error) {
    const hs_expr_ext * const *ext = nullptr; // unused for this call.
    return hs_compile_lit_multi_int(expressions, flags, ids, ext, lens,
                                    elements, mode, platform, db, error,
                                    Grey());
}

static
hs_error_t hs_expression_info_int(const char *expression, unsigned int flags,
                                  const hs_expr_ext_t *ext, unsigned int mode,
                                  hs_expr_info_t **info,
                                  hs_compile_error_t **error) {
    if (!error) {
        // nowhere to write an error, but we can still return an error code.
        return HS_COMPILER_ERROR;
    }

#if defined(FAT_RUNTIME)
    if (!check_ssse3()) {
        *error = generateCompileError("Unsupported architecture", -1);
        return HS_ARCH_ERROR;
    }
#endif

    if (!info) {
        *error = generateCompileError("Invalid parameter: info is NULL", -1);
        return HS_COMPILER_ERROR;
    }

    if (!expression) {
        *error = generateCompileError("Invalid parameter: expression is NULL",
                                      -1);
        return HS_COMPILER_ERROR;
    }

    *info = nullptr;
    *error = nullptr;

    hs_expr_info local_info;
    memset(&local_info, 0, sizeof(local_info));

    try {
        bool isStreaming = mode & (HS_MODE_STREAM | HS_MODE_VECTORED);
        bool isVectored = mode & HS_MODE_VECTORED;

        CompileContext cc(isStreaming, isVectored, get_current_target(),
                          Grey());

        // Ensure that our pattern isn't too long (in characters).
        if (strlen(expression) > cc.grey.limitPatternLength) {
            throw ParseError("Pattern length exceeds limit.");
        }

        ReportManager rm(cc.grey);
        ParsedExpression pe(0, expression, flags, 0, ext);
        assert(pe.component);

        // Apply prefiltering transformations if desired.
        if (pe.expr.prefilter) {
            prefilterTree(pe.component, ParseMode(flags));
        }

        // Expressions containing zero-width assertions and other extended pcre
        // types aren't supported yet. This call will throw a ParseError
        // exception if the component tree contains such a construct.
        checkUnsupported(*pe.component);

        pe.component->checkEmbeddedStartAnchor(true);
        pe.component->checkEmbeddedEndAnchor(true);

        auto built_expr = buildGraph(rm, cc, pe);
        unique_ptr<NGHolder> &g = built_expr.g;
        ExpressionInfo &expr = built_expr.expr;

        if (!g) {
            DEBUG_PRINTF("NFA build failed, but no exception was thrown.\n");
            throw ParseError("Internal error.");
        }

        fillExpressionInfo(rm, cc, *g, expr, &local_info);
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *error = generateCompileError(e);
        return HS_COMPILER_ERROR;
    }
    catch (std::bad_alloc &) {
        *error = const_cast<hs_compile_error_t *>(&hs_enomem);
        return HS_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal error, unexpected exception");
        *error = const_cast<hs_compile_error_t *>(&hs_einternal);
        return HS_COMPILER_ERROR;
    }

    hs_expr_info *rv = (hs_expr_info *)hs_misc_alloc(sizeof(*rv));
    if (!rv) {
        *error = const_cast<hs_compile_error_t *>(&hs_enomem);
        return HS_COMPILER_ERROR;
    }

    *rv = local_info;
    *info = rv;
    return HS_SUCCESS;
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_expression_info(const char *expression,
                                       unsigned int flags,
                                       hs_expr_info_t **info,
                                       hs_compile_error_t **error) {
    return hs_expression_info_int(expression, flags, nullptr, HS_MODE_BLOCK,
                                  info, error);
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_expression_ext_info(const char *expression,
                                           unsigned int flags,
                                           const hs_expr_ext_t *ext,
                                           hs_expr_info_t **info,
                                           hs_compile_error_t **error) {
    return hs_expression_info_int(expression, flags, ext, HS_MODE_BLOCK, info,
                                  error);
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_populate_platform(hs_platform_info_t *platform) {
    if (!platform) {
        return HS_INVALID;
    }

    memset(platform, 0, sizeof(*platform));

    platform->cpu_features = cpuid_flags();
    platform->tune = cpuid_tune();

    return HS_SUCCESS;
}

extern "C" HS_PUBLIC_API
hs_error_t HS_CDECL hs_free_compile_error(hs_compile_error_t *error) {
#if defined(FAT_RUNTIME)
    if (!check_ssse3()) {
        return HS_ARCH_ERROR;
    }
#endif
    freeCompileError(error);
    return HS_SUCCESS;
}
