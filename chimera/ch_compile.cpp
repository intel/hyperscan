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

/** \file
 * \brief Compiler front-end, including public API calls for compilation.
 */

#include "ch_compile.h"
#include "ch_alloc.h"
#include "ch_internal.h"
#include "ch_database.h"
#include "grey.h"
#include "hs_common.h"
#include "hs_internal.h"
#include "ue2common.h"
#include "util/compile_error.h"
#include "util/make_unique.h"
#include "util/multibit_build.h"
#include "util/target_info.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <memory>
#include <ostream>
#include <sstream>
#include <limits.h>
#include <string>
#include <vector>

#include <boost/core/noncopyable.hpp>

#define PCRE_ERROR_MSG "Internal error building PCRE pattern."

using namespace std;
using namespace ue2;

static const char failureNoMemory[] = "Unable to allocate memory.";
static const char failureInternal[] = "Internal error.";
static const char failureBadAlloc[] = "Allocator returned misaligned memory.";

static const ch_compile_error_t ch_enomem
    = { const_cast<char *>(failureNoMemory), 0 };
static const ch_compile_error_t ch_einternal
    = { const_cast<char *>(failureInternal), 0 };
static const ch_compile_error_t ch_badalloc
    = { const_cast<char *>(failureBadAlloc), 0 };

static
ch_compile_error_t *generateChimeraCompileError(const string &err,
                                                int expression) {
    ch_compile_error_t *ret =
        (struct ch_compile_error *)ch_misc_alloc(sizeof(ch_compile_error_t));
    if (ret) {
        ch_error_t e = ch_check_alloc(ret);
        if (e != CH_SUCCESS) {
            ch_misc_free(ret);
            return const_cast<ch_compile_error_t *>(&ch_badalloc);
        }
        char *msg = (char *)ch_misc_alloc(err.size() + 1);
        if (msg) {
            e = ch_check_alloc(msg);
            if (e != HS_SUCCESS) {
                ch_misc_free(msg);
                return const_cast<ch_compile_error_t *>(&ch_badalloc);
            }
            memcpy(msg, err.c_str(), err.size() + 1);
            ret->message = msg;
        } else {
            ch_misc_free(ret);
            ret = nullptr;
        }
    }

    if (!ret || !ret->message) {
        return const_cast<ch_compile_error_t *>(&ch_enomem);
    }

    ret->expression = expression;

    return ret;
}

static
void freeChimeraCompileError(ch_compile_error_t *error) {
    if (!error) {
        return;
    }
    if (error == &ch_enomem || error == &ch_einternal ||
        error == &ch_badalloc) {
        // These are not allocated.
        return;
    }

    ch_misc_free(error->message);
    ch_misc_free(error);
}

static
bool checkMode(unsigned int mode, ch_compile_error_t **comp_error) {
    static const unsigned int supported = CH_MODE_GROUPS;

    if (mode & ~supported) {
        *comp_error =
            generateChimeraCompileError("Invalid mode flag supplied.", -1);
        return false;
    }
    return true;
}

/** \brief Throw a compile error if we're passed some unsupported flags. */
static
void checkFlags(const unsigned int flags) {
    static const unsigned int supported = HS_FLAG_DOTALL
                                        | HS_FLAG_MULTILINE
                                        | HS_FLAG_CASELESS
                                        | HS_FLAG_SINGLEMATCH
                                        | HS_FLAG_UCP
                                        | HS_FLAG_UTF8;

    if (flags & ~supported) {
        throw CompileError("Unrecognized flag used.");
    }
}

static
bool isHyperscanSupported(const char *expression, unsigned int flags,
                          const hs_platform_info *platform) {
    hs_database_t *db = nullptr;
    hs_compile_error *comp_error = nullptr;

    unsigned int id = 0;
    hs_error_t err = hs_compile_multi(&expression, &flags, &id,
                                      1, HS_MODE_BLOCK, platform, &db,
                                      &comp_error);
    if (err != HS_SUCCESS) {
        assert(!db);
        assert(comp_error);
        DEBUG_PRINTF("unsupported: %s\n", comp_error->message);
        hs_free_compile_error(comp_error);

        return false;
    }

    assert(db);
    assert(!comp_error);
    hs_free_database(db);
    return true;
}

static
bool writeHyperscanDatabase(char *ptr, hs_database_t *db) {
    // Note: we must use our serialization calls to re-home the database.
    char *serialized = nullptr;
    size_t slen = 0;
    hs_error_t err = hs_serialize_database(db, &serialized, &slen);
    if (err != HS_SUCCESS) {
        DEBUG_PRINTF("hs_serialize_database returned %d\n", err);
        assert(0);
        return false;
    }

    DEBUG_PRINTF("writing database to ptr %p\n", ptr);

    // deserialize_at without the platform tests.
    err = hs_deserialize_database_at(serialized, slen, (hs_database_t *)ptr);
    if (err != HS_SUCCESS) {
        DEBUG_PRINTF("hs_deserialize_database_at returned %d\n", err);
        assert(0);
        ch_misc_free(serialized);
        return false;
    }

    ch_misc_free(serialized);
    return true;
}

static
bool writeHyperscanDatabase(ch_bytecode *db, hs_database_t *hs_db) {
    db->databaseOffset = ROUNDUP_CL(sizeof(*db));
    char *ptr = (char *)db + db->databaseOffset;
    return writeHyperscanDatabase(ptr, hs_db);
}

static
int convertFlagsToPcreOptions(unsigned int flags) {
    int options = 0;
    if (flags & HS_FLAG_CASELESS) {
        options |= PCRE_CASELESS;
    }
    if (flags & HS_FLAG_DOTALL) {
        options |= PCRE_DOTALL;
    }
    if (flags & HS_FLAG_MULTILINE) {
        options |= PCRE_MULTILINE;
    }
    if (flags & HS_FLAG_UTF8) {
        options |= PCRE_UTF8;
    }
    if (flags & HS_FLAG_UCP) {
        options |= PCRE_UCP;
    }

    // All other flags are meaningless to PCRE.

    return options;
}

namespace {

/** \brief Data about a single pattern. */
struct PatternData : boost::noncopyable {
    PatternData(const char *pattern, u32 flags, u32 idx, u32 id_in,
                unsigned mode, unsigned long int match_limit,
                unsigned long int match_limit_recursion,
                const hs_platform_info *platform);
    ~PatternData() {
        pcre_free(compiled);
        pcre_free(extra);
    }

    void buildPcre(const char *pattern, u32 flags);

    size_t patternSize() const;

    void writePattern(ch_pattern *pattern) const;

    pcre *compiled;                     //!< pcre_compile output
    pcre_extra *extra;                  //!< pcre_study output
    size_t compiled_size;
    int study_size;
    int capture_cnt;
    bool utf8;
    u32 id;                             //!< ID from the user
    u32 expr_index;                     //!< index in the expression array
    bool singlematch;                   //!< pattern is in highlander mode
    bool guard;       //!< this pattern should be guarded by the multimatcher
    u32 minWidth;     //!< min match width
    u32 maxWidth;     //!< max match width
    u32 fixedWidth;   //!< fixed pattern width
    unsigned long int matchLimit; //! pcre match limit
    unsigned long int matchLimitRecursion; //! pcre match_limit_recursion
};

PatternData::PatternData(const char *pattern, u32 flags, u32 idx, u32 id_in,
                         unsigned mode, unsigned long int match_limit,
                         unsigned long int match_limit_recursion,
                         const hs_platform_info *platform)
    : compiled(nullptr), extra(nullptr), id(id_in), expr_index(idx),
      singlematch(flags & HS_FLAG_SINGLEMATCH),
      guard(false), minWidth(0), maxWidth(UINT_MAX),
      fixedWidth(UINT_MAX), matchLimit(match_limit),
      matchLimitRecursion(match_limit_recursion) {
    assert(pattern);

    flags |= HS_FLAG_ALLOWEMPTY; /* don't hand things off to pcre for no
                                    reason */

    buildPcre(pattern, flags);

    // Fetch the expression info for a prefiltering, non-singlematch version of
    // this pattern, if possible.
    hs_expr_info *info = nullptr;
    hs_compile_error_t *error = nullptr;
    u32 infoflags = (flags | HS_FLAG_PREFILTER) & ~HS_FLAG_SINGLEMATCH;
    u32 rawflags = (flags | HS_FLAG_SOM_LEFTMOST) & ~HS_FLAG_SINGLEMATCH;
    hs_error_t err = hs_expression_info(pattern, infoflags, &info, &error);
    if (err == HS_SUCCESS) {
        assert(info);
        hs_expr_info *i = (hs_expr_info *)info;
        minWidth = i->min_width;
        maxWidth = i->max_width;
        bool ordered = i->unordered_matches ? false : true;

        // Only enable capturing if required
        u32 captureCnt = 0;
        if (mode & CH_MODE_GROUPS) {
            captureCnt = capture_cnt;
        }

        // No need to confirm with PCRE if:
        // 1) pattern is fixed width
        // 2) pattern isn't vacuous as it can't combine with start of match
        // 3) no capturing in this pattern
        // 4) no offset adjust in this pattern as hyperscan match callback
        //    will arrive without order, i.e. [^a]\z has offset adjust
        // 5) hyperscan compile succeeds without prefiltering
        if (minWidth == maxWidth && minWidth && maxWidth != UINT_MAX &&
            !captureCnt && ordered &&
            isHyperscanSupported(pattern, rawflags, platform)) {
            fixedWidth = maxWidth;
        }

        DEBUG_PRINTF("gathered info: widths=[%u,%u]\n", minWidth, maxWidth);

        ch_misc_free(info);

        u32 guardflags;
        guardflags = (flags | HS_FLAG_PREFILTER) & ~HS_FLAG_SINGLEMATCH;
        guard = isHyperscanSupported(pattern, guardflags, platform);
    } else {
        // We can't even prefilter this pattern, so we're dependent on Big Dumb
        // Pcre Scans.
        DEBUG_PRINTF("hs_expression_info failed, falling back to pcre\n");
        hs_free_compile_error(error);
    }
}

void PatternData::buildPcre(const char *pattern, u32 flags) {
    int options = convertFlagsToPcreOptions(flags);
    const char *errptr = nullptr;
    int erroffset = 0;

    compiled = pcre_compile(pattern, options, &errptr, &erroffset, nullptr);
    if (!compiled) {
        DEBUG_PRINTF("PCRE failed to compile: %s\n", pattern);
        string err("PCRE compilation failed: ");
        err += string(errptr);
        err += ".";
        throw CompileError(expr_index, err);
    }

    extra = pcre_study(compiled, PCRE_STUDY_JIT_COMPILE, &errptr);
    // Note that it's OK for pcre_study to return NULL if there's nothing
    // to be found, but a non-NULL error is always bad.
    if (errptr) {
        DEBUG_PRINTF("PCRE could not be studied: %s\n", errptr);
        string err("PCRE compilation failed: ");
        err += string(errptr);
        err += ".";
        throw CompileError(expr_index, err);
    }

    if (pcre_fullinfo(compiled, extra, PCRE_INFO_SIZE, &compiled_size)) {
        throw CompileError(PCRE_ERROR_MSG);
    }

    if (!extra) {
        study_size = 0;
    } else {
        if (pcre_fullinfo(compiled, extra, PCRE_INFO_STUDYSIZE, &study_size)) {
            throw CompileError(PCRE_ERROR_MSG);
        }
    }

    if (pcre_fullinfo(compiled, extra, PCRE_INFO_CAPTURECOUNT, &capture_cnt)) {
        throw CompileError(PCRE_ERROR_MSG);
    }

    /* We use the pcre rather than hs to get this information as we may need it
     * even in the pure unguarded pcre mode where there is no hs available. We
     * can not use the compile flags due to (*UTF8) verb */
    unsigned long int opts = 0; // PCRE_INFO_OPTIONS demands an unsigned long
    if (pcre_fullinfo(compiled, extra, PCRE_INFO_OPTIONS, &opts)) {
        throw CompileError(PCRE_ERROR_MSG);
    }
    utf8 = opts & PCRE_UTF8;
}

size_t PatternData::patternSize() const {
    size_t len = 0;

    // ch_pattern header.
    len += sizeof(ch_pattern);

    len = ROUNDUP_N(len, 8);
    DEBUG_PRINTF("compiled pcre at %zu\n", len);
    len += compiled_size;

    // PCRE study data, which may be zero.
    if (study_size) {
        len = ROUNDUP_N(len, 8);
        DEBUG_PRINTF("study at %zu\n", len);
        len += (size_t)study_size;
    }

    DEBUG_PRINTF("pattern size %zu\n", len);
    return len;
}

/** \brief Write out an ch_pattern structure, which should already be sized
 * correctly according to PatternData::patternSize. */
void PatternData::writePattern(ch_pattern *pattern) const {
    assert(pattern);
    assert(ISALIGNED_CL(pattern));

    pattern->id = id;

    u32 flags = 0;
    if (singlematch) {
        flags |= CHIMERA_PATTERN_FLAG_SINGLEMATCH;
    }
    if (utf8) {
        flags |= CHIMERA_PATTERN_FLAG_UTF8;
    }

    pattern->flags = flags;
    pattern->maxWidth = maxWidth;
    pattern->minWidth = minWidth == UINT_MAX ? 0 : minWidth;
    pattern->fixedWidth = fixedWidth;

    // Compiled PCRE pattern.
    char *ptr = (char *)pattern;
    ptr += ROUNDUP_N(sizeof(*pattern), 8);
    DEBUG_PRINTF("compiled pcre at %zu\n", (size_t)(ptr - (char *)pattern));
    memcpy(ptr, compiled, compiled_size);
    ptr += compiled_size;

    // PCRE match limits
    pattern->extra.flags = PCRE_EXTRA_MATCH_LIMIT |
                           PCRE_EXTRA_MATCH_LIMIT_RECURSION;
    pattern->extra.match_limit = matchLimit ? matchLimit : 10000000;
    // Set to avoid segment fault
    pattern->extra.match_limit_recursion =
        matchLimitRecursion ? matchLimitRecursion : 1500;

    // PCRE study_data.
    u32 studyOffset = 0;
    if (extra) {
        assert(extra->study_data);
        ptr = ROUNDUP_PTR(ptr, 8);
        DEBUG_PRINTF("study at %zu\n", (size_t)(ptr - (char *)pattern));
        memcpy(ptr, extra->study_data, study_size);
        studyOffset = (size_t)(ptr - (char *)pattern);

        pattern->extra.flags |= PCRE_EXTRA_STUDY_DATA;
        pattern->extra.study_data = ptr;

        ptr += study_size;
    } else {
        pattern->extra.flags &= ~PCRE_EXTRA_STUDY_DATA;
    }
    pattern->studyOffset = studyOffset;

    size_t pcreLen = (ptr - (char *)pattern);
    assert(pcreLen <= patternSize());
    pattern->length = (u32)pcreLen;

    // We shouldn't overrun the space we've allocated for this pattern.
    assert(patternSize() >= (size_t)(ptr - (char *)pattern));
}

} // namespace

namespace ch {

static
void ch_compile_multi_int(const char *const *expressions, const unsigned *flags,
                          const unsigned *ids, unsigned elements,
                          unsigned mode, unsigned long int match_limit,
                          unsigned long int match_limit_recursion,
                          const hs_platform_info_t *platform,
                          ch_database_t **out) {
    vector<unique_ptr<PatternData>> pcres;
    pcres.reserve(elements);
    vector<u32> unguarded; // indices of unguarded PCREs.
    vector<const char *> multiExpr;
    vector<unsigned int> multiFlags;
    vector<unsigned int> multiIds;
    bool allConfirm = true;
    bool allSingleMatch = true;
    for (unsigned int i = 0; i < elements; i++) {
        const char *myExpr = expressions[i];
        unsigned int myFlags = flags ? flags[i] : 0;
        unsigned int myId = ids ? ids[i] : 0;

        checkFlags(myFlags);

        // First, build with libpcre. A build failure from libpcre will throw
        // an exception up to the caller.
        auto patternData =
            ue2::make_unique<PatternData>(myExpr, myFlags, i, myId, mode, match_limit,
                                          match_limit_recursion, platform);
        pcres.push_back(move(patternData));
        PatternData &curr = *pcres.back();

        if (!(myFlags & HS_FLAG_SINGLEMATCH)) {
            allSingleMatch = false;
        }

        // in the multimatch, we always run in prefilter mode and accept vacuous
        // patterns.
        myFlags |=
            HS_FLAG_ALLOWEMPTY | HS_FLAG_PREFILTER;

        if (curr.fixedWidth != UINT_MAX) {
            myFlags |= HS_FLAG_SOM_LEFTMOST;
            DEBUG_PRINTF("fixed width, turn off prefiltering\n");
            myFlags &= ~HS_FLAG_PREFILTER;
            allConfirm = false;

            // Single match can't coexist with SOM.
            myFlags &= ~HS_FLAG_SINGLEMATCH;
        }

        if (curr.guard) {
            // We use the index into the PCREs array as the Hyperscan idx.
            multiExpr.push_back(myExpr);
            multiFlags.push_back(myFlags);
            multiIds.push_back(i);
        } else {
            // No Hyperscan support, PCRE is unguarded.
            unguarded.push_back(i);
        }
    }

    DEBUG_PRINTF("built %zu PCREs, %zu of which are unguarded\n",
                 pcres.size(), unguarded.size());

    // Work out our sizing for the output database.
    size_t patternSize = 0;
    for (unsigned int i = 0; i < elements; i++) {
        size_t len = pcres[i]->patternSize();
        patternSize += ROUNDUP_CL(len);
    }
    DEBUG_PRINTF("pcre bytecode takes %zu bytes\n", patternSize);

    bool noMulti = multiExpr.empty();
    size_t multiSize = 0;
    hs_database *multidb = nullptr;
    if (!noMulti) {
        hs_compile_error_t *hs_comp_error = nullptr;
        hs_error_t err = hs_compile_multi(&multiExpr[0], &multiFlags[0],
                                          &multiIds[0], multiExpr.size(),
                                          HS_MODE_BLOCK, platform, &multidb,
                                          &hs_comp_error);

        if (err != HS_SUCCESS) {
            assert(hs_comp_error);
            DEBUG_PRINTF("hs_compile_multi returned error: %s\n",
                         hs_comp_error->message);
            assert(0);
            hs_free_compile_error(hs_comp_error);
            throw CompileError("Internal error.");
        }

        assert(multidb);
        err = hs_database_size(multidb, &multiSize);
        if (err != HS_SUCCESS) {
            assert(0);
            throw CompileError("Internal error.");
        }
        DEBUG_PRINTF("built hyperscan database with len %zu bytes\n", multiSize);
    }

    size_t bytecodeLen = sizeof(ch_bytecode) +
                         multiSize + alignof(u32) +
                         (sizeof(u32) * unguarded.size()) +
                         (sizeof(u32) * elements) +
                         patternSize +
                         128; // padding for alignment
    size_t totalSize = sizeof(ch_database) + bytecodeLen;

    DEBUG_PRINTF("allocating %zu bytes for database\n", totalSize);
    char *ptr = (char *)ch_database_alloc(totalSize);
    if (ch_check_alloc(ptr) != CH_SUCCESS) {
        ch_database_free(ptr);
        throw std::bad_alloc();
    }

    memset(ptr, 0, totalSize);

    // First, the header.
    ch_database *hydb = (ch_database *)ptr;
    hydb->magic = CH_DB_MAGIC;
    hydb->version = HS_VERSION_32BIT;
    hydb->length = bytecodeLen;

    // Then, the bytecode.
    size_t shift = (size_t)hydb->bytes & 0x3f;
    hydb->bytecode = offsetof(struct ch_database, bytes) - shift;
    ch_bytecode *db = (ch_bytecode *)((char *)hydb + hydb->bytecode);
    db->patternCount = elements;
    db->activeSize = mmbit_size(elements);
    db->flags = 0;
    db->length = bytecodeLen;

    if (noMulti) {
        db->flags |= CHIMERA_FLAG_NO_MULTIMATCH;
    }
    if (mode & CH_MODE_GROUPS) {
        db->flags |= CHIMERA_FLAG_GROUPS;
    }
    if (allConfirm) {
        db->flags |= CHIMERA_FLAG_ALL_CONFIRM;
    }
    if (allSingleMatch) {
        db->flags |= CHIMERA_FLAG_ALL_SINGLE;
    }


    // Find and set the max ovector size by looking at the capture count for
    // each pcre.
    u32 maxCaptureGroups = 0;
    for (unsigned int i = 0; i < elements; i++) {
        maxCaptureGroups = max(maxCaptureGroups, (u32)pcres[i]->capture_cnt);
    }
    db->maxCaptureGroups = maxCaptureGroups;
    DEBUG_PRINTF("max capture groups is %u\n", maxCaptureGroups);

    if (!noMulti) {
        DEBUG_PRINTF("write hyperscan database\n");
        // Write Hyperscan database directly after the header struct, then free it.
        if (!writeHyperscanDatabase(db, multidb)) {
            ch_database_free(hydb);
            hs_free_database(multidb);
            throw CompileError("Internal error.");
        }
        hs_free_database(multidb);
    } else {
        db->databaseOffset = ROUNDUP_CL(sizeof(*db));
    }

    // Then, write our unguarded PCRE list.
    db->unguardedCount = unguarded.size();
    db->unguardedOffset = ROUNDUP_N(db->databaseOffset + multiSize, 4);
    ptr = (char *)db + db->unguardedOffset;
    copy(unguarded.begin(), unguarded.end(), (u32 *)ptr);

    // Then, write all our compiled PCRE patterns and the lookup table for
    // them.
    db->patternOffset = db->unguardedOffset + unguarded.size() * sizeof(u32);
    u32 *patternOffset = (u32 *)((char *)db + db->patternOffset);
    u32 offset = ROUNDUP_CL(db->patternOffset + elements * sizeof(u32));
    for (unsigned int i = 0; i < elements; i++) {
        *patternOffset = offset;
        size_t len = pcres[i]->patternSize();
        ptr = (char *)db + offset;
        struct ch_pattern *pattern = (struct ch_pattern *)ptr;
        pcres[i]->writePattern(pattern);
        DEBUG_PRINTF("wrote pcre %u into offset %u, len %zu\n", i, offset, len);
        offset += ROUNDUP_CL(len);
        patternOffset++;
    }

    assert(offset <= totalSize);
    assert(hydb->magic == CH_DB_MAGIC);
    DEBUG_PRINTF("built hybrid database, size %zu bytes\n", totalSize);
    DEBUG_PRINTF("offset=%u\n", offset);
    *out = hydb;
}

} // namespace ch

extern "C" HS_PUBLIC_API
ch_error_t HS_CDECL ch_compile(const char *expression, unsigned flags,
                               unsigned mode,
                               const hs_platform_info_t *platform,
                               ch_database_t **db,
                               ch_compile_error_t **comp_error) {
    if (!comp_error) {
        if (db) {
            db = nullptr;
        }
        // nowhere to write the string, but we can still report an error code
        return CH_COMPILER_ERROR;
    }
    if (!db) {
        *comp_error =
            generateChimeraCompileError("Invalid parameter: db is NULL", -1);
        return CH_COMPILER_ERROR;
    }
    if (!expression) {
        *db = nullptr;
        *comp_error =
            generateChimeraCompileError("Invalid parameter: expressions is\
                                         NULL", -1);
        return CH_COMPILER_ERROR;
    }

    if (!checkMode(mode, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkMode
        return CH_COMPILER_ERROR;
    }

    try {
        unsigned id = 0; // single expressions get zero as an ID
        // Internal function to do all the work, now that we've handled all the
        // argument checking.
        ch::ch_compile_multi_int(&expression, &flags, &id, 1, mode, 0, 0,
                                 platform, db);
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *db = nullptr;
        *comp_error = generateChimeraCompileError(e.reason, e.hasIndex ?
                                                  (int)e.index : -1);
        return CH_COMPILER_ERROR;
    }
    catch (std::bad_alloc &) {
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_enomem);
        return CH_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal error, unexpected exception");
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_einternal);
        return CH_COMPILER_ERROR;
    }

    DEBUG_PRINTF("success!\n");
    return CH_SUCCESS;
}

extern "C" HS_PUBLIC_API
ch_error_t HS_CDECL ch_compile_multi(const char *const *expressions,
                                     const unsigned *flags, const unsigned *ids,
                                     unsigned elements, unsigned mode,
                                     const hs_platform_info_t *platform,
                                     ch_database_t **db,
                                     ch_compile_error_t **comp_error) {
    if (!comp_error) {
        if (db) {
            db = nullptr;
        }
        // nowhere to write the string, but we can still report an error code
        return CH_COMPILER_ERROR;
    }
    if (!db) {
        *comp_error =
            generateChimeraCompileError("Invalid parameter: db is NULL", -1);
        return CH_COMPILER_ERROR;
    }
    if (!expressions) {
        *db = nullptr;
        *comp_error =
            generateChimeraCompileError("Invalid parameter: expressions is\
                                         NULL", -1);
        return CH_COMPILER_ERROR;
    }
    if (!elements) {
        *db = nullptr;
        *comp_error = generateChimeraCompileError("Invalid parameter:\
                                                   elements is zero", -1);
        return CH_COMPILER_ERROR;
    }

    if (!checkMode(mode, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkMode
        return CH_COMPILER_ERROR;
    }

    try {
        // Internal function to do all the work, now that we've handled all the
        // argument checking.
        ch::ch_compile_multi_int(expressions, flags, ids, elements, mode, 0, 0,
                                 platform, db);
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *db = nullptr;
        *comp_error = generateChimeraCompileError(e.reason, e.hasIndex ?
                                                  (int)e.index : -1);
        return CH_COMPILER_ERROR;
    }
    catch (std::bad_alloc &) {
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_enomem);
        return CH_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal error, unexpected exception");
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_einternal);
        return CH_COMPILER_ERROR;
    }

    DEBUG_PRINTF("success!\n");
    return CH_SUCCESS;
}

extern "C" HS_PUBLIC_API
ch_error_t HS_CDECL ch_compile_ext_multi(
                                    const char *const *expressions,
                                    const unsigned *flags,
                                    const unsigned *ids,
                                    unsigned elements, unsigned mode,
                                    unsigned long int match_limit,
                                    unsigned long int match_limit_recursion,
                                    const hs_platform_info_t *platform,
                                    ch_database_t **db,
                                    ch_compile_error_t **comp_error) {
    if (!comp_error) {
        if (db) {
            db = nullptr;
        }
        // nowhere to write the string, but we can still report an error code
        return CH_COMPILER_ERROR;
    }
    if (!db) {
        *comp_error =
            generateChimeraCompileError("Invalid parameter: db is NULL", -1);
        return CH_COMPILER_ERROR;
    }
    if (!expressions) {
        *db = nullptr;
        *comp_error =
            generateChimeraCompileError("Invalid parameter: expressions is\
                                         NULL", -1);
        return CH_COMPILER_ERROR;
    }
    if (!elements) {
        *db = nullptr;
        *comp_error = generateChimeraCompileError("Invalid parameter:\
                                                   elements is zero", -1);
        return CH_COMPILER_ERROR;
    }

    if (!checkMode(mode, comp_error)) {
        *db = nullptr;
        assert(*comp_error); // set by checkMode
        return CH_COMPILER_ERROR;
    }

    try {
        // Internal function to do all the work, now that we've handled all the
        // argument checking.
        ch::ch_compile_multi_int(expressions, flags, ids, elements, mode,
                                 match_limit, match_limit_recursion, platform,
                                 db);
    }
    catch (const CompileError &e) {
        // Compiler error occurred
        *db = nullptr;
        *comp_error = generateChimeraCompileError(e.reason, e.hasIndex ?
                                                  (int)e.index : -1);
        return CH_COMPILER_ERROR;
    }
    catch (std::bad_alloc &) {
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_enomem);
        return CH_COMPILER_ERROR;
    }
    catch (...) {
        assert(!"Internal error, unexpected exception");
        *db = nullptr;
        *comp_error = const_cast<ch_compile_error_t *>(&ch_einternal);
        return CH_COMPILER_ERROR;
    }

    DEBUG_PRINTF("success!\n");
    return CH_SUCCESS;
}

extern "C" HS_PUBLIC_API
ch_error_t HS_CDECL ch_free_compile_error(ch_compile_error_t *error) {
    freeChimeraCompileError(error);
    return CH_SUCCESS;
}
