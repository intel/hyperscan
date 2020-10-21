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
 * \brief Compiler front-end interface.
 */
#include "allocator.h"
#include "asserts.h"
#include "compiler.h"
#include "crc32.h"
#include "database.h"
#include "grey.h"
#include "hs_internal.h"
#include "hs_runtime.h"
#include "ue2common.h"
#include "nfagraph/ng_builder.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_util.h"
#include "parser/buildstate.h"
#include "parser/dump.h"
#include "parser/Component.h"
#include "parser/logical_combination.h"
#include "parser/parse_error.h"
#include "parser/Parser.h"          // for flags
#include "parser/position.h"
#include "parser/position_dump.h"
#include "parser/position_info.h"
#include "parser/prefilter.h"
#include "parser/shortcut_literal.h"
#include "parser/unsupported.h"
#include "parser/utf8_validate.h"
#include "rose/rose_build.h"
#include "rose/rose_internal.h"
#include "som/slot_manager_dump.h"
#include "util/bytecode_ptr.h"
#include "util/compile_error.h"
#include "util/target_info.h"
#include "util/verify_types.h"
#include "util/ue2string.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>

using namespace std;

namespace ue2 {

static
void validateExt(const hs_expr_ext &ext) {
    static const unsigned long long ALL_EXT_FLAGS = HS_EXT_FLAG_MIN_OFFSET |
                                                    HS_EXT_FLAG_MAX_OFFSET |
                                                    HS_EXT_FLAG_MIN_LENGTH |
                                                    HS_EXT_FLAG_EDIT_DISTANCE |
                                                    HS_EXT_FLAG_HAMMING_DISTANCE;
    if (ext.flags & ~ALL_EXT_FLAGS) {
        throw CompileError("Invalid hs_expr_ext flag set.");
    }

    if ((ext.flags & HS_EXT_FLAG_MIN_OFFSET) &&
        (ext.flags & HS_EXT_FLAG_MAX_OFFSET) &&
        (ext.min_offset > ext.max_offset)) {
        throw CompileError("In hs_expr_ext, min_offset must be less than or "
                           "equal to max_offset.");
    }

    if ((ext.flags & HS_EXT_FLAG_MIN_LENGTH) &&
        (ext.flags & HS_EXT_FLAG_MAX_OFFSET) &&
        (ext.min_length > ext.max_offset)) {
        throw CompileError("In hs_expr_ext, min_length must be less than or "
                           "equal to max_offset.");
    }

    if ((ext.flags & HS_EXT_FLAG_EDIT_DISTANCE) &&
        (ext.flags & HS_EXT_FLAG_HAMMING_DISTANCE)) {
        throw CompileError("In hs_expr_ext, cannot have both edit distance and "
                           "Hamming distance.");
    }

}

void ParsedLitExpression::parseLiteral(const char *expression, size_t len,
                                       bool nocase) {
    const char *c = expression;
    for (size_t i = 0; i < len; i++) {
        lit.push_back(*c, nocase);
        c++;
    }
}

ParsedLitExpression::ParsedLitExpression(unsigned index_in,
                                         const char *expression,
                                         size_t expLength, unsigned flags,
                                         ReportID report)
    : expr(index_in, false, flags & HS_FLAG_SINGLEMATCH, false, false,
           SOM_NONE, report, 0, MAX_OFFSET, 0, 0, 0, false) {
    // For pure literal expression, below 'HS_FLAG_'s are unuseful:
    // DOTALL/ALLOWEMPTY/UTF8/UCP/PREFILTER/COMBINATION/QUIET/MULTILINE

    if (flags & ~HS_FLAG_ALL) {
        DEBUG_PRINTF("Unrecognised flag, flags=%u.\n", flags);
        throw CompileError("Unrecognised flag.");
    }

    // FIXME: we disallow highlander + SOM, see UE-1850.
    if ((flags & HS_FLAG_SINGLEMATCH) && (flags & HS_FLAG_SOM_LEFTMOST)) {
        throw CompileError("HS_FLAG_SINGLEMATCH is not supported in "
                           "combination with HS_FLAG_SOM_LEFTMOST.");
    }

    // Set SOM type.
    if (flags & HS_FLAG_SOM_LEFTMOST) {
        expr.som = SOM_LEFT;
    }

    // Transfer expression text into ue2_literal.
    bool nocase = flags & HS_FLAG_CASELESS ? true : false;
    parseLiteral(expression, expLength, nocase);

}

ParsedExpression::ParsedExpression(unsigned index_in, const char *expression,
                                   unsigned flags, ReportID report,
                                   const hs_expr_ext *ext)
    : expr(index_in, flags & HS_FLAG_ALLOWEMPTY, flags & HS_FLAG_SINGLEMATCH,
           false, flags & HS_FLAG_PREFILTER, SOM_NONE, report, 0, MAX_OFFSET,
           0, 0, 0, flags & HS_FLAG_QUIET) {
    // We disallow SOM + Quiet.
    if ((flags & HS_FLAG_QUIET) && (flags & HS_FLAG_SOM_LEFTMOST)) {
        throw CompileError("HS_FLAG_QUIET is not supported in "
                           "combination with HS_FLAG_SOM_LEFTMOST.");
    }
    flags &= ~HS_FLAG_QUIET;
    ParseMode mode(flags);

    component = parse(expression, mode);

    expr.utf8 = mode.utf8; /* utf8 may be set by parse() */

    const size_t len = strlen(expression);
    if (expr.utf8 && !isValidUtf8(expression, len)) {
        throw ParseError("Expression is not valid UTF-8.");
    }

    if (!component) {
        assert(0); // parse() should have thrown a ParseError.
        throw ParseError("Parse error.");
    }

    if (flags & ~HS_FLAG_ALL) {
        DEBUG_PRINTF("Unrecognised flag, flags=%u.\n", flags);
        throw CompileError("Unrecognised flag.");
    }

    // FIXME: we disallow highlander + SOM, see UE-1850.
    if ((flags & HS_FLAG_SINGLEMATCH) && (flags & HS_FLAG_SOM_LEFTMOST)) {
        throw CompileError("HS_FLAG_SINGLEMATCH is not supported in "
                           "combination with HS_FLAG_SOM_LEFTMOST.");
    }

    // FIXME: we disallow prefilter + SOM, see UE-1899.
    if ((flags & HS_FLAG_PREFILTER) && (flags & HS_FLAG_SOM_LEFTMOST)) {
        throw CompileError("HS_FLAG_PREFILTER is not supported in "
                           "combination with HS_FLAG_SOM_LEFTMOST.");
    }

    // Set SOM type.
    if (flags & HS_FLAG_SOM_LEFTMOST) {
        expr.som = SOM_LEFT;
    }

    // Set extended parameters, if we have them.
    if (ext) {
        // Ensure that the given parameters make sense.
        validateExt(*ext);

        if (ext->flags & HS_EXT_FLAG_MIN_OFFSET) {
            expr.min_offset = ext->min_offset;
        }
        if (ext->flags & HS_EXT_FLAG_MAX_OFFSET) {
            expr.max_offset = ext->max_offset;
        }
        if (ext->flags & HS_EXT_FLAG_MIN_LENGTH) {
            expr.min_length = ext->min_length;
        }
        if (ext->flags & HS_EXT_FLAG_EDIT_DISTANCE) {
            expr.edit_distance = ext->edit_distance;
        }
        if (ext->flags & HS_EXT_FLAG_HAMMING_DISTANCE) {
            expr.hamm_distance = ext->hamming_distance;
        }
    }

    // These are validated in validateExt, so an error will already have been
    // thrown if these conditions don't hold.
    assert(expr.max_offset >= expr.min_offset);
    assert(expr.max_offset >= expr.min_length);

    // Since prefiltering and SOM aren't supported together, we must squash any
    // min_length constraint as well.
    if (flags & HS_FLAG_PREFILTER && expr.min_length) {
        DEBUG_PRINTF("prefiltering mode: squashing min_length constraint\n");
        expr.min_length = 0;
    }
}

#if defined(DUMP_SUPPORT) || defined(DEBUG)
/**
 * \brief Dumps the parse tree to screen in debug mode and to disk in dump
 * mode.
 */
void dumpExpression(UNUSED const ParsedExpression &pe,
                    UNUSED const char *stage, UNUSED const Grey &grey) {
#if defined(DEBUG)
    DEBUG_PRINTF("===== Rule ID: %u (expression index: %u) =====\n",
                 pe.expr.report, pe.expr.index);
    ostringstream debug_tree;
    dumpTree(debug_tree, pe.component.get());
    printf("%s\n", debug_tree.str().c_str());
#endif // DEBUG

#if defined(DUMP_SUPPORT)
    if (grey.dumpFlags & Grey::DUMP_PARSE) {
        stringstream ss;
        ss << grey.dumpPath << "Expr_" << pe.expr.index << "_componenttree_"
           << stage << ".txt";
        ofstream out(ss.str().c_str());
        out << "Component Tree for " << pe.expr.report << endl;
        dumpTree(out, pe.component.get());
        if (pe.expr.utf8) {
            out << "UTF8 mode" << endl;
        }
    }
#endif // DEBUG
}
#endif

/** \brief Run Component tree optimisations on \a expr. */
static
void optimise(ParsedExpression &pe) {
    if (pe.expr.min_length || pe.expr.som) {
        return;
    }

    DEBUG_PRINTF("optimising\n");
    pe.component->optimise(true /* root is connected to sds */);
}

void addExpression(NG &ng, unsigned index, const char *expression,
                   unsigned flags, const hs_expr_ext *ext, ReportID id) {
    assert(expression);
    const CompileContext &cc = ng.cc;
    DEBUG_PRINTF("index=%u, id=%u, flags=%u, expr='%s'\n", index, id, flags,
                 expression);

    if (flags & HS_FLAG_COMBINATION) {
        if (flags & ~(HS_FLAG_COMBINATION | HS_FLAG_QUIET |
                      HS_FLAG_SINGLEMATCH)) {
            throw CompileError("only HS_FLAG_QUIET and HS_FLAG_SINGLEMATCH "
                               "are supported in combination "
                               "with HS_FLAG_COMBINATION.");
        }
        if (flags & HS_FLAG_QUIET) {
            DEBUG_PRINTF("skip QUIET logical combination expression %u\n", id);
        } else {
            u32 ekey = INVALID_EKEY;
            u64a min_offset = 0;
            u64a max_offset = MAX_OFFSET;
            if (flags & HS_FLAG_SINGLEMATCH) {
                ekey = ng.rm.getExhaustibleKey(id);
            }
            if (ext) {
                validateExt(*ext);
                if (ext->flags & ~(HS_EXT_FLAG_MIN_OFFSET |
                                   HS_EXT_FLAG_MAX_OFFSET)) {
                    throw CompileError("only HS_EXT_FLAG_MIN_OFFSET and "
                                       "HS_EXT_FLAG_MAX_OFFSET extra flags "
                                       "are supported in combination "
                                       "with HS_FLAG_COMBINATION.");
                }
                if (ext->flags & HS_EXT_FLAG_MIN_OFFSET) {
                    min_offset = ext->min_offset;
                }
                if (ext->flags & HS_EXT_FLAG_MAX_OFFSET) {
                    max_offset = ext->max_offset;
                }
            }
            ng.rm.pl.parseLogicalCombination(id, expression, ekey, min_offset,
                                             max_offset);
            DEBUG_PRINTF("parsed logical combination expression %u\n", id);
        }
        return;
    }

    // Ensure that our pattern isn't too long (in characters).
    if (strlen(expression) > cc.grey.limitPatternLength) {
        throw CompileError("Pattern length exceeds limit.");
    }

    // Do per-expression processing: errors here will result in an exception
    // being thrown up to our caller
    ParsedExpression pe(index, expression, flags, id, ext);
    dumpExpression(pe, "orig", cc.grey);

    // Apply prefiltering transformations if desired.
    if (pe.expr.prefilter) {
        prefilterTree(pe.component, ParseMode(flags));
        dumpExpression(pe, "prefiltered", cc.grey);
    }

    // Expressions containing zero-width assertions and other extended pcre
    // types aren't supported yet. This call will throw a ParseError exception
    // if the component tree contains such a construct.
    checkUnsupported(*pe.component);

    pe.component->checkEmbeddedStartAnchor(true);
    pe.component->checkEmbeddedEndAnchor(true);

    if (cc.grey.optimiseComponentTree) {
        optimise(pe);
        dumpExpression(pe, "opt", cc.grey);
    }

    DEBUG_PRINTF("component=%p, nfaId=%u, reportId=%u\n",
                 pe.component.get(), pe.expr.index, pe.expr.report);

    // You can only use the SOM flags if you've also specified an SOM
    // precision mode.
    if (pe.expr.som != SOM_NONE && cc.streaming && !ng.ssm.somPrecision()) {
        throw CompileError("To use a SOM expression flag in streaming mode, "
                           "an SOM precision mode (e.g. "
                           "HS_MODE_SOM_HORIZON_LARGE) must be specified.");
    }

    // If this expression is a literal, we can feed it directly to Rose rather
    // than building the NFA graph.
    if (shortcutLiteral(ng, pe)) {
        DEBUG_PRINTF("took literal short cut\n");
        return;
    }

    auto built_expr = buildGraph(ng.rm, cc, pe);
    if (!built_expr.g) {
        DEBUG_PRINTF("NFA build failed on ID %u, but no exception was "
                     "thrown.\n", pe.expr.report);
        throw CompileError("Internal error.");
    }

    if (!pe.expr.allow_vacuous && matches_everywhere(*built_expr.g)) {
        throw CompileError("Pattern matches empty buffer; use "
                           "HS_FLAG_ALLOWEMPTY to enable support.");
    }

    if (!ng.addGraph(built_expr.expr, std::move(built_expr.g))) {
        DEBUG_PRINTF("NFA addGraph failed on ID %u.\n", pe.expr.report);
        throw CompileError("Error compiling expression.");
    }
}

void addLitExpression(NG &ng, unsigned index, const char *expression,
                      unsigned flags, const hs_expr_ext *ext, ReportID id,
                      size_t expLength) {
    assert(expression);
    const CompileContext &cc = ng.cc;
    DEBUG_PRINTF("index=%u, id=%u, flags=%u, expr='%s', len='%zu'\n", index,
                 id, flags, expression, expLength);

    // Extended parameters are not supported for pure literal patterns.
    if (ext && ext->flags != 0LLU) {
        throw CompileError("Extended parameters are not supported for pure "
                           "literal matching API.");
    }

    // Ensure that our pattern isn't too long (in characters).
    if (expLength > cc.grey.limitPatternLength) {
        throw CompileError("Pattern length exceeds limit.");
    }

    // filter out flags not supported by pure literal API.
    u64a not_supported = HS_FLAG_DOTALL | HS_FLAG_ALLOWEMPTY | HS_FLAG_UTF8 |
                         HS_FLAG_UCP | HS_FLAG_PREFILTER | HS_FLAG_COMBINATION |
                         HS_FLAG_QUIET | HS_FLAG_MULTILINE;

    if (flags & not_supported) {
        throw CompileError("Only HS_FLAG_CASELESS, HS_FLAG_SINGLEMATCH and "
                           "HS_FLAG_SOM_LEFTMOST are supported in literal API.");
    }

    // This expression must be a pure literal, we can build ue2_literal
    // directly based on expression text.
    ParsedLitExpression ple(index, expression, expLength, flags, id);

    // Feed the ue2_literal into Rose.
    const auto &expr = ple.expr;
    if (ng.addLiteral(ple.lit, expr.index, expr.report, expr.highlander,
                      expr.som, expr.quiet)) {
        DEBUG_PRINTF("took pure literal\n");
        return;
    }
}

static
bytecode_ptr<RoseEngine> generateRoseEngine(NG &ng) {
    const u32 minWidth =
        ng.minWidth.is_finite() ? verify_u32(ng.minWidth) : ROSE_BOUND_INF;
    auto rose = ng.rose->buildRose(minWidth);

    if (!rose) {
        DEBUG_PRINTF("error building rose\n");
        assert(0);
        return nullptr;
    }

    dumpReportManager(ng.rm, ng.cc.grey);
    dumpSomSlotManager(ng.ssm, ng.cc.grey);
    dumpSmallWrite(rose.get(), ng.cc.grey);

    return rose;
}

platform_t target_to_platform(const target_t &target_info) {
    platform_t p;
    p = 0;

    if (!target_info.has_avx2()) {
        p |= HS_PLATFORM_NOAVX2;
    }
    if (!target_info.has_avx512()) {
        p |= HS_PLATFORM_NOAVX512;
    }
    if (!target_info.has_avx512vbmi()) {
        p |= HS_PLATFORM_NOAVX512VBMI;
    }
    return p;
}

/** \brief Encapsulate the given bytecode (RoseEngine) in a newly-allocated
 * \ref hs_database, ensuring that it is padded correctly to give cacheline
 * alignment.  */
static
hs_database_t *dbCreate(const char *in_bytecode, size_t len, u64a platform) {
    size_t db_len = sizeof(struct hs_database) + len;
    DEBUG_PRINTF("db size %zu\n", db_len);
    DEBUG_PRINTF("db platform %llx\n", platform);

    struct hs_database *db = (struct hs_database *)hs_database_alloc(db_len);
    if (hs_check_alloc(db) != HS_SUCCESS) {
        hs_database_free(db);
        return nullptr;
    }

    // So that none of our database is uninitialized
    memset(db, 0, db_len);

    // we need to align things manually
    size_t shift = (uintptr_t)db->bytes & 0x3f;
    DEBUG_PRINTF("shift is %zu\n", shift);

    db->bytecode = offsetof(struct hs_database, bytes) - shift;
    char *bytecode = (char *)db + db->bytecode;
    assert(ISALIGNED_CL(bytecode));

    db->magic = HS_DB_MAGIC;
    db->version = HS_DB_VERSION;
    db->length = len;
    db->platform = platform;

    // Copy bytecode
    memcpy(bytecode, in_bytecode, len);

    db->crc32 = Crc32c_ComputeBuf(0, bytecode, db->length);
    return db;
}


struct hs_database *build(NG &ng, unsigned int *length, u8 pureFlag) {
    assert(length);

    auto rose = generateRoseEngine(ng);
    struct RoseEngine *roseHead = rose.get();
    roseHead->pureLiteral = pureFlag;

    if (!rose) {
        throw CompileError("Unable to generate bytecode.");
    }
    *length = rose.size();
    if (!*length) {
        DEBUG_PRINTF("RoseEngine has zero length\n");
        assert(0);
        throw CompileError("Internal error.");
    }

    const char *bytecode = (const char *)(rose.get());
    const platform_t p = target_to_platform(ng.cc.target_info);
    struct hs_database *db = dbCreate(bytecode, *length, p);
    if (!db) {
        throw CompileError("Could not allocate memory for bytecode.");
    }

    return db;
}

static
void stripFromPositions(vector<PositionInfo> &v, Position pos) {
    auto removed = remove(v.begin(), v.end(), PositionInfo(pos));
    v.erase(removed, v.end());
}

static
void connectInitialStates(GlushkovBuildState &bs,
                          const ParsedExpression &expr) {
    vector<PositionInfo> initials = expr.component->first();
    const NFABuilder &builder = bs.getBuilder();
    const Position startState = builder.getStart();
    const Position startDotStarState = builder.getStartDotStar();

    DEBUG_PRINTF("wiring initials = %s\n",
                 dumpPositions(initials.begin(), initials.end()).c_str());

    vector<PositionInfo> starts = {startState, startDotStarState};

    // strip start and startDs, which can be present due to boundaries
    stripFromPositions(initials, startState);
    stripFromPositions(initials, startDotStarState);

    // replace epsilons with accepts
    for (const auto &s : initials) {
        if (s.pos != GlushkovBuildState::POS_EPSILON) {
            continue;
        }

        assert(starts.size() == 2); /* start, startds */
        vector<PositionInfo> starts_temp = starts;
        starts_temp[0].flags = s.flags;
        starts_temp[1].flags = s.flags;
        bs.connectAccepts(starts_temp);
    }

    if (!initials.empty()) {
        bs.connectRegions(starts, initials);
    }
}

static
void connectFinalStates(GlushkovBuildState &bs, const ParsedExpression &expr) {
    vector<PositionInfo> finals = expr.component->last();

    DEBUG_PRINTF("wiring finals = %s\n",
                 dumpPositions(finals.begin(), finals.end()).c_str());

    bs.connectAccepts(finals);
}

#ifndef NDEBUG
static
bool isSupported(const Component &c) {
    try {
        checkUnsupported(c);
        return true;
    }
    catch (ParseError &) {
        return false;
    }
}
#endif

BuiltExpression buildGraph(ReportManager &rm, const CompileContext &cc,
                           const ParsedExpression &pe) {
    assert(isSupported(*pe.component));

    const auto builder = makeNFABuilder(rm, cc, pe);
    assert(builder);

    // Set up START and ACCEPT states; retrieve the special states
    const auto bs = makeGlushkovBuildState(*builder, pe.expr.prefilter);

    // Map position IDs to characters/components
    pe.component->notePositions(*bs);

    // Wire the start dotstar state to the firsts
    connectInitialStates(*bs, pe);

    DEBUG_PRINTF("wire up body of expr\n");
    // Build the rest of the FOLLOW set
    vector<PositionInfo> initials = {builder->getStartDotStar(),
                                     builder->getStart()};
    pe.component->buildFollowSet(*bs, initials);

    // Wire the lasts to the accept state
    connectFinalStates(*bs, pe);

    // Create our edges
    bs->buildEdges();

    BuiltExpression built_expr = builder->getGraph();
    assert(built_expr.g);

    dumpDotWrapper(*built_expr.g, built_expr.expr, "00_before_asserts",
                   cc.grey);
    removeAssertVertices(rm, *built_expr.g, built_expr.expr);

    return built_expr;
}

} // namespace ue2
