/*
 * Copyright (c) 2015, Intel Corporation
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
#include "asserts.h"
#include "compiler.h"
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
#include "parser/parse_error.h"
#include "parser/Parser.h"          // for flags
#include "parser/position.h"
#include "parser/position_dump.h"
#include "parser/position_info.h"
#include "parser/prefilter.h"
#include "parser/shortcut_literal.h"
#include "parser/unsupported.h"
#include "parser/utf8_validate.h"
#include "smallwrite/smallwrite_build.h"
#include "rose/rose_build.h"
#include "rose/rose_build_dump.h"
#include "som/slot_manager_dump.h"
#include "util/alloc.h"
#include "util/compile_error.h"
#include "util/target_info.h"
#include "util/verify_types.h"

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
                                                    HS_EXT_FLAG_MIN_LENGTH;
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
}

ParsedExpression::ParsedExpression(unsigned index_in, const char *expression,
                                   unsigned flags, ReportID actionId,
                                   const hs_expr_ext *ext)
    : utf8(false),
      allow_vacuous(flags & HS_FLAG_ALLOWEMPTY),
      highlander(flags & HS_FLAG_SINGLEMATCH),
      prefilter(flags & HS_FLAG_PREFILTER),
      som(SOM_NONE),
      index(index_in),
      id(actionId),
      min_offset(0),
      max_offset(MAX_OFFSET),
      min_length(0) {
    ParseMode mode(flags);

    component = parse(expression, mode);

    utf8 = mode.utf8; /* utf8 may be set by parse() */

    if (utf8 && !isValidUtf8(expression)) {
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
        som = SOM_LEFT;
    }

    // Set extended parameters, if we have them.
    if (ext) {
        // Ensure that the given parameters make sense.
        validateExt(*ext);

        if (ext->flags & HS_EXT_FLAG_MIN_OFFSET) {
            min_offset = ext->min_offset;
        }
        if (ext->flags & HS_EXT_FLAG_MAX_OFFSET) {
            max_offset = ext->max_offset;
        }
        if (ext->flags & HS_EXT_FLAG_MIN_LENGTH) {
            min_length = ext->min_length;
        }
    }

    // These are validated in validateExt, so an error will already have been
    // thrown if these conditions don't hold.
    assert(max_offset >= min_offset);
    assert(max_offset >= min_length);

    // Since prefiltering and SOM aren't supported together, we must squash any
    // min_length constraint as well.
    if (flags & HS_FLAG_PREFILTER && min_length) {
        DEBUG_PRINTF("prefiltering mode: squashing min_length constraint\n");
        min_length = 0;
    }
}

#if defined(DUMP_SUPPORT) || defined(DEBUG)
/**
 * \brief Dumps the parse tree to screen in debug mode and to disk in dump
 * mode.
 */
void dumpExpression(UNUSED const ParsedExpression &expr,
                    UNUSED const char *stage, UNUSED const Grey &grey) {
#if defined(DEBUG)
    DEBUG_PRINTF("===== Rule ID: %u (internalID:  %u) =====\n", expr.id,
                 expr.index);
    ostringstream debug_tree;
    dumpTree(debug_tree, expr.component.get());
    printf("%s\n", debug_tree.str().c_str());
#endif // DEBUG

#if defined(DUMP_SUPPORT)
    if (grey.dumpFlags & Grey::DUMP_PARSE) {
        stringstream ss;
        ss << grey.dumpPath << "Expr_" << expr.index << "_componenttree_"
           << stage << ".txt";
        ofstream out(ss.str().c_str());
        out << "Component Tree for " << expr.id << endl;
        dumpTree(out, expr.component.get());
        if (expr.utf8) {
            out << "UTF8 mode" << endl;
        }
    }
#endif // DEBUG
}
#endif

/** \brief Run Component tree optimisations on \a expr. */
static
void optimise(ParsedExpression &expr) {
    if (expr.min_length || expr.som) {
        return;
    }

    DEBUG_PRINTF("optimising\n");
    expr.component->optimise(true /* root is connected to sds */);
}

void addExpression(NG &ng, unsigned index, const char *expression,
                   unsigned flags, const hs_expr_ext *ext, ReportID id) {
    assert(expression);
    const CompileContext &cc = ng.cc;
    DEBUG_PRINTF("index=%u, id=%u, flags=%u, expr='%s'\n", index, id, flags,
                 expression);

    // Ensure that our pattern isn't too long (in characters).
    if (strlen(expression) > cc.grey.limitPatternLength) {
        throw CompileError("Pattern length exceeds limit.");
    }

    // Do per-expression processing: errors here will result in an exception
    // being thrown up to our caller
    ParsedExpression expr(index, expression, flags, id, ext);
    dumpExpression(expr, "orig", cc.grey);

    // Apply prefiltering transformations if desired.
    if (expr.prefilter) {
        prefilterTree(expr.component, ParseMode(flags));
        dumpExpression(expr, "prefiltered", cc.grey);
    }

    // Expressions containing zero-width assertions and other extended pcre
    // types aren't supported yet. This call will throw a ParseError exception
    // if the component tree contains such a construct.
    checkUnsupported(*expr.component);

    expr.component->checkEmbeddedStartAnchor(true);
    expr.component->checkEmbeddedEndAnchor(true);

    if (cc.grey.optimiseComponentTree) {
        optimise(expr);
        dumpExpression(expr, "opt", cc.grey);
    }

    DEBUG_PRINTF("component=%p, nfaId=%u, reportId=%u\n",
                 expr.component.get(), expr.index, expr.id);

    // You can only use the SOM flags if you've also specified an SOM
    // precision mode.
    if (expr.som != SOM_NONE && cc.streaming && !ng.ssm.somPrecision()) {
        throw CompileError("To use a SOM expression flag in streaming mode, "
                           "an SOM precision mode (e.g. "
                           "HS_MODE_SOM_HORIZON_LARGE) must be specified.");
    }

    // If this expression is a literal, we can feed it directly to Rose rather
    // than building the NFA graph.
    if (shortcutLiteral(ng, expr)) {
        DEBUG_PRINTF("took literal short cut\n");
        return;
    }

    unique_ptr<NGWrapper> g = buildWrapper(ng.rm, cc, expr);

    if (!g) {
        DEBUG_PRINTF("NFA build failed on ID %u, but no exception was "
                     "thrown.\n", expr.id);
        throw CompileError("Internal error.");
    }

    if (!expr.allow_vacuous && matches_everywhere(*g)) {
        throw CompileError("Pattern matches empty buffer; use "
                           "HS_FLAG_ALLOWEMPTY to enable support.");
    }

    if (!ng.addGraph(*g)) {
        DEBUG_PRINTF("NFA addGraph failed on ID %u.\n", expr.id);
        throw CompileError("Error compiling expression.");
    }
}

static
aligned_unique_ptr<RoseEngine> generateRoseEngine(NG &ng) {
    const u32 minWidth =
        ng.minWidth.is_finite() ? verify_u32(ng.minWidth) : ROSE_BOUND_INF;
    auto rose = ng.rose->buildRose(minWidth);

    if (!rose) {
        DEBUG_PRINTF("error building rose\n");
        assert(0);
        return nullptr;
    }

    /* avoid building a smwr if just a pure floating case. */
    if (!roseIsPureLiteral(rose.get())) {
        u32 qual = roseQuality(rose.get());
        auto smwr = ng.smwr->build(qual);
        if (smwr) {
            rose = roseAddSmallWrite(rose.get(), smwr.get());
        }
    }

    dumpRose(*ng.rose, rose.get(), ng.cc.grey);
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
    return p;
}

struct hs_database *build(NG &ng, unsigned int *length) {
    assert(length);

    auto rose = generateRoseEngine(ng);
    if (!rose) {
        throw CompileError("Unable to generate bytecode.");
    }
    *length = roseSize(rose.get());
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

unique_ptr<NGWrapper> buildWrapper(ReportManager &rm, const CompileContext &cc,
                                   const ParsedExpression &expr) {
    assert(isSupported(*expr.component));

    const unique_ptr<NFABuilder> builder = makeNFABuilder(rm, cc, expr);
    assert(builder);

    // Set up START and ACCEPT states; retrieve the special states
    const auto bs = makeGlushkovBuildState(*builder, expr.prefilter);

    // Map position IDs to characters/components
    expr.component->notePositions(*bs);

    // Wire the start dotstar state to the firsts
    connectInitialStates(*bs, expr);

    DEBUG_PRINTF("wire up body of expr\n");
    // Build the rest of the FOLLOW set
    vector<PositionInfo> initials = {builder->getStartDotStar(),
                                     builder->getStart()};
    expr.component->buildFollowSet(*bs, initials);

    // Wire the lasts to the accept state
    connectFinalStates(*bs, expr);

    // Create our edges
    bs->buildEdges();

    auto g = builder->getGraph();
    assert(g);

    dumpDotWrapper(*g, "00_before_asserts", cc.grey);
    removeAssertVertices(rm, *g);

    return g;
}

} // namespace ue2
