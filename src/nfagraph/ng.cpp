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

/** \file
 * \brief NG and graph handling.
 */
#include "ng.h"

#include "grey.h"
#include "ng_anchored_acyclic.h"
#include "ng_anchored_dots.h"
#include "ng_asserts.h"
#include "ng_calc_components.h"
#include "ng_cyclic_redundancy.h"
#include "ng_dump.h"
#include "ng_edge_redundancy.h"
#include "ng_equivalence.h"
#include "ng_extparam.h"
#include "ng_fixed_width.h"
#include "ng_fuzzy.h"
#include "ng_haig.h"
#include "ng_literal_component.h"
#include "ng_literal_decorated.h"
#include "ng_misc_opt.h"
#include "ng_puff.h"
#include "ng_prefilter.h"
#include "ng_prune.h"
#include "ng_redundancy.h"
#include "ng_region.h"
#include "ng_region_redundancy.h"
#include "ng_reports.h"
#include "ng_sep.h"
#include "ng_small_literal_set.h"
#include "ng_som.h"
#include "ng_vacuous.h"
#include "ng_violet.h"
#include "ng_utf8.h"
#include "ng_util.h"
#include "ng_width.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "nfa/goughcompile.h"
#include "rose/rose_build.h"
#include "smallwrite/smallwrite_build.h"
#include "util/compile_error.h"
#include "util/container.h"
#include "util/depth.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/ue2string.h"

using namespace std;

namespace ue2 {

NG::NG(const CompileContext &in_cc, size_t num_patterns,
       unsigned in_somPrecision)
    : maxSomRevHistoryAvailable(in_cc.grey.somMaxRevNfaLength),
      minWidth(depth::infinity()),
      rm(in_cc.grey),
      ssm(in_somPrecision),
      cc(in_cc),
      smwr(makeSmallWriteBuilder(num_patterns, rm, cc)),
      rose(makeRoseBuilder(rm, ssm, *smwr, cc, boundary)) {
}

NG::~NG() {
    // empty
}

/** \brief SOM handling code, called by \ref addComponent.
 *
 * \return true if the component was handled completely by something (e.g. a
 * Haig outfix), false if SOM could be established but implementation via an
 * engine will be required.
 *
 * \throw CompileError if SOM cannot be supported for the component.
 */
static
bool addComponentSom(NG &ng, NGHolder &g, const ExpressionInfo &expr,
                     const som_type som, const u32 comp_id) {
    DEBUG_PRINTF("doing som\n");
    dumpComponent(g, "03_presom", expr.index, comp_id, ng.cc.grey);
    assert(hasCorrectlyNumberedVertices(g));
    assert(allMatchStatesHaveReports(g));

    // First, we try the "SOM chain" support in ng_som.cpp.

    sombe_rv rv = doSom(ng, g, expr, comp_id, som);
    if (rv == SOMBE_HANDLED_INTERNAL) {
        return false;
    } else if (rv == SOMBE_HANDLED_ALL) {
        return true;
    }
    assert(rv == SOMBE_FAIL);

    /* Next, Sombe style approaches */
    rv = doSomWithHaig(ng, g, expr, comp_id, som);
    if (rv == SOMBE_HANDLED_INTERNAL) {
        return false;
    } else if (rv == SOMBE_HANDLED_ALL) {
        return true;
    }
    assert(rv == SOMBE_FAIL);

    // If the previous approach could not support this pattern, we try treating
    // it monolithically, as a Haig outfix.

    vector<vector<CharReach> > triggers; /* empty for outfix */

    assert(g.kind == NFA_OUTFIX);
    dumpComponent(g, "haig", expr.index, comp_id, ng.cc.grey);
    makeReportsSomPass(ng.rm, g);
    auto haig = attemptToBuildHaig(g, som, ng.ssm.somPrecision(), triggers,
                                   ng.cc.grey);
    if (haig) {
        DEBUG_PRINTF("built haig outfix\n");
        ng.rose->addOutfix(g, *haig);
        return true;
    }

    /* Our various strategies for supporting SOM for this pattern have failed.
     * Provide a generic pattern not supported/too large return value as it is
     * unclear what the meaning of a specific SOM error would be */
    throw CompileError(expr.index, "Pattern is too large.");

    assert(0); // unreachable
    return false;
}

void reduceGraph(NGHolder &g, som_type som, bool utf8,
                 const CompileContext &cc) {
    if (!cc.grey.performGraphSimplification) {
        return;
    }

    // We run reduction passes until either the graph stops changing or we hit
    // a (small) limit.

    if (!som) {
        mergeCyclicDotStars(g);
    }

    const unsigned MAX_PASSES = 3;
    for (unsigned pass = 1; pass <= MAX_PASSES; pass++) {
        bool changed = false;
        DEBUG_PRINTF("reduce pass %u/%u\n", pass, MAX_PASSES);
        changed |= removeEdgeRedundancy(g, som, cc);
        changed |= reduceGraphEquivalences(g, cc);
        changed |= removeRedundancy(g, som);
        changed |= removeCyclicPathRedundancy(g);
        if (!changed) {
            DEBUG_PRINTF("graph unchanged after pass %u, stopping\n", pass);
            break;
        }
    }

    if (utf8) {
        utf8DotRestoration(g, som);
    }

    /* Minor non-redundancy improvements */
    if (improveGraph(g, som)) {
        /* may be some more edges to remove */
        removeEdgeRedundancy(g, som, cc);
    }

    removeCyclicDominated(g, som);

    if (!som) {
        mergeCyclicDotStars(g);
    }

    if (!som) {
        removeSiblingsOfStartDotStar(g);
    }
}

static
bool addComponent(NG &ng, NGHolder &g, const ExpressionInfo &expr,
                  const som_type som, const u32 comp_id) {
    const CompileContext &cc = ng.cc;
    assert(hasCorrectlyNumberedVertices(g));

    DEBUG_PRINTF("expr=%u, comp=%u: %zu vertices, %zu edges\n",
                 expr.index, comp_id, num_vertices(g), num_edges(g));

    dumpComponent(g, "01_begin", expr.index, comp_id, ng.cc.grey);

    assert(allMatchStatesHaveReports(g));

    reduceExtendedParams(g, ng.rm, som);
    reduceGraph(g, som, expr.utf8, cc);

    dumpComponent(g, "02_reduced", expr.index, comp_id, ng.cc.grey);

    // There may be redundant regions that we can remove
    if (cc.grey.performGraphSimplification) {
        removeRegionRedundancy(g, som);
    }

    // We might be done at this point: if we've run out of vertices, we can
    // stop processing.
    if (num_vertices(g) == N_SPECIALS) {
        DEBUG_PRINTF("all vertices claimed\n");
        return true;
    }

    // "Short Exhaustible Passthrough" patterns always become outfixes.
    if (!som && isSEP(g, ng.rm, cc.grey)) {
        DEBUG_PRINTF("graph is SEP\n");
        if (ng.rose->addOutfix(g)) {
            return true;
        }
    }

    // Start Of Match handling.
    if (som) {
        if (addComponentSom(ng, g, expr, som, comp_id)) {
            return true;
        }
    }

    assert(allMatchStatesHaveReports(g));

    if (splitOffAnchoredAcyclic(*ng.rose, g, cc)) {
        return true;
    }

    if (handleSmallLiteralSets(*ng.rose, g, cc)
        || handleFixedWidth(*ng.rose, g, cc.grey)) {
        return true;
    }

    if (handleDecoratedLiterals(*ng.rose, g, cc)) {
        return true;
    }

    if (doViolet(*ng.rose, g, expr.prefilter, false, ng.rm, cc)) {
        return true;
    }

    if (splitOffPuffs(*ng.rose, ng.rm, g, expr.prefilter, cc)) {
        return true;
    }

    if (handleSmallLiteralSets(*ng.rose, g, cc)
        || handleFixedWidth(*ng.rose, g, cc.grey)) {
        return true;
    }

    if (handleDecoratedLiterals(*ng.rose, g, cc)) {
        return true;
    }

    if (doViolet(*ng.rose, g, expr.prefilter, true, ng.rm, cc)) {
        return true;
    }

    DEBUG_PRINTF("testing for outfix\n");
    assert(allMatchStatesHaveReports(g));
    if (ng.rose->addOutfix(g)) {
        return true;
    }

    return false;
}

// Returns true if all components have been added.
static
bool processComponents(NG &ng, ExpressionInfo &expr,
                       deque<unique_ptr<NGHolder>> &g_comp,
                       const som_type som) {
    const u32 num_components = g_comp.size();

    u32 failed = 0;
    for (u32 i = 0; i < num_components; i++) {
        if (!g_comp[i]) {
            continue;
        }
        if (addComponent(ng, *g_comp[i], expr, som, i)) {
            g_comp[i].reset();
            continue;
        }

        if (som) { /* bail immediately */
            return false;
        }
        failed++;
    }

    if (!failed) {
        DEBUG_PRINTF("all components claimed\n");
        return true;
    }

    DEBUG_PRINTF("%u components still remain\n", failed);
    return false;
}

bool NG::addGraph(ExpressionInfo &expr, unique_ptr<NGHolder> g_ptr) {
    assert(g_ptr);
    NGHolder &g = *g_ptr;

    // remove reports that aren't on vertices connected to accept.
    clearReports(g);

    som_type som = expr.som;
    if (som && isVacuous(g)) {
        throw CompileError(expr.index, "Start of match is not "
                           "currently supported for patterns which match an "
                           "empty buffer.");
    }

    dumpDotWrapper(g, expr, "01_initial", cc.grey);
    assert(allMatchStatesHaveReports(g));

    /* ensure utf8 starts at cp boundary */
    ensureCodePointStart(rm, g, expr);

    if (can_never_match(g)) {
        throw CompileError(expr.index, "Pattern can never match.");
    }

    bool hamming = expr.hamm_distance > 0;
    u32 e_dist = hamming ? expr.hamm_distance : expr.edit_distance;

    DEBUG_PRINTF("edit distance = %u hamming = %s\n", e_dist, hamming ? "true" : "false");

    // validate graph's suitability for fuzzing before resolving asserts
    validate_fuzzy_compile(g, e_dist, hamming, expr.utf8, cc.grey);

    resolveAsserts(rm, g, expr);
    dumpDotWrapper(g, expr, "02_post_assert_resolve", cc.grey);
    assert(allMatchStatesHaveReports(g));

    make_fuzzy(g, e_dist, hamming, cc.grey);
    dumpDotWrapper(g, expr, "02a_post_fuzz", cc.grey);

    pruneUseless(g);
    pruneEmptyVertices(g);

    if (can_never_match(g)) {
        throw CompileError(expr.index, "Pattern can never match.");
    }

    optimiseVirtualStarts(g); /* good for som */

    propagateExtendedParams(g, expr, rm);
    reduceExtendedParams(g, rm, som);

    // We may have removed all the edges to accept, in which case this
    // expression cannot match.
    if (can_never_match(g)) {
        throw CompileError(expr.index, "Extended parameter constraints can not "
                                       "be satisfied for any match from this "
                                       "expression.");
    }

    if (any_of_in(all_reports(g), [&](ReportID id) {
            return rm.getReport(id).minLength;
        })) {
        // We have at least one report with a minimum length constraint, which
        // we currently use SOM to satisfy.
        som = SOM_LEFT;
        ssm.somPrecision(8);
    }

    if (som) {
        rose->setSom();
    }

    // first, we can perform graph work that can be done on an individual
    // expression basis.

    if (expr.utf8) {
        relaxForbiddenUtf8(g, expr);
    }

    if (all_of_in(all_reports(g), [&](ReportID id) {
            const auto &report = rm.getReport(id);
            return report.ekey != INVALID_EKEY && !report.minLength &&
                   !report.minOffset;
        })) {
        // In highlander mode: if we don't have constraints on our reports that
        // may prevent us accepting our first match (i.e. extended params) we
        // can prune the other out-edges of all vertices connected to accept.
        // TODO: shift the report checking down into pruneHighlanderAccepts()
        // to allow us to handle the parts we can in mixed cases.
        pruneHighlanderAccepts(g, rm);
    }

    dumpDotWrapper(g, expr, "02b_fairly_early", cc.grey);

    // If we're a vacuous pattern, we can handle this early.
    if (splitOffVacuous(boundary, rm, g, expr)) {
        DEBUG_PRINTF("split off vacuous\n");
    }

    // We might be done at this point: if we've run out of vertices, we can
    // stop processing.
    if (num_vertices(g) == N_SPECIALS) {
        DEBUG_PRINTF("all vertices claimed by vacuous handling\n");
        return true;
    }

    // Now that vacuous edges have been removed, update the min width exclusive
    // of boundary reports.
    minWidth = min(minWidth, findMinWidth(g));

    // Add the pattern to the small write builder.
    smwr->add(g, expr);

    if (!som) {
        removeSiblingsOfStartDotStar(g);
    }

    dumpDotWrapper(g, expr, "03_early", cc.grey);

    // Perform a reduction pass to merge sibling character classes together.
    if (cc.grey.performGraphSimplification) {
        removeRedundancy(g, som);
        prunePathsRedundantWithSuccessorOfCyclics(g, som);
    }

    dumpDotWrapper(g, expr, "04_reduced", cc.grey);

    // If we've got some literals that span the graph from start to accept, we
    // can split them off into Rose from here.
    if (!som) {
        if (splitOffLiterals(*this, g)) {
            DEBUG_PRINTF("some vertices claimed by literals\n");
        }
    }

    // We might be done at this point: if we've run out of vertices, we can
    // stop processing.
    if (num_vertices(g) == N_SPECIALS) {
        DEBUG_PRINTF("all vertices claimed before calc components\n");
        return true;
    }

    // Split the graph into a set of connected components and process those.
    // Note: this invalidates g_ptr.

    auto g_comp = calcComponents(std::move(g_ptr), cc.grey);
    assert(!g_comp.empty());

    if (!som) {
        for (auto &gc : g_comp) {
            assert(gc);
            reformLeadingDots(*gc);
        }

        recalcComponents(g_comp, cc.grey);
    }

    if (processComponents(*this, expr, g_comp, som)) {
        return true;
    }

    // If we're in prefiltering mode, we can run the prefilter reductions and
    // have another shot at accepting the graph.

    if (cc.grey.prefilterReductions && expr.prefilter) {
        for (auto &gc : g_comp) {
            if (!gc) {
                continue;
            }
            prefilterReductions(*gc, cc);
        }

        if (processComponents(*this, expr, g_comp, som)) {
            return true;
        }
    }

    // We must have components that could not be compiled.
    for (u32 i = 0; i < g_comp.size(); i++) {
        if (g_comp[i]) {
            DEBUG_PRINTF("could not compile component %u with %zu vertices\n",
                         i, num_vertices(*g_comp[i]));
            throw CompileError(expr.index, "Pattern is too large.");
        }
    }

    assert(0); // should have thrown.
    return false;
}

/** \brief Used from SOM mode to add an arbitrary NGHolder as an engine. */
bool NG::addHolder(NGHolder &g) {
    DEBUG_PRINTF("adding holder of %zu states\n", num_vertices(g));
    assert(allMatchStatesHaveReports(g));
    assert(hasCorrectlyNumberedVertices(g));

    /* We don't update the global minWidth here as we care about the min width
     * of the whole pattern - not a just a prefix of it. */

    bool prefilter = false;
    //dumpDotComp(comp, g, *this, 20, "prefix_init");

    som_type som = SOM_NONE; /* the prefixes created by the SOM code do not
                                themselves track som */
    bool utf8 = false; // handling done earlier
    reduceGraph(g, som, utf8, cc);

    // There may be redundant regions that we can remove
    if (cc.grey.performGraphSimplification) {
        removeRegionRedundancy(g, som);
    }

    // "Short Exhaustible Passthrough" patterns always become outfixes.
    if (isSEP(g, rm, cc.grey)) {
        DEBUG_PRINTF("graph is SEP\n");
        if (rose->addOutfix(g)) {
            return true;
        }
    }

    if (splitOffAnchoredAcyclic(*rose, g, cc)) {
        return true;
    }

    if (handleSmallLiteralSets(*rose, g, cc)
        || handleFixedWidth(*rose, g, cc.grey)) {
        return true;
    }

    if (handleDecoratedLiterals(*rose, g, cc)) {
        return true;
    }

    if (doViolet(*rose, g, prefilter, false, rm, cc)) {
        return true;
    }
    if (splitOffPuffs(*rose, rm, g, prefilter, cc)) {
        return true;
    }
    if (doViolet(*rose, g, prefilter, true, rm, cc)) {
        return true;
    }

    DEBUG_PRINTF("trying for outfix\n");
    if (rose->addOutfix(g)) {
        DEBUG_PRINTF("ok\n");
        return true;
    }
    DEBUG_PRINTF("trying for outfix - failed\n");
    DEBUG_PRINTF("nobody would take us\n");
    return false;
}

bool NG::addLiteral(const ue2_literal &literal, u32 expr_index,
                    u32 external_report, bool highlander, som_type som,
                    bool quiet) {
    assert(!literal.empty());

    if (!cc.grey.shortcutLiterals) {
        return false;
    }

    // We can't natively handle arbitrary literals with mixed case sensitivity
    // in Rose -- they require mechanisms like benefits masks, which have
    // length limits etc. Better to let those go through full graph processing.
    if (mixed_sensitivity(literal)) {
        DEBUG_PRINTF("mixed sensitivity\n");
        return false;
    }

    // Register external report and validate highlander constraints.
    rm.registerExtReport(external_report,
                         external_report_info(highlander, expr_index));

    ReportID id;
    if (som) {
        assert(!highlander); // not allowed, checked earlier.
        Report r = makeSomRelativeCallback(external_report, 0, literal.length());
        id = rm.getInternalId(r);
        rose->setSom();
    } else {
        u32 ekey = highlander ? rm.getExhaustibleKey(external_report)
                              : INVALID_EKEY;
        Report r = makeECallback(external_report, 0, ekey, quiet);
        id = rm.getInternalId(r);
    }

    DEBUG_PRINTF("success: graph is literal '%s', report ID %u\n",
                 dumpString(literal).c_str(), id);

    rose->add(false, false, literal, {id});

    minWidth = min(minWidth, depth(literal.length()));

    /* inform small write handler about this literal */
    smwr->add(literal, id);

    return true;
}

} // namespace ue2
