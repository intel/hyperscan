/*
 * Copyright (c) 2015-2017, Intel Corporation
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

#include "rdfa_merge.h"

#include "grey.h"
#include "dfa_min.h"
#include "mcclellancompile_util.h"
#include "rdfa.h"
#include "ue2common.h"
#include "nfagraph/ng_mcclellan_internal.h"
#include "util/container.h"
#include "util/determinise.h"
#include "util/flat_containers.h"
#include "util/make_unique.h"
#include "util/report_manager.h"
#include "util/unordered.h"

#include <algorithm>
#include <queue>

using namespace std;

namespace ue2 {

#define MAX_DFA_STATES 16383

namespace {

class Automaton_Merge {
public:
    using StateSet = vector<u16>;
    using StateMap = ue2_unordered_map<StateSet, dstate_id_t>;

    Automaton_Merge(const raw_dfa *rdfa1, const raw_dfa *rdfa2,
                    const ReportManager *rm_in, const Grey &grey_in)
        : rm(rm_in), grey(grey_in), nfas{rdfa1, rdfa2}, dead(2) {
        calculateAlphabet();
        populateAsFs();
        prunable = isPrunable();
    }

    Automaton_Merge(const vector<const raw_dfa *> &dfas,
                    const ReportManager *rm_in, const Grey &grey_in)
        : rm(rm_in), grey(grey_in), nfas(dfas), dead(nfas.size()) {
        calculateAlphabet();
        populateAsFs();
        prunable = isPrunable();
    }

    void populateAsFs(void) {
        bool fs_same = true;
        bool fs_dead = true;

        as.resize(nfas.size());
        fs.resize(nfas.size());
        for (size_t i = 0, end = nfas.size(); i < end; i++) {
            as[i] = nfas[i]->start_anchored;
            fs[i] = nfas[i]->start_floating;

            if (fs[i]) {
                fs_dead = false;
            }

            if (as[i] != fs[i]) {
                fs_same = false;
            }
        }

        start_anchored = DEAD_STATE + 1;
        if (fs_same) {
            start_floating = start_anchored;
        } else if (fs_dead) {
            start_floating = DEAD_STATE;
        } else {
            start_floating = start_anchored + 1;
        }
    }

    void calculateAlphabet(void) {
        DEBUG_PRINTF("calculating alphabet\n");
        vector<CharReach> esets = {CharReach::dot()};

        for (const auto &rdfa : nfas) {
            DEBUG_PRINTF("...next dfa alphabet\n");
            assert(rdfa);
            const auto &alpha_remap = rdfa->alpha_remap;

            for (size_t i = 0; i < esets.size(); i++) {
                assert(esets[i].count());
                if (esets[i].count() == 1) {
                    DEBUG_PRINTF("skipping singleton eq set\n");
                    continue;
                }

                CharReach t;
                u8 leader_s = alpha_remap[esets[i].find_first()];

                DEBUG_PRINTF("checking eq set, leader %02hhx \n", leader_s);

                for (size_t s = esets[i].find_first(); s != CharReach::npos;
                     s = esets[i].find_next(s)) {
                    if (alpha_remap[s] != leader_s) {
                        t.set(s);
                    }
                }

                if (t.any() && t != esets[i]) {
                    esets[i] &= ~t;
                    esets.push_back(t);
                }
            }
        }

        // Sort so that our alphabet mapping isn't dependent on the order of
        // rdfas passed in.
        sort(esets.begin(), esets.end());

        alphasize = buildAlphabetFromEquivSets(esets, alpha, unalpha);
    }

    bool isPrunable() const {
        if (!grey.highlanderPruneDFA || !rm) {
            DEBUG_PRINTF("disabled, or not managed reports\n");
            return false;
        }

        assert(!nfas.empty());
        if (!generates_callbacks(nfas.front()->kind)) {
            DEBUG_PRINTF("doesn't generate callbacks\n");
            return false;
        }

        // Collect all reports from all merge candidates.
        flat_set<ReportID> merge_reports;
        for (const auto &rdfa : nfas) {
            insert(&merge_reports, all_reports(*rdfa));
        }

        DEBUG_PRINTF("all reports: %s\n", as_string_list(merge_reports).c_str());

        // Return true if they're all exhaustible with the same exhaustion key.
        u32 ekey = INVALID_EKEY;
        for (const auto &report_id : merge_reports) {
            const Report &r = rm->getReport(report_id);
            if (!isSimpleExhaustible(r)) {
                DEBUG_PRINTF("report %u not simple exhaustible\n", report_id);
                return false;
            }
            assert(r.ekey != INVALID_EKEY);
            if (ekey == INVALID_EKEY) {
                ekey = r.ekey;
            } else if (ekey != r.ekey) {
                DEBUG_PRINTF("two different ekeys, %u and %u\n", ekey, r.ekey);
                return false;
            }
        }

        DEBUG_PRINTF("is prunable\n");
        return true;
    }


    void transition(const StateSet &in, StateSet *next) {
        u16 t[ALPHABET_SIZE];

        for (u32 i = 0; i < alphasize; i++) {
            next[i].resize(nfas.size());
        }

        for (size_t j = 0, j_end = nfas.size(); j < j_end; j++) {
            getFullTransitionFromState(*nfas[j], in[j], t);
            for (u32 i = 0; i < alphasize; i++) {
                next[i][j] = t[unalpha[i]];
            }
        }
    }

    const vector<StateSet> initial() {
        vector<StateSet> rv = {as};
        if (start_floating != DEAD_STATE && start_floating != start_anchored) {
            rv.push_back(fs);
        }
        return rv;
    }

private:
    void reports_i(const StateSet &in, flat_set<ReportID> dstate::*r_set,
                   flat_set<ReportID> &r) const {
        for (size_t i = 0, end = nfas.size(); i < end; i++) {
            const auto &rs = nfas[i]->states[in[i]].*r_set;
            insert(&r, rs);
        }
    }

public:
    void reports(const StateSet &in, flat_set<ReportID> &rv) const {
        reports_i(in, &dstate::reports, rv);
    }
    void reportsEod(const StateSet &in, flat_set<ReportID> &rv) const {
        reports_i(in, &dstate::reports_eod, rv);
    }

    bool canPrune(const flat_set<ReportID> &test_reports) const {
        if (!grey.highlanderPruneDFA || !prunable) {
            return false;
        }

        // Must all be external reports.
        assert(rm);
        for (const auto &report_id : test_reports) {
            if (!isExternalReport(rm->getReport(report_id))) {
                return false;
            }
        }

        return true;
    }

    /** True if the minimization algorithm should be run after merging. */
    bool shouldMinimize() const {
        // We only need to run minimization if our merged DFAs shared a report.
        flat_set<ReportID> seen_reports;
        for (const auto &rdfa : nfas) {
            for (const auto &report_id : all_reports(*rdfa)) {
                if (!seen_reports.insert(report_id).second) {
                    DEBUG_PRINTF("report %u in several dfas\n", report_id);
                    return true;
                }
            }
        }

        return false;
    }

private:
    const ReportManager *rm;
    const Grey &grey;

    vector<const raw_dfa *> nfas;
    vector<dstate_id_t> as;
    vector<dstate_id_t> fs;

    bool prunable = false;

public:
    std::array<u16, ALPHABET_SIZE> alpha;
    std::array<u16, ALPHABET_SIZE> unalpha;
    u16 alphasize;
    StateSet dead;

    u16 start_anchored;
    u16 start_floating;
};

} // namespace

unique_ptr<raw_dfa> mergeTwoDfas(const raw_dfa *d1, const raw_dfa *d2,
                                 size_t max_states, const ReportManager *rm,
                                 const Grey &grey) {
    assert(d1 && d2);
    assert(d1->kind == d2->kind);
    assert(max_states <= MAX_DFA_STATES);

    auto rdfa = ue2::make_unique<raw_dfa>(d1->kind);

    Automaton_Merge autom(d1, d2, rm, grey);
    if (determinise(autom, rdfa->states, max_states)) {
        rdfa->start_anchored = autom.start_anchored;
        rdfa->start_floating = autom.start_floating;
        rdfa->alpha_size = autom.alphasize;
        rdfa->alpha_remap = autom.alpha;
        DEBUG_PRINTF("merge succeeded, %zu states\n", rdfa->states.size());

        if (autom.shouldMinimize()) {
            minimize_hopcroft(*rdfa, grey);
            DEBUG_PRINTF("minimized, %zu states\n", rdfa->states.size());
        }

        return rdfa;
    }

    return nullptr;
}

void mergeDfas(vector<unique_ptr<raw_dfa>> &dfas, size_t max_states,
               const ReportManager *rm, const Grey &grey) {
    assert(max_states <= MAX_DFA_STATES);

    if (dfas.size() <= 1) {
        return;
    }

    DEBUG_PRINTF("before merging, we have %zu dfas\n", dfas.size());

    queue<unique_ptr<raw_dfa>> q;
    for (auto &dfa : dfas) {
        q.push(move(dfa));
    }

    // All DFAs are now on the queue, so we'll clear the vector and use it for
    // output from here.
    dfas.clear();

    while (q.size() > 1) {
        // Attempt to merge the two front elements of the queue.
        unique_ptr<raw_dfa> d1 = move(q.front());
        q.pop();
        unique_ptr<raw_dfa> d2 = move(q.front());
        q.pop();

        auto rdfa = mergeTwoDfas(d1.get(), d2.get(), max_states, rm, grey);
        if (rdfa) {
            q.push(move(rdfa));
        } else {
            DEBUG_PRINTF("failed to merge\n");
            // Put the larger of the two DFAs on the output list, retain the
            // smaller one on the queue for further merge attempts.
            if (d2->states.size() > d1->states.size()) {
                dfas.push_back(move(d2));
                q.push(move(d1));
            } else {
                dfas.push_back(move(d1));
                q.push(move(d2));
            }
        }
    }

    while (!q.empty()) {
        dfas.push_back(move(q.front()));
        q.pop();
    }

    DEBUG_PRINTF("after merging, we have %zu dfas\n", dfas.size());
}

unique_ptr<raw_dfa> mergeAllDfas(const vector<const raw_dfa *> &dfas,
                                 size_t max_states, const ReportManager *rm,
                                 const Grey &grey) {
    assert(max_states <= MAX_DFA_STATES);
    assert(!dfas.empty());

    // All the DFAs should be of the same kind.
    const auto kind = dfas.front()->kind;
    assert(all_of(begin(dfas), end(dfas),
                  [&kind](const raw_dfa *rdfa) { return rdfa->kind == kind; }));

    auto rdfa = ue2::make_unique<raw_dfa>(kind);
    Automaton_Merge n(dfas, rm, grey);

    DEBUG_PRINTF("merging dfa\n");

    if (!determinise(n, rdfa->states, max_states)) {
        DEBUG_PRINTF("state limit (%zu) exceeded\n", max_states);
        return nullptr; /* over state limit */
    }

    rdfa->start_anchored = n.start_anchored;
    rdfa->start_floating = n.start_floating;
    rdfa->alpha_size = n.alphasize;
    rdfa->alpha_remap = n.alpha;

    DEBUG_PRINTF("merged, building impl dfa (a,f) = (%hu,%hu)\n",
                 rdfa->start_anchored, rdfa->start_floating);

    if (n.shouldMinimize()) {
        minimize_hopcroft(*rdfa, grey);
        DEBUG_PRINTF("minimized, %zu states\n", rdfa->states.size());
    }

    return rdfa;
}

} // namespace ue2
