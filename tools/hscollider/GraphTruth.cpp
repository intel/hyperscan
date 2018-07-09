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

#include "config.h"

#include "GraphTruth.h"

#include "common.h"
#include "expressions.h"
#include "ExpressionParser.h"
#include "ng_find_matches.h"
#include "pcre_util.h"

#include "grey.h"
#include "hs_compile.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "nfagraph/ng_depth.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_fuzzy.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_util.h"
#include "parser/Parser.h"
#include "parser/unsupported.h"
#include "parser/logical_combination.h"
#include "util/compile_context.h"
#include "util/make_unique.h"
#include "util/report_manager.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <ostream>
#include <string>
#include <vector>

using namespace std;
using namespace ue2;

// Struct to store the actual compiled NFA graph.
class CompiledNG : boost::noncopyable {
public:
    CompiledNG(unique_ptr<NGHolder> g_in,
               unique_ptr<ReportManager> rm_in)
        : g(std::move(g_in)), rm(std::move(rm_in)) {}
    CompiledNG(unique_ptr<ParsedLogical> pl_in)
        : pl(std::move(pl_in)) {}
    unique_ptr<ue2::NGHolder> g;
    unique_ptr<ue2::ReportManager> rm;
    unique_ptr<ue2::ParsedLogical> pl;
};

static
void populateMatchSet(ResultSet &rs, const set<pair<size_t, size_t>> &matches,
                      const CNGInfo &cngi) {
    for (const auto &m : matches) {
        u64a from = m.first;
        u64a to = m.second;
        if (g_streamOffset) {
            // Subtract stream offset imposed by offset test.
            u64a offset = min(100ull, g_streamOffset);
            assert(to >= offset);
            from -= min(offset, from);
            to -= offset;
        }
        u64a len = to - from;

        if (to < cngi.min_offset || to > cngi.max_offset ||
            len < cngi.min_length) {
            // this match does not satisfy extparams constraints
            DEBUG_PRINTF("skipping NFA Match @ (%llu,%llu)\n", from, to);
            continue;
        }
        if (!cngi.som) {
            from = 0;
        }
        rs.addMatch(from, to);
    }
}

CNGInfo::CNGInfo(unsigned id_in, const ExpressionMap &m_expr_in)
    : id(id_in), m_expr(m_expr_in) {}

CNGInfo::~CNGInfo() = default;

void CNGInfo::compile() {
    auto i = m_expr.find(id);
    if (i == m_expr.end()) {
        throw NGCompileFailure("ID not found in expression map.");
    }

    string re;
    unsigned hs_flags;
    hs_expr_ext ext;

    // read the flags for NFA compiler
    if (!readExpression(i->second, re, &hs_flags, &ext)) {
        throw NGCompileFailure("Cannot parse expression flags.");
    }
    // make sure we respect collider's UTF-8 setting
    if (force_utf8) {
        hs_flags |= HS_FLAG_UTF8;
    }

    try {
        if (combination) {
            auto pl = ue2::make_unique<ParsedLogical>();
            pl->parseLogicalCombination(id, re.c_str(), ~0U, 0, ~0ULL);
            pl->logicalKeyRenumber();
            cng = make_unique<CompiledNG>(move(pl));
            return;
        }

        bool isStreaming = colliderMode == MODE_STREAMING;
        bool isVectored = colliderMode == MODE_VECTORED;
        CompileContext cc(isStreaming, isVectored, get_current_target(),
                          Grey());
        ParsedExpression pe(0, re.c_str(), hs_flags, 0, &ext);

        // UE-2850: ParsedExpression may have updated the utf8 flag if the
        // original expression starts with (*UTF8)
        utf8 |= pe.expr.utf8;

        auto rm = ue2::make_unique<ReportManager>(cc.grey);

        // Expressions containing zero-width assertions and other extended pcre
        // types aren't supported yet. This call will throw a ParseError
        // exception if the component tree contains such a construct.
        checkUnsupported(*pe.component);

        pe.component->checkEmbeddedStartAnchor(true);
        pe.component->checkEmbeddedEndAnchor(true);

        // edit distance may be set globally
        if (force_edit_distance) {
            pe.expr.edit_distance = edit_distance;
        }

        // validate_fuzzy_compile checks this, but we don't need to build the
        // graph to know it will fail
        if (pe.expr.edit_distance && utf8) {
            throw NGCompileFailure("UTF-8 patterns cannot be "
                                   "approximately matched");
        }

        auto built_expr = buildGraph(*rm, cc, pe);
        auto &expr = built_expr.expr;
        auto &g = built_expr.g;

        if (expr.edit_distance || expr.hamm_distance) {
            // check if this pattern can be approximately matched, throws
            // CompileError on failure
            bool hamming = expr.hamm_distance > 0;
            u32 e_dist = hamming ? expr.hamm_distance : expr.edit_distance;
            validate_fuzzy_compile(*g, e_dist, hamming, utf8, cc.grey);
        }

        if (isVacuous(*g)) {
            if (som) {
                throw NGUnsupportedFailure("Vacuous patterns are not supported "
                                           "in SOM mode");
            }
            if (expr.min_length > 0) {
                throw NGUnsupportedFailure("Vacuous patterns are not supported "
                                           "in combination with min_length");
            }
        }

        cng = make_unique<CompiledNG>(move(g), move(rm));
    } catch (CompileError &e) {
        throw NGCompileFailure(e.reason);
    } catch (NGUnsupportedFailure &e) {
        throw NGCompileFailure(e.msg);
    } catch (...) {
        throw NGCompileFailure("NFA graph construction failed");
    }
}

GraphTruth::GraphTruth(ostream &os, const ExpressionMap &expr)
    : out(os), m_expr(expr) {}

unique_ptr<CNGInfo> GraphTruth::preprocess(unsigned id,
                                           bool ignoreUnsupported) {
    bool highlander = false;
    bool prefilter = false;
    bool som = false;
    bool combination = false;
    bool quiet = false;

    auto i = m_expr.find(id);
    if (i == m_expr.end()) {
        throw NGCompileFailure("ID not found in expression map.");
    }

    string re;
    unsigned flags, hs_flags;
    hs_expr_ext ext;

    // read the flags for NFA compiler
    if (!readExpression(i->second, re, &hs_flags, &ext)) {
        throw NGCompileFailure("Cannot parse expression flags.");
    }
    // read PCRE flags
    if (!getPcreFlags(hs_flags, &flags, &highlander, &prefilter, &som,
                      &combination, &quiet)) {
        throw NGCompileFailure("Cannot get PCRE flags.");
    }
    if (force_utf8) {
        hs_flags |= HS_FLAG_UTF8;
    }

    // edit distance might be set globally
    if (force_edit_distance) {
        ext.edit_distance = edit_distance;
    }

    // SOM flags might be set globally.
    som |= !!somFlags;

    if (force_prefilter) {
        prefilter = true;
    }

    u64a supported_flags = HS_EXT_FLAG_HAMMING_DISTANCE |
                           HS_EXT_FLAG_EDIT_DISTANCE | HS_EXT_FLAG_MIN_OFFSET |
                           HS_EXT_FLAG_MAX_OFFSET | HS_EXT_FLAG_MIN_LENGTH;
    if (ext.flags & ~supported_flags) {
        if (!ignoreUnsupported) {
            throw NGUnsupportedFailure("Unsupported extended flags specified.");
        }
    }

    auto cngi = make_unique<CNGInfo>(id, m_expr);
    cngi->utf8 = hs_flags & HS_FLAG_UTF8;
    cngi->highlander = highlander;
    cngi->prefilter = prefilter;
    cngi->som = som;
    cngi->combination = combination;
    cngi->quiet = quiet;
    cngi->min_offset = ext.min_offset;
    cngi->max_offset = ext.max_offset;
    cngi->min_length = ext.min_length;
    cngi->max_edit_distance = ext.edit_distance;
    cngi->max_hamm_distance = ext.hamming_distance;

    return cngi;
}

/** \brief Returns 1 if compliant to all logical combinations. */
static
char isLogicalCombination(vector<char> &lv, const vector<LogicalOp> &comb,
                          size_t lkeyCount, unsigned start, unsigned result) {
    assert(start <= result);
    for (unsigned i = start; i <= result; i++) {
        const LogicalOp &op = comb[i - lkeyCount];
        assert(i == op.id);
        switch (op.op) {
        case LOGICAL_OP_NOT:
            lv[op.id] = !lv[op.ro];
            break;
        case LOGICAL_OP_AND:
            lv[op.id] = lv[op.lo] & lv[op.ro]; // &&
            break;
        case LOGICAL_OP_OR:
            lv[op.id] = lv[op.lo] | lv[op.ro]; // ||
            break;
        default:
            assert(0);
            break;
        }
    }
    return lv[result];
}

bool GraphTruth::run(unsigned, const CompiledNG &cng, const CNGInfo &cngi,
                     const string &buffer, ResultSet &rs, string &error) {
    if (cngi.quiet) {
        return true;
    }

    if (cngi.combination) {
        // Compile and run sub-expressions, store match results.
        map<unsigned long long, set<MatchResult>> offset_to_matches;
        map<unsigned long long, set<unsigned>> offset_to_lkeys;
        set<unsigned> sub_exps;
        const auto &m_lkey = cng.pl->getLkeyMap();
        for (const auto &it_lkey : m_lkey) {
            if (sub_exps.find(it_lkey.first) == sub_exps.end()) {
                sub_exps.emplace(it_lkey.first);
                ResultSet sub_rs(RESULT_FROM_PCRE);
                shared_ptr<CNGInfo> sub_cngi = preprocess(it_lkey.first);
                const CompiledNG *sub_cng;
                try {
                    sub_cng = sub_cngi->get();
                }
                catch (const NGCompileFailure &err) {
                    return false;
                }
                catch (const NGUnsupportedFailure &err) {
                    return false;
                }
                sub_cngi->quiet = false; // force not quiet in sub-exp.
                if (!run(it_lkey.first, *sub_cng, *sub_cngi, buffer, sub_rs, error)) {
                    rs.clear();
                    return false;
                }
                for (const auto &it_mr : sub_rs.matches) {
                    offset_to_matches[it_mr.to].emplace(it_mr);
                    offset_to_lkeys[it_mr.to].emplace(it_lkey.second);
                    if (sub_cngi->highlander) {
                        break;
                    }
                }
            }
        }
        // Calculate rs for combination expression.
        vector<char> lv;
        const auto &comb = cng.pl->getLogicalTree();
        lv.resize(m_lkey.size() + comb.size());
        const auto &li = cng.pl->getCombInfoById(cngi.id);
        for (const auto &it : offset_to_lkeys) {
            for (auto report : it.second) {
                lv[report] = 1;
            }
            if (isLogicalCombination(lv, comb, m_lkey.size(),
                                     li.start, li.result)) {
                for (const auto &mr : offset_to_matches.at(it.first)) {
                    if ((mr.to >= cngi.min_offset) &&
                        (mr.to <= cngi.max_offset)) {
                        rs.addMatch(mr.from, mr.to);
                    }
                }
            }
        }
        return true;
    }

    set<pair<size_t, size_t>> matches;

    if (g_streamOffset) {
        size_t offset = MIN(100, g_streamOffset);
        assert(offset > 0);
        const string preamble(string(offset, '\0'));

        set<pair<size_t, size_t>> pre_matches;

        // First, scan an empty buffer size of the preamble so that we can
        // discard any matches therein after the real scan, later. We use
        // notEod so that end-anchors in our expression don't match at the
        // end of the buffer.
        if (!findMatches(*cng.g, *cng.rm, preamble, pre_matches,
                         cngi.max_edit_distance, cngi.max_hamm_distance, true,
                         cngi.utf8)) {
            return false;
        }

        // Real scan.
        if (!findMatches(*cng.g, *cng.rm, preamble + buffer, matches,
                         cngi.max_edit_distance, cngi.max_hamm_distance, false,
                         cngi.utf8)) {
            return false;
        }

        // Erase any matches due entirely to the preamble.
        for (const auto &m : pre_matches) {
            matches.erase(m);
        }
    } else {
        if (!findMatches(*cng.g, *cng.rm, buffer, matches,
                         cngi.max_edit_distance, cngi.max_hamm_distance, false,
                         cngi.utf8)) {
            return false;
        }
    }

    populateMatchSet(rs, matches, cngi);

    if (echo_matches) {
        for (const auto &m : rs.matches) {
            out << "NFA Match @ (" << m.from << "," << m.to << ")" << endl;
        }
    }

    return true;
}
