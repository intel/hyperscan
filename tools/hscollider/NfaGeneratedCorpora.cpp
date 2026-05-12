/*
 * Copyright (C) 2026 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they were
 * provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit this
 * software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
 */

#include "config.h"

#include "ng_corpus_properties.h"
#include "ng_corpus_generator.h"
#include "NfaGeneratedCorpora.h"
#include "ExpressionParser.h"
#include "common.h"

#include "grey.h"
#include "hs_compile.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "parser/parse_error.h"
#include "parser/Parser.h"
#include "parser/prefilter.h"
#include "parser/unsupported.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/report_manager.h"
#include "util/string_util.h"
#include "util/target_info.h"

#include <string>
#include <sstream>
#include <vector>

using namespace std;
using namespace ue2;

NfaGeneratedCorpora::NfaGeneratedCorpora(const ExpressionMap &expr,
                                         const CorpusProperties &props,
                                         bool force_utf8_mode_in,
                                         bool force_prefilter_mode_in)
    : m_expr(expr), m_props(props), force_utf8_mode(force_utf8_mode_in),
      force_prefilter_mode(force_prefilter_mode_in) {
    // empty
}

NfaGeneratedCorpora *NfaGeneratedCorpora::clone() const {
    return new NfaGeneratedCorpora(m_expr, m_props, force_utf8_mode,
                                   force_prefilter_mode);
}

void NfaGeneratedCorpora::generate(unsigned id, vector<Corpus> &data) {
    ExpressionMap::const_iterator i = m_expr.find(id);
    if (i == m_expr.end()) {
        throw CorpusFailure("Expression not found.");
    }

    string re;
    u32 hs_flags;
    hs_expr_ext ext;
    if (!readExpression(i->second, re, &hs_flags, &ext)) {
        throw CorpusFailure("Expression could not be read: " + i->second);
    }

    // When PTL literal API is on, transfer the regex string into hex.
    if (use_literal_api && !(hs_flags & HS_FLAG_COMBINATION)) {
        unsigned char *pat
            = reinterpret_cast<unsigned char *>(const_cast<char *>(re.c_str()));
        char *str = makeHex(pat, re.length());
        if (!str) {
            throw CorpusFailure("makeHex() malloc failure.");
        }
        re.assign(str);
        free(str);
    }

    // Combination's corpus is consist of sub-expressions' corpuses.
    if (hs_flags & HS_FLAG_COMBINATION) {
        ParsedLogical pl;
        pl.parseLogicalCombination(id, re.c_str(), ~0U, 0, ~0ULL);
        pl.logicalKeyRenumber();
        const auto &m_lkey = pl.getLkeyMap();
        assert(!m_lkey.empty());
        u32 a_subid = 0; // arbitrary sub id
        unordered_map<u32, vector<Corpus>> m_data;
        for (const auto &it : m_lkey) {
            a_subid = it.first;
            vector<Corpus> sub_data;
            generate(a_subid, sub_data);
            m_data.emplace(a_subid, std::move(sub_data));
        }
        assert(!m_data.empty());
        size_t num_corpus = m_data[a_subid].size();
        data.reserve(data.size() + num_corpus);
        while (num_corpus) {
            string cc; // 1 combination corpus
            for (const auto &it : m_lkey) {
                assert(!m_data[it.first].empty());
                cc += m_data[it.first].back().data;
                if (m_data[it.first].size() > 1) {
                    m_data[it.first].pop_back();
                }
            }
            data.push_back(Corpus(cc));
            num_corpus--;
        }
        return;
    }

    if (force_utf8_mode) {
        hs_flags |= HS_FLAG_UTF8;
    }

    if (force_prefilter_mode) {
        hs_flags |= HS_FLAG_PREFILTER;
    }

    // Wrap the UE2 parser and compiler functionality and use it to generate
    // corpora for us.
    vector<string> c;

    try {
        ParsedExpression pe(0, re.c_str(), hs_flags, 0, &ext);

        // Apply prefiltering transformations if desired.
        if (pe.expr.prefilter) {
            prefilterTree(pe.component, ParseMode(hs_flags));
        }

        // Bail on patterns with unsupported constructs.
        checkUnsupported(*pe.component);
        pe.component->checkEmbeddedStartAnchor(true);
        pe.component->checkEmbeddedEndAnchor(true);

        CompileContext cc(false, false, get_current_target(), Grey());
        ReportManager rm(cc.grey);
        auto built_expr = buildGraph(rm, cc, pe);
        if (!built_expr.g) {
            // A more specific error should probably have been thrown by
            // buildGraph.
            throw CorpusFailure("could not build graph.");
        }

        const auto cg =
            makeCorpusGenerator(*built_expr.g, built_expr.expr, m_props);
        cg->generateCorpus(c);
    }
    catch (const ParseError &e) {
        throw CorpusFailure("compilation failed, " + e.reason);
    }
    catch (const CompileError &e) {
        throw CorpusFailure("compilation failed, " + e.reason);
    }
    catch (const std::bad_alloc &) {
        throw CorpusFailure("out of memory.");
    }
    catch (const CorpusGenerationFailure &e) {
        // if corpus generation failed, just pass up the error message
        throw CorpusFailure("corpus generation failed: " + e.message);
    }
    catch (...) {
        throw CorpusFailure("unknown error.");
    }

    if (c.empty()) {
        throw CorpusFailure("no corpora generated.");
    }

    data.reserve(data.size() + c.size());
    for (const auto &e : c) {
        data.push_back(Corpus(e));
    }
}
