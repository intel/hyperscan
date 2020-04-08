/*
 * Copyright (c) 2015-2019, Intel Corporation
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

#ifdef _WIN32
#define PCRE_STATIC
#endif
#include "config.h"

#include "common.h"
#include "ExpressionParser.h"
#include "expressions.h"
#include "GroundTruth.h"
#include "pcre_util.h"

#include "hs_compile.h" // for hs_expr_ext
#include "ue2common.h"
#include "parser/control_verbs.h"
#include "parser/Parser.h"
#include "parser/parse_error.h"
#include "util/make_unique.h"
#include "util/string_util.h"
#include "util/unicode_def.h"
#include "util/unordered.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcre.h>

/* -X, -Y support
 * as PCRE performance is `non-linear' and these options add a large amount of
 * scanning, the following short cuts are used:
 * 1: the suffix is not scanned - we are more interested in the matches from
 *    the original corpora.
 * 2: only the last 50 bytes of the prefix is scanned. This may lead to some
 *    minor correctness issues for a few patterns.
 */

using namespace std;
using namespace ue2;

// We store matches in a hash table as we're likely to see lots of them. These
// are moved into a ResultSet at the end.
using PcreMatchSet = ue2::ue2_unordered_set<pair<unsigned, unsigned>>;

namespace {
struct CalloutContext {
    explicit CalloutContext(ostream &os) : out(os) {}
    ostream &out;
    PcreMatchSet matches;
};
}

static
int pcreCallOut(pcre_callout_block *block) {
    assert(block);
    assert(block->callout_data);
    CalloutContext *ctx = static_cast<CalloutContext *>(block->callout_data);

    if (echo_matches) {
        ctx->out << "PCRE Match @ (" << block->start_match << ","
                 << block->current_position << ")" << endl;
    }

    unsigned int from = block->start_match;
    unsigned int to = block->current_position;
    assert(from <= to);

    ctx->matches.insert(make_pair(from, to));
    return 1;
}

static
bool decodeExprPcre(string &expr, unsigned *flags, bool *highlander,
                    bool *prefilter, bool *som, bool *combination,
                    bool *quiet, hs_expr_ext *ext) {
    string regex;
    unsigned int hs_flags = 0;
    if (!readExpression(expr, regex, &hs_flags, ext)) {
        return false;
    }

    if (use_literal_api) {
        // filter out flags not supported by pure literal API.
        u32 not_supported = HS_FLAG_DOTALL | HS_FLAG_ALLOWEMPTY | HS_FLAG_UTF8 |
                             HS_FLAG_UCP | HS_FLAG_PREFILTER;
        hs_flags &= ~not_supported;
        force_utf8 = false;
        force_prefilter = false;
    }

    expr.swap(regex);

    if (!getPcreFlags(hs_flags, flags, highlander, prefilter, som,
                      combination, quiet)) {
        return false;
    }

    if (force_utf8) {
        *flags |= PCRE_UTF8;
    }

    if (force_prefilter) {
        *prefilter = true;
    }

    return true;
}

static
string pcreErrStr(int err) {
    switch (err) {
        case PCRE_ERROR_NOMATCH:
            return "PCRE_ERROR_NOMATCH";
        case PCRE_ERROR_NULL:
            return "PCRE_ERROR_NULL";
        case PCRE_ERROR_BADOPTION:
            return "PCRE_ERROR_BADOPTION";
        case PCRE_ERROR_BADMAGIC:
            return "PCRE_ERROR_BADMAGIC";
#if defined(PCRE_ERROR_UNKNOWN_OPCODE)
        case PCRE_ERROR_UNKNOWN_OPCODE:
            return "PCRE_ERROR_UNKNOWN_OPCODE";
#else
         case PCRE_ERROR_UNKNOWN_NODE:
             return "PCRE_ERROR_UNKNOWN_NODE";
#endif
        case PCRE_ERROR_NOMEMORY:
            return "PCRE_ERROR_NOMEMORY";
        case PCRE_ERROR_NOSUBSTRING:
            return "PCRE_ERROR_NOSUBSTRING";
        case PCRE_ERROR_MATCHLIMIT:
            return "PCRE_ERROR_MATCHLIMIT";
        case PCRE_ERROR_CALLOUT:
            return "PCRE_ERROR_CALLOUT";
        case PCRE_ERROR_BADUTF8:
            return "PCRE_ERROR_BADUTF8";
        case PCRE_ERROR_BADUTF8_OFFSET:
            return "PCRE_ERROR_BADUTF8_OFFSET";
        case PCRE_ERROR_PARTIAL:
            return "PCRE_ERROR_PARTIAL";
        case PCRE_ERROR_BADPARTIAL:
            return "PCRE_ERROR_BADPARTIAL";
        case PCRE_ERROR_INTERNAL:
            return "PCRE_ERROR_INTERNAL";
        case PCRE_ERROR_BADCOUNT:
            return "PCRE_ERROR_BADCOUNT";
#if defined(PCRE_ERROR_RECURSIONLIMIT)
        case PCRE_ERROR_RECURSIONLIMIT:
            return "PCRE_ERROR_RECURSIONLIMIT";
#endif
        case PCRE_ERROR_DFA_UITEM:
            return "PCRE_ERROR_DFA_UITEM";
        case PCRE_ERROR_DFA_UCOND:
            return "PCRE_ERROR_DFA_UCOND";
        case PCRE_ERROR_DFA_UMLIMIT:
            return "PCRE_ERROR_DFA_UMLIMIT";
        case PCRE_ERROR_DFA_WSSIZE:
            return "PCRE_ERROR_DFA_WSSIZE";
        case PCRE_ERROR_DFA_RECURSE:
            return "PCRE_ERROR_DFA_RECURSE";
        default:
            {
                ostringstream oss;
                oss << "Unknown PCRE error (value: " << err << ")";
                return oss.str();
            }
    }
}

/* that is, a mode provided by native hyperscan */
static
bool isStandardMode(unsigned int mode) {
    return mode == MODE_BLOCK
        || mode == MODE_STREAMING
        || mode == MODE_VECTORED;
}

GroundTruth::GroundTruth(ostream &os, const ExpressionMap &expr,
                         unsigned long int limit,
                         unsigned long int limit_recursion)
    : out(os), m_expr(expr), matchLimit(limit),
      matchLimitRecursion(limit_recursion) {}

void GroundTruth::global_prep() {
    if (isStandardMode(colliderMode)) {
        // We're using pcre callouts
        pcre_callout = &pcreCallOut;
    }
}

static
void addCallout(string &re) {
    // If the string begins with "(*UTF8)" or "(*UTF8)(*UCP)", we want to keep
    // it at the front. We reuse the control verbs mini-parser for this.
    size_t startpos = 0;
    try {
        ue2::ParseMode mode;
        const char *ptr = ue2::read_control_verbs(
            re.c_str(), re.c_str() + re.size(), 0, mode);
        startpos = ptr - re.c_str();
    } catch (const ue2::ParseError &err) {
        // fall through
    }
    assert(startpos <= re.length());
    re.insert(startpos, "(?:");
    // We include a \E to close any open \Q quoted block. If there isn't
    // one, pcre will ignore the \E.
    re.append("\\E)(?C)");
}

static
bool isUtf8(const CompiledPcre &compiled) {
    unsigned long int options = 0;
    pcre_fullinfo(compiled.bytecode, NULL, PCRE_INFO_OPTIONS, &options);
    return options & PCRE_UTF8;
}

unique_ptr<CompiledPcre>
GroundTruth::compile(unsigned id, bool no_callouts) {
    bool highlander = false;
    bool prefilter = false;
    bool som = false;
    bool combination = false;
    bool quiet = false;

    // we can still match approximate matching patterns with PCRE if edit
    // distance 0 is requested
    if (force_edit_distance && edit_distance) {
        throw SoftPcreCompileFailure("Edit distance not supported by PCRE.");
    }

    ExpressionMap::const_iterator i = m_expr.find(id);
    if (i == m_expr.end()) {
        throw PcreCompileFailure("ID not found in expression map.");
    }

    string re(i->second);
    unsigned flags;
    hs_expr_ext ext;

    // Decode the flags
    if (!decodeExprPcre(re, &flags, &highlander, &prefilter, &som,
                        &combination, &quiet, &ext)) {
        throw PcreCompileFailure("Unable to decode flags.");
    }

    // When hyperscan literal api is on, transfer the regex string into hex.
    if (use_literal_api && !combination) {
        unsigned char *pat
            = reinterpret_cast<unsigned char *>(const_cast<char *>(re.c_str()));
        char *str = makeHex(pat, re.length());
        if (!str) {
            throw PcreCompileFailure("makeHex() malloc failure.");
        }
        re.assign(str);
        free(str);
    }

    // filter out flags not supported by PCRE
    u64a supported = HS_EXT_FLAG_MIN_OFFSET | HS_EXT_FLAG_MAX_OFFSET |
                     HS_EXT_FLAG_MIN_LENGTH;
    if (use_literal_api) {
        ext.flags &= 0ULL;
        ext.min_offset = 0;
        ext.max_offset = MAX_OFFSET;
        ext.min_length = 0;
        ext.edit_distance = 0;
        ext.hamming_distance = 0;
    }
    if (ext.flags & ~supported) {
        // edit distance is a known unsupported flag, so just throw a soft error
        if (ext.flags & HS_EXT_FLAG_EDIT_DISTANCE) {
            throw SoftPcreCompileFailure("Edit distance not supported by PCRE.");
        }
        if (ext.flags & HS_EXT_FLAG_HAMMING_DISTANCE) {
            throw SoftPcreCompileFailure(
                "Hamming distance not supported by PCRE.");
        }
        throw PcreCompileFailure("Unsupported extended flags.");
    }

    // Hybrid mode implies SOM.
    if (colliderMode == MODE_HYBRID) {
        assert(!use_NFA);
        som = true;
    }

    // SOM flags might be set globally.
    som |= !!somFlags;

    // For traditional Hyperscan, add global callout to pattern.
    if (!combination && !no_callouts && isStandardMode(colliderMode)) {
        addCallout(re);
    }

    // Compile the pattern
    const char *errptr = nullptr;
    int errloc = 0;
    int errcode = 0;

    unique_ptr<CompiledPcre> compiled = make_unique<CompiledPcre>();
    compiled->utf8 = flags & PCRE_UTF8;
    compiled->highlander = highlander;
    compiled->prefilter = prefilter;
    compiled->som = som;
    compiled->combination = combination;
    compiled->quiet = quiet;
    compiled->min_offset = ext.min_offset;
    compiled->max_offset = ext.max_offset;
    compiled->min_length = ext.min_length;
    compiled->expression = i->second; // original PCRE
    flags |= PCRE_NO_AUTO_POSSESS;

    if (compiled->combination) {
        compiled->pl.parseLogicalCombination(id, re.c_str(), ~0U, 0, ~0ULL);
        compiled->pl.logicalKeyRenumber();
        compiled->report = id;
        return compiled;
    }

    compiled->bytecode =
        pcre_compile2(re.c_str(), flags, &errcode, &errptr, &errloc, nullptr);

    if (!compiled->bytecode || errptr) {
        assert(errcode);
        ostringstream oss;
        oss << "Failed to compile expression '" << re << '\'';
        oss << " (" << errptr << " at " << errloc << ").";
        if (errcode == 20) { // "regular expression is too large"
            throw SoftPcreCompileFailure(oss.str());
        } else if (errcode == 25) { // "lookbehind assertion is not fixed length"
            throw SoftPcreCompileFailure(oss.str());
        } else {
            throw PcreCompileFailure(oss.str());
        }
    }

    // Study the pattern
    shared_ptr<pcre_extra> extra(pcre_study(compiled->bytecode, 0, &errptr),
                                 free);
    if (errptr) {
        ostringstream oss;
        oss << "Error studying pattern (" << errptr << ").";
        throw PcreCompileFailure(oss.str());
    }

    int infoRes =
        pcre_fullinfo(compiled->bytecode, extra.get(), PCRE_INFO_CAPTURECOUNT,
                      &compiled->captureCount);
    if (infoRes < PCRE_ERROR_NOMATCH) {
        ostringstream oss;
        oss << "Error determining number of capturing subpatterns ("
            << pcreErrStr(infoRes) << ").";
        throw PcreCompileFailure(oss.str());
    }

    compiled->utf8 |= isUtf8(*compiled);

    return compiled;
}

static
void filterLeftmostSom(ResultSet &rs) {
    if (rs.matches.size() <= 1) {
        return;
    }

    set<u64a> seen; // End offsets.
    set<MatchResult>::iterator it = rs.matches.begin();
    while (it != rs.matches.end()) {
        if (seen.insert(it->to).second) {
            ++it; // First time we've seen this end-offset.
        } else {
            rs.matches.erase(it++); // Dupe with a "righter" SOM.
        }
    }
}

static
void filterExtParams(ResultSet &rs, const CompiledPcre &compiled) {
    set<MatchResult>::iterator it = rs.matches.begin();
    while (it != rs.matches.end()) {
        unsigned int from = it->from, to = it->to;
        unsigned int len = to - from;
        if (to < compiled.min_offset || to > compiled.max_offset ||
                len < compiled.min_length) {
            rs.matches.erase(it++);
        } else {
            ++it;
        }
    }
}

static
int scanBasic(const CompiledPcre &compiled, const string &buffer,
              const pcre_extra &extra, vector<int> &ovector,
              CalloutContext &ctx) {
    const size_t prefix_len = g_corpora_prefix.size();
    const size_t suffix_len = g_corpora_suffix.size();

    size_t begin_offset = prefix_len - MIN(50, prefix_len);
    size_t real_len = buffer.size();

    if (suffix_len > 2) {
        real_len -= suffix_len - 2;
    }

    int flags = suffix_len ? PCRE_NOTEOL : 0;
    int ret = pcre_exec(compiled.bytecode, &extra, buffer.c_str(), real_len,
                        begin_offset, flags, &ovector[0], ovector.size());

    if (!g_corpora_prefix.empty()) {
        PcreMatchSet tmp;
        tmp.swap(ctx.matches);

        for (const auto &m : tmp) {
            unsigned from = m.first;
            unsigned to = m.second;
            if (to >= prefix_len && to <= buffer.size() - suffix_len) {
                from = from < prefix_len ? 0 : from - prefix_len;
                to -= prefix_len;
                ctx.matches.insert(make_pair(from, to));
            }
        }
    }

    return ret;
}

static
CaptureVec makeCaptureVec(const vector<int> &ovector, int ret) {
    assert(ret > 0);

    CaptureVec cap;

    if (no_groups) {
        return cap; // No group info requested.
    }

    cap.reserve(ret * 2);
    for (int i = 0; i < ret * 2; i += 2) {
        int from = ovector[i], to = ovector[i + 1];
        cap.push_back(make_pair(from, to));
    }
    return cap;
}

static
int scanHybrid(const CompiledPcre &compiled, const string &buffer,
               const pcre_extra &extra, vector<int> &ovector,
               ResultSet &rs, ostream &out) {
    int len = (int)buffer.length();
    int startoffset = 0;
    bool utf8 = isUtf8(compiled);

    int flags = 0;
    int ret;
    do {
        ret = pcre_exec(compiled.bytecode, &extra, buffer.c_str(), len,
                        startoffset, flags, &ovector[0], ovector.size());

        if (ret <= PCRE_ERROR_NOMATCH) {
            return ret;
        }

        int from = ovector.at(0);
        int to = ovector.at(1);
        rs.addMatch(from, to, makeCaptureVec(ovector, ret));

        if (echo_matches) {
            out << "PCRE Match @ (" << from << "," << to << ")" << endl;
        }

        // If we only wanted a single match, we're done.
        if (compiled.highlander) break;

        // Next scan starts at the first codepoint after the match. It's
        // possible that we have a vacuous match, in which case we must step
        // past it to ensure that we always progress.
        if (from != to) {
            startoffset = to;
        } else if (utf8) {
            startoffset = to + 1;
            while (startoffset < len
                   && ((buffer[startoffset] & 0xc0) == UTF_CONT_BYTE_HEADER)) {
                ++startoffset;
            }
        } else {
            startoffset = to + 1;
        }
    } while (startoffset <= len);

    return ret;
}

static
int scanOffset(const CompiledPcre &compiled, const string &buffer,
               const pcre_extra &extra, vector<int> &ovector,
               CalloutContext &ctx) {
    size_t offset = MIN(100, g_streamOffset);
    assert(offset > 0);

    const string buf(string(offset, '\0') + buffer);

    // First, scan our preamble so that we can discard any matches therein
    // after the real scan, later. We use PCRE_NOTEOL so that end-anchors in
    // our expression don't match at the end of the preamble.
    int ret = pcre_exec(compiled.bytecode, &extra, buf.c_str(), offset, 0,
                        PCRE_NOTEOL, &ovector[0], ovector.size());
    if (ret < PCRE_ERROR_NOMATCH) {
        return ret;
    }

    PcreMatchSet pre_matches;
    pre_matches.swap(ctx.matches);

    // Real scan.
    ret = pcre_exec(compiled.bytecode, &extra, buf.c_str(), buf.size(), 0, 0,
                    &ovector[0], ovector.size());
    if (ret < PCRE_ERROR_NOMATCH) {
        return ret;
    }

    // Erase any matches due entirely to the preamble.
    for (const auto &m : pre_matches) {
        ctx.matches.erase(m);
    }

    return ret;
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

/** \brief Returns 1 if combination matches when no sub-expression matches. */
static
char isPurelyNegativeMatch(vector<char> &lv, const vector<LogicalOp> &comb,
                           size_t lkeyCount, unsigned start, unsigned result) {
    assert(start <= result);
    for (unsigned i = start; i <= result; i++) {
        const LogicalOp &op = comb[i - lkeyCount];
        assert(i == op.id);
        switch (op.op) {
        case LOGICAL_OP_NOT:
            if ((op.ro < lkeyCount) && lv[op.ro]) {
                // sub-expression not negative
                return 0;
            }
            lv[op.id] = !lv[op.ro];
            break;
        case LOGICAL_OP_AND:
            if (((op.lo < lkeyCount) && lv[op.lo]) ||
                ((op.ro < lkeyCount) && lv[op.ro])) {
                // sub-expression not negative
                return 0;
            }
            lv[op.id] = lv[op.lo] & lv[op.ro]; // &&
            break;
        case LOGICAL_OP_OR:
            if (((op.lo < lkeyCount) && lv[op.lo]) ||
                ((op.ro < lkeyCount) && lv[op.ro])) {
                // sub-expression not negative
                return 0;
            }
            lv[op.id] = lv[op.lo] | lv[op.ro]; // ||
            break;
        default:
            assert(0);
            break;
        }
    }
    return lv[result];
}

bool GroundTruth::run(unsigned, const CompiledPcre &compiled,
                      const string &buffer, ResultSet &rs, string &error) {
    if (compiled.quiet) {
        return true;
    }

    if (compiled.combination) {
        // Compile and run sub-expressions, store match results.
        map<unsigned long long, set<MatchResult>> offset_to_matches;
        map<unsigned long long, set<unsigned>> offset_to_lkeys;
        set<unsigned> sub_exps;
        const auto &m_lkey = compiled.pl.getLkeyMap();
        for (const auto &it_lkey : m_lkey) {
            if (sub_exps.find(it_lkey.first) == sub_exps.end()) {
                sub_exps.emplace(it_lkey.first);
                ResultSet sub_rs(RESULT_FROM_PCRE);
                shared_ptr<CompiledPcre> sub_pcre;
                try {
                    sub_pcre = compile(it_lkey.first);
                }
                catch (const SoftPcreCompileFailure &err) {
                    return false;
                }
                catch (const PcreCompileFailure &err) {
                    return false;
                }
                sub_pcre->quiet = false; // force not quiet in sub-exp.
                if (!run(it_lkey.first, *sub_pcre, buffer, sub_rs, error)) {
                    rs.clear();
                    return false;
                }
                for (const auto &it_mr : sub_rs.matches) {
                    offset_to_matches[it_mr.to].emplace(it_mr);
                    offset_to_lkeys[it_mr.to].emplace(it_lkey.second);
                    if (sub_pcre->highlander) {
                        break;
                    }
                }
            }
        }
        // Calculate rs for combination expression.
        vector<char> lv;
        const auto &comb = compiled.pl.getLogicalTree();
        lv.resize(m_lkey.size() + comb.size());
        const auto &li = compiled.pl.getCombInfoById(compiled.report);
        for (const auto &it : offset_to_lkeys) {
            for (auto report : it.second) {
                lv[report] = 1;
            }
            if (isLogicalCombination(lv, comb, m_lkey.size(),
                                     li.start, li.result)) {
                for (const auto &mr : offset_to_matches.at(it.first)) {
                    if ((mr.to >= compiled.min_offset) &&
                        (mr.to <= compiled.max_offset)) {
                        rs.addMatch(mr.from, mr.to);
                    }
                }
            }
        }
        if (isPurelyNegativeMatch(lv, comb, m_lkey.size(),
                                  li.start, li.result)) {
            u64a to = buffer.length();
            if ((to >= compiled.min_offset) && (to <= compiled.max_offset)) {
                rs.addMatch(0, to);
            }
        }
        return true;
    }

    CalloutContext ctx(out);

    pcre_extra extra;
    extra.flags = 0;

    // If running in traditional HyperScan mode, switch on callouts.
    bool usingCallouts = isStandardMode(colliderMode);
    if (usingCallouts) {
        // Switch on callouts.
        extra.flags |= PCRE_EXTRA_CALLOUT_DATA;
        extra.callout_data = &ctx;
    }

    // Set the match_limit (in order to bound execution time on very complex
    // patterns)
    extra.flags |= (PCRE_EXTRA_MATCH_LIMIT | PCRE_EXTRA_MATCH_LIMIT_RECURSION);
    if (colliderMode == MODE_HYBRID) {
        extra.match_limit = 10000000;
        extra.match_limit_recursion = 1500;
    } else {
        extra.match_limit = matchLimit;
        extra.match_limit_recursion = matchLimitRecursion;
    }

#ifdef PCRE_NO_START_OPTIMIZE
    // Switch off optimizations that may result in callouts not occurring.
    extra.flags |= PCRE_NO_START_OPTIMIZE;
#endif

    // Ensure there's enough room in the ovector for the capture groups in this
    // pattern.
    int ovecsize = (compiled.captureCount + 1) * 3;
    ovector.resize(ovecsize);

    int ret;
    bool hybrid = false;
    switch (colliderMode) {
    case MODE_BLOCK:
    case MODE_STREAMING:
    case MODE_VECTORED:
        if (g_streamOffset) {
            ret = scanOffset(compiled, buffer, extra, ovector, ctx);
        } else {
            ret = scanBasic(compiled, buffer, extra, ovector, ctx);
        }
        break;
    case MODE_HYBRID:
        ret = scanHybrid(compiled, buffer, extra, ovector, rs, out);
        hybrid = true;
        break;
    default:
        assert(0);
        ret = PCRE_ERROR_NULL;
        break;
    }

    if (ret < PCRE_ERROR_NOMATCH) {
        error = pcreErrStr(ret);
        return false;
    }

    // Move matches into a ResultSet.
    for (const auto &m : ctx.matches) {
        unsigned long long from = m.first;
        unsigned long long to = m.second;

        if (g_streamOffset) {
            // Subtract stream offset imposed by offset test.
            unsigned long long offset = min(100ull, g_streamOffset);
            assert(to >= offset);
            from -= min(offset, from);
            to -= offset;
        }

        rs.addMatch(from, to);
    }

    // If we have no matches, there's no further work to do.
    if (rs.matches.empty()) {
        return true;
    }

    if (compiled.som && !hybrid) {
        filterLeftmostSom(rs);
    }

    filterExtParams(rs, compiled);

    // If we haven't been asked for SOM, strip the from offsets.
    if (!compiled.som) {
        set<MatchResult> endonly;
        for (const auto &m : rs.matches) {
            endonly.insert(MatchResult(0, m.to));
        }
        rs.matches.swap(endonly);
    }

    return true;
}
