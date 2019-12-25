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

#ifndef ROSE_BUILD_IMPL_H
#define ROSE_BUILD_IMPL_H

#include "rose_build.h"
#include "rose_build_util.h"
#include "rose_common.h"
#include "rose_graph.h"
#include "nfa/mpvcompile.h"
#include "nfa/goughcompile.h"
#include "nfa/nfa_internal.h"
#include "nfagraph/ng_holder.h"
#include "nfagraph/ng_revacc.h"
#include "util/bytecode_ptr.h"
#include "util/flat_containers.h"
#include "util/hash.h"
#include "util/order_check.h"
#include "util/queue_index_factory.h"
#include "util/ue2string.h"
#include "util/unordered.h"
#include "util/verify_types.h"

#include <deque>
#include <map>
#include <string>
#include <vector>
#include <boost/variant.hpp>

struct RoseEngine;

namespace ue2 {

#define ROSE_GROUPS_MAX 64

#define ROSE_LONG_LITERAL_THRESHOLD_MIN 33

/**
 * \brief The largest allowable "short" literal fragment which can be given to
 * a literal matcher directly.
 *
 * Literals longer than this will be truncated to their suffix and confirmed in
 * the Rose interpreter, either as "medium length" literals which can be
 * confirmed from history, or "long literals" which make use of the streaming
 * table support.
 */
#define ROSE_SHORT_LITERAL_LEN_MAX 8

struct BoundaryReports;
struct CastleProto;
struct CompileContext;
class ReportManager;
class SmallWriteBuild;
class SomSlotManager;

struct suffix_id {
    suffix_id(const RoseSuffixInfo &in)
        : g(in.graph.get()), c(in.castle.get()), d(in.rdfa.get()),
          h(in.haig.get()), t(in.tamarama.get()),
          dfa_min_width(in.dfa_min_width),
          dfa_max_width(in.dfa_max_width) {
            assert(!g || g->kind == NFA_SUFFIX);
    }
    bool operator==(const suffix_id &b) const {
        bool rv = g == b.g && c == b.c && h == b.h && d == b.d && t == b.t;
        assert(!rv || dfa_min_width == b.dfa_min_width);
        assert(!rv || dfa_max_width == b.dfa_max_width);
        return rv;
    }
    bool operator!=(const suffix_id &b) const { return !(*this == b); }
    bool operator<(const suffix_id &b) const {
        const suffix_id &a = *this;
        ORDER_CHECK(g);
        ORDER_CHECK(c);
        ORDER_CHECK(d);
        ORDER_CHECK(h);
        ORDER_CHECK(t);
        return false;
    }

    NGHolder *graph() {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return g;
    }
    const NGHolder *graph() const {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return g;
    }
    CastleProto *castle() {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return c;
    }
    const CastleProto *castle() const {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return c;
    }
    TamaProto *tamarama() {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return t;
    }
    const TamaProto *tamarama() const {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return t;
    }


    raw_som_dfa *haig() { return h; }
    const raw_som_dfa *haig() const { return h; }
    raw_dfa *dfa() { return d; }
    const raw_dfa *dfa() const { return d; }

    size_t hash() const;

private:
    NGHolder *g;
    CastleProto *c;
    raw_dfa *d;
    raw_som_dfa *h;
    TamaProto *t;
    depth dfa_min_width;
    depth dfa_max_width;

    friend depth findMinWidth(const suffix_id &s);
    friend depth findMaxWidth(const suffix_id &s);
    friend depth findMinWidth(const suffix_id &s, u32 top);
    friend depth findMaxWidth(const suffix_id &s, u32 top);
};

std::set<ReportID> all_reports(const suffix_id &s);
std::set<u32> all_tops(const suffix_id &s);
bool has_eod_accepts(const suffix_id &s);
bool has_non_eod_accepts(const suffix_id &s);
depth findMinWidth(const suffix_id &s);
depth findMaxWidth(const suffix_id &s);
depth findMinWidth(const suffix_id &s, u32 top);
depth findMaxWidth(const suffix_id &s, u32 top);

/** \brief represents an engine to the left of a rose role */
struct left_id {
    left_id(const LeftEngInfo &in)
        : g(in.graph.get()), c(in.castle.get()), d(in.dfa.get()),
          h(in.haig.get()), dfa_min_width(in.dfa_min_width),
          dfa_max_width(in.dfa_max_width) {
        assert(!g || !has_managed_reports(*g));
    }
    bool operator==(const left_id &b) const {
        bool rv = g == b.g && c == b.c && h == b.h && d == b.d;
        assert(!rv || dfa_min_width == b.dfa_min_width);
        assert(!rv || dfa_max_width == b.dfa_max_width);
        return rv;
    }
    bool operator!=(const left_id &b) const { return !(*this == b); }
    bool operator<(const left_id &b) const {
        const left_id &a = *this;
        ORDER_CHECK(g);
        ORDER_CHECK(c);
        ORDER_CHECK(d);
        ORDER_CHECK(h);
        return false;
    }

    NGHolder *graph() {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return g;
    }
    const NGHolder *graph() const {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }
        return g;
    }
    CastleProto *castle() {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }

        return c;
    }
    const CastleProto *castle() const {
        if (!d && !h) {
            assert(dfa_min_width == depth(0));
            assert(dfa_max_width == depth::infinity());
        }

        return c;
    }
    raw_som_dfa *haig() { return h; }
    const raw_som_dfa *haig() const { return h; }
    raw_dfa *dfa() { return d; }
    const raw_dfa *dfa() const { return d; }

    size_t hash() const;

private:
    NGHolder *g;
    CastleProto *c;
    raw_dfa *d;
    raw_som_dfa *h;
    depth dfa_min_width;
    depth dfa_max_width;

    friend bool isAnchored(const left_id &r);
    friend depth findMinWidth(const left_id &r);
    friend depth findMaxWidth(const left_id &r);
};

std::set<u32> all_tops(const left_id &r);
std::set<ReportID> all_reports(const left_id &left);
bool isAnchored(const left_id &r);
depth findMinWidth(const left_id &r);
depth findMaxWidth(const left_id &r);
u32 num_tops(const left_id &r);

struct rose_literal_info {
    flat_set<u32> delayed_ids;
    flat_set<RoseVertex> vertices;
    rose_group group_mask = 0;
    u32 undelayed_id = MO_INVALID_IDX;
    bool squash_group = false;
    bool requires_benefits = false;
};

/**
 * \brief Main literal struct used at Rose build time. Numeric literal IDs
 * used at build time point at these (via the RoseBuildImpl::literals map).
 */
struct rose_literal_id {
    rose_literal_id(const ue2_literal &s_in, rose_literal_table table_in,
                    u32 delay_in)
        : s(s_in), table(table_in), delay(delay_in), distinctiveness(0) {}

    rose_literal_id(const ue2_literal &s_in, const std::vector<u8> &msk_in,
                    const std::vector<u8> &cmp_in, rose_literal_table table_in,
                    u32 delay_in);

    ue2_literal s;
    std::vector<u8> msk;
    std::vector<u8> cmp;
    rose_literal_table table;
    u32 delay;
    u32 distinctiveness;

    size_t elength(void) const { return s.length() + delay; }
    size_t elength_including_mask(void) const {
        size_t mask_len = msk.size();
        for (u8 c : msk) {
            if (!c) {
                mask_len--;
            } else {
                break;
            }
        }
        return MAX(mask_len, s.length()) + delay;
    }

    bool operator==(const rose_literal_id &b) const {
        return s == b.s && msk == b.msk && cmp == b.cmp && table == b.table &&
               delay == b.delay && distinctiveness == b.distinctiveness;
    }

    size_t hash() const {
        return hash_all(s, msk, cmp, table, delay, distinctiveness);
    }
};

static inline
bool operator<(const rose_literal_id &a, const rose_literal_id &b) {
    ORDER_CHECK(distinctiveness);
    ORDER_CHECK(table);
    ORDER_CHECK(s);
    ORDER_CHECK(delay);
    ORDER_CHECK(msk);
    ORDER_CHECK(cmp);
    return 0;
}

class RoseLiteralMap {
    /**
     * \brief Main storage for literals.
     *
     * Note that this cannot be a vector, as the present code relies on
     * iterator stability when iterating over this list and adding to it inside
     * the loop.
     */
    std::deque<rose_literal_id> lits;

    /** \brief Quick-lookup index from literal -> index in lits. */
    ue2_unordered_map<rose_literal_id, u32> lits_index;

public:
    std::pair<u32, bool> insert(const rose_literal_id &lit) {
        auto it = lits_index.find(lit);
        if (it != lits_index.end()) {
            return {it->second, false};
        }
        u32 id = verify_u32(lits.size());
        lits.push_back(lit);
        lits_index.emplace(lit, id);
        return {id, true};
    }

    // Erase the last num elements.
    void erase_back(size_t num) {
        assert(num <= lits.size());
        for (size_t i = 0; i < num; i++) {
            lits_index.erase(lits.back());
            lits.pop_back();
        }
        assert(lits.size() == lits_index.size());
    }

    const rose_literal_id &at(u32 id) const {
        assert(id < lits.size());
        return lits.at(id);
    }

    using const_iterator = decltype(lits)::const_iterator;
    const_iterator begin() const { return lits.begin(); }
    const_iterator end() const { return lits.end(); }

    size_t size() const {
        return lits.size();
    }
};

struct simple_anchored_info {
    simple_anchored_info(u32 min_b, u32 max_b, const ue2_literal &lit)
    : min_bound(min_b), max_bound(max_b), literal(lit) {}
    u32 min_bound; /**< min number of characters required before literal can
                    * start matching */
    u32 max_bound; /**< max number of characters allowed before literal can
                    * start matching */
    ue2_literal literal;
};

static really_inline
bool operator<(const simple_anchored_info &a, const simple_anchored_info &b) {
    ORDER_CHECK(min_bound);
    ORDER_CHECK(max_bound);
    ORDER_CHECK(literal);
    return 0;
}

struct MpvProto {
    bool empty() const {
        return puffettes.empty() && triggered_puffettes.empty();
    }
    void reset() {
        puffettes.clear();
        triggered_puffettes.clear();
    }
    std::vector<raw_puff> puffettes;
    std::vector<raw_puff> triggered_puffettes;
};

struct OutfixInfo {
    template<class T>
    explicit OutfixInfo(std::unique_ptr<T> x) : proto(std::move(x)) {}

    explicit OutfixInfo(MpvProto mpv_in) : proto(std::move(mpv_in)) {}

    u32 get_queue(QueueIndexFactory &qif);

    u32 get_queue() const {
        assert(queue != ~0U);
        return queue;
    }

    bool is_nonempty_mpv() const {
        auto *m = boost::get<MpvProto>(&proto);
        return m && !m->empty();
    }

    bool is_dead() const {
        auto *m = boost::get<MpvProto>(&proto);
        if (m) {
            return m->empty();
        }
        return boost::get<boost::blank>(&proto) != nullptr;
    }

    void clear() {
        proto = boost::blank();
    }

    // Convenience accessor functions.

    NGHolder *holder() {
        auto *up = boost::get<std::unique_ptr<NGHolder>>(&proto);
        return up ? up->get() : nullptr;
    }
    raw_dfa *rdfa() {
        auto *up = boost::get<std::unique_ptr<raw_dfa>>(&proto);
        return up ? up->get() : nullptr;
    }
    raw_som_dfa *haig() {
        auto *up = boost::get<std::unique_ptr<raw_som_dfa>>(&proto);
        return up ? up->get() : nullptr;
    }
    MpvProto *mpv() {
        return boost::get<MpvProto>(&proto);
    }

    // Convenience const accessor functions.

    const NGHolder *holder() const {
        auto *up = boost::get<std::unique_ptr<NGHolder>>(&proto);
        return up ? up->get() : nullptr;
    }
    const raw_dfa *rdfa() const {
        auto *up = boost::get<std::unique_ptr<raw_dfa>>(&proto);
        return up ? up->get() : nullptr;
    }
    const raw_som_dfa *haig() const {
        auto *up = boost::get<std::unique_ptr<raw_som_dfa>>(&proto);
        return up ? up->get() : nullptr;
    }
    const MpvProto *mpv() const {
        return boost::get<MpvProto>(&proto);
    }

    /**
     * \brief Variant wrapping the various engine types. If this is
     * boost::blank, it means that this outfix is unused (dead).
     */
    boost::variant<
        boost::blank,
        std::unique_ptr<NGHolder>,
        std::unique_ptr<raw_dfa>,
        std::unique_ptr<raw_som_dfa>,
        MpvProto> proto = boost::blank();

    RevAccInfo rev_info;
    u32 maxBAWidth = 0; //!< max bi-anchored width
    depth minWidth{depth::infinity()};
    depth maxWidth{0};
    u64a maxOffset = 0;
    bool in_sbmatcher = false; //!< handled by small-block matcher.

private:
    u32 queue = ~0U;
};

std::set<ReportID> all_reports(const OutfixInfo &outfix);

// Concrete impl class
class RoseBuildImpl : public RoseBuild {
public:
    RoseBuildImpl(ReportManager &rm, SomSlotManager &ssm, SmallWriteBuild &smwr,
                  const CompileContext &cc, const BoundaryReports &boundary);

    ~RoseBuildImpl() override;

    // Adds a single literal.
    void add(bool anchored, bool eod, const ue2_literal &lit,
             const flat_set<ReportID> &ids) override;

    bool addRose(const RoseInGraph &ig, bool prefilter) override;
    bool addSombeRose(const RoseInGraph &ig) override;

    bool addOutfix(const NGHolder &h) override;
    bool addOutfix(const NGHolder &h, const raw_som_dfa &haig) override;
    bool addOutfix(const raw_puff &rp) override;

    bool addChainTail(const raw_puff &rp, u32 *queue_out, u32 *event_out) override;

    // Returns true if we were able to add it as a mask
    bool add(bool anchored, const std::vector<CharReach> &mask,
             const flat_set<ReportID> &reports) override;

    bool addAnchoredAcyclic(const NGHolder &graph) override;

    bool validateMask(const std::vector<CharReach> &mask,
                      const flat_set<ReportID> &reports, bool anchored,
                      bool eod) const override;
    void addMask(const std::vector<CharReach> &mask,
                 const flat_set<ReportID> &reports, bool anchored,
                 bool eod) override;

    // Construct a runtime implementation.
    bytecode_ptr<RoseEngine> buildRose(u32 minWidth) override;
    bytecode_ptr<RoseEngine> buildFinalEngine(u32 minWidth);

    void setSom() override { hasSom = true; }

    std::unique_ptr<RoseDedupeAux> generateDedupeAux() const override;

    // Find the maximum bound on the edges to this vertex's successors.
    u32 calcSuccMaxBound(RoseVertex u) const;

    /* Returns the ID of the given literal in the literal map, adding it if
     * necessary. */
    u32 getLiteralId(const ue2_literal &s, u32 delay, rose_literal_table table);

    // Variant with msk/cmp.
    u32 getLiteralId(const ue2_literal &s, const std::vector<u8> &msk,
                     const std::vector<u8> &cmp, u32 delay,
                     rose_literal_table table);

    u32 getNewLiteralId(void);

    void removeVertices(const std::vector<RoseVertex> &dead);

    // Is the Rose anchored?
    bool hasNoFloatingRoots() const;

    u32 calcHistoryRequired() const;

    rose_group getInitialGroups() const;
    rose_group getSuccGroups(RoseVertex start) const;
    rose_group getGroups(RoseVertex v) const;

    bool hasDelayedLiteral(RoseVertex v) const;
    bool hasDelayPred(RoseVertex v) const;
    bool hasLiteralInTable(RoseVertex v, enum rose_literal_table t) const;
    bool hasAnchoredTablePred(RoseVertex v) const;

    // Is the given vertex a successor of either root or anchored_root?
    bool isRootSuccessor(const RoseVertex &v) const;
    /* Is the given vertex a successor of something other than root or
     * anchored_root? */
    bool isNonRootSuccessor(const RoseVertex &v) const;

    bool isDirectReport(u32 id) const;
    bool isDelayed(u32 id) const;

    bool isAnchored(RoseVertex v) const; /* true iff has literal in anchored
                                          * table */
    bool isFloating(RoseVertex v) const; /* true iff has literal in floating
                                          * table */
    bool isInETable(RoseVertex v) const; /* true iff has literal in eod
                                          * table */

    size_t maxLiteralLen(RoseVertex v) const;
    size_t minLiteralLen(RoseVertex v) const;

    // max overlap considered for every pair (ulit, vlit).
    size_t maxLiteralOverlap(RoseVertex u, RoseVertex v) const;

    bool isPseudoStar(const RoseEdge &e) const;
    bool isPseudoStarOrFirstOnly(const RoseEdge &e) const;
    bool hasOnlyPseudoStarInEdges(RoseVertex v) const;

    bool isAnyStart(const RoseVertex &v) const {
        return v == root || v == anchored_root;
    }

    bool isVirtualVertex(const RoseVertex &v) const {
        return g[v].eod_accept || isAnyStart(v);
    }

    void handleMixedSensitivity(void);

    void findTransientLeftfixes(void);

    const CompileContext &cc;
    RoseGraph g;
    const RoseVertex root;
    const RoseVertex anchored_root;
    RoseLiteralMap literals;
    std::map<RoseVertex, RoseVertex> ghost;
    ReportID getNewNfaReport() override {
        return next_nfa_report++;
    }
    std::deque<rose_literal_info> literal_info;
    bool hasSom; //!< at least one pattern requires SOM.
    std::map<size_t, std::vector<std::unique_ptr<raw_dfa>>> anchored_nfas;
    std::map<simple_anchored_info, std::set<u32>> anchored_simple;
    std::map<u32, std::set<u32> > group_to_literal;
    u32 group_end;

    u32 ematcher_region_size; /**< number of bytes the eod table runs over */

    /** \brief Mapping from anchored literal ID to the original literal suffix
     * present when the literal was added to the literal matcher. Used for
     * overlap calculation in history assignment. */
    std::map<u32, rose_literal_id> anchoredLitSuffix;

    ue2_unordered_set<left_id> transient;
    ue2_unordered_map<left_id, rose_group> rose_squash_masks;

    std::vector<OutfixInfo> outfixes;

    /** \brief MPV outfix entry. Null if not used, and moved into the outfixes
     * list before we start building the bytecode (at which point it is set to
     * null again). */
    std::unique_ptr<OutfixInfo> mpv_outfix = nullptr;

    u32 eod_event_literal_id; // ID of EOD event literal, or MO_INVALID_IDX.

    u32 max_rose_anchored_floating_overlap;

    rose_group boundary_group_mask = 0;

    QueueIndexFactory qif;
    ReportManager &rm;
    SomSlotManager &ssm;
    SmallWriteBuild &smwr;
    const BoundaryReports &boundary;

private:
    ReportID next_nfa_report;
};

size_t calcLongLitThreshold(const RoseBuildImpl &build,
                            const size_t historyRequired);

// Free functions, in rose_build_misc.cpp

bool hasAnchHistorySucc(const RoseGraph &g, RoseVertex v);
bool hasLastByteHistorySucc(const RoseGraph &g, RoseVertex v);

size_t maxOverlap(const rose_literal_id &a, const rose_literal_id &b);
ue2_literal findNonOverlappingTail(const std::set<ue2_literal> &lits,
                                   const ue2_literal &s);

#ifndef NDEBUG
bool roseHasTops(const RoseBuildImpl &build, RoseVertex v);
bool hasOrphanedTops(const RoseBuildImpl &build);
#endif

u64a findMaxOffset(const std::set<ReportID> &reports, const ReportManager &rm);

// Function that operates on a msk/cmp pair and a literal, as used in
// hwlmLiteral, and zeroes msk elements that don't add any power to the
// literal.
void normaliseLiteralMask(const ue2_literal &s, std::vector<u8> &msk,
                          std::vector<u8> &cmp);

u32 findMinOffset(const RoseBuildImpl &build, u32 lit_id);
u32 findMaxOffset(const RoseBuildImpl &build, u32 lit_id);

bool canEagerlyReportAtEod(const RoseBuildImpl &build, const RoseEdge &e);

#ifndef NDEBUG
bool canImplementGraphs(const RoseBuildImpl &tbi);
#endif

} // namespace ue2

namespace std {

template<>
struct hash<ue2::left_id> {
    size_t operator()(const ue2::left_id &l) const {
        return l.hash();
    }
};

template<>
struct hash<ue2::suffix_id> {
    size_t operator()(const ue2::suffix_id &s) const {
        return s.hash();
    }
};

} // namespace std

#endif /* ROSE_BUILD_IMPL_H */
