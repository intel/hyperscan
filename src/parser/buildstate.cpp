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

/** \file
 * \brief Glushkov construction.
 */
#include "buildstate.h"
#include "position.h"
#include "position_dump.h"
#include "position_info.h"
#include "parse_error.h"
#include "hs_internal.h"
#include "ue2common.h"
#include "nfagraph/ng_builder.h"
#include "util/charreach.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/hash.h"
#include "util/make_unique.h"
#include "util/unordered.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <map>
#include <utility>

#if defined(DEBUG) || defined(DUMP_SUPPORT)
#include <ostream>
#include <sstream>
#endif

using namespace std;

namespace ue2 {

/** \brief Represents an uninitialized state. */
const Position GlushkovBuildState::POS_UNINITIALIZED =
    numeric_limits<Position>::max();

/** \brief Represents an epsilon transition in the firsts of a component. */
const Position GlushkovBuildState::POS_EPSILON =
    numeric_limits<Position>::max() - 1;

GlushkovBuildState::~GlushkovBuildState() { }

namespace /* anonymous */ {

class CheckPositionFlags {
public:
    explicit CheckPositionFlags(int fl) : flags(fl) {}
    bool operator()(const PositionInfo &p) const {
        return (p.flags & flags) == flags;
    }
private:
    int flags;
};

class CheckUnflaggedEpsilon {
public:
    bool operator()(const PositionInfo &p) const {
        return p.pos == GlushkovBuildState::POS_EPSILON && p.flags == 0;
    }
};

/** \brief Concrete impl of the GlushkovBuildState interface. */
class GlushkovBuildStateImpl : public GlushkovBuildState {
public:
    GlushkovBuildStateImpl(NFABuilder &b, bool prefilter);

    /** \brief Returns a reference to the NFABuilder being used. */
    NFABuilder &getBuilder() override { return builder; }

    /** \brief Returns a const reference to the NFABuilder being used. */
    const NFABuilder &getBuilder() const override { return builder; }

    /** \brief Wire up the lasts of one component to the firsts of another. */
    void connectRegions(const vector<PositionInfo> &lasts,
                        const vector<PositionInfo> &firsts) override;

    /** \brief Wire the lasts of the main sequence to accepts. */
    void connectAccepts(const vector<PositionInfo> &lasts) override;

    /** \brief Wire up a single last to a list of firsts. */
    void connectSuccessors(const PositionInfo &last,
                           vector<PositionInfo> firsts);

    /** Wire up a pair of positions. */
    void addSuccessor(Position from, Position to) override;

    /** \brief Clone the vertex properties and edges of all vertices between
     * two positions. */
    void cloneFollowSet(Position from, Position to, unsigned offset) override;

    /** \brief Build the prioritised list of edges out of our successor map. */
    void buildEdges() override;

    /** Construct an edge, called internally by \ref buildEdges. */
    void buildEdge(Position from, const PositionInfo &to);

    Position startState;
    Position startDotstarState;
    Position acceptState;
    Position acceptEodState;
    Position acceptNlEodState;
    Position acceptNlState;

    NFABuilder &builder; //!< \brief builder for the NFAGraph

    bool doPrefilter; //!< \brief we're building a prefiltering pattern

    /** \brief Map storing successors for each position. */
    map<Position, flat_set<PositionInfo>> successors;
};

} // namespace

GlushkovBuildStateImpl::GlushkovBuildStateImpl(NFABuilder &b,
                                               bool prefilter) :
      startState(b.getStart()),
      startDotstarState(b.getStartDotStar()),
      acceptState(b.getAccept()),
      acceptEodState(b.getAcceptEOD()),
      acceptNlEodState(POS_UNINITIALIZED),
      acceptNlState(POS_UNINITIALIZED),
      builder(b),
      doPrefilter(prefilter)
{
    // Our special nodes need special relationships.
    vector<PositionInfo> lasts, firsts;

    // start->startDs and startDs self-loop.
    lasts.push_back(startState);
    lasts.push_back(startDotstarState);
    firsts.push_back(startDotstarState);
    connectRegions(lasts, firsts);

    // accept to acceptEod edges already wired

    // XXX: a small hack to support vacuous NFAs: give start and startDs an
    // initial report ID.
    builder.setNodeReportID(startState, 0);
    builder.setNodeReportID(startDotstarState, 0);
}

static
void checkEmbeddedEndAnchor(const PositionInfo &from,
                            const vector<PositionInfo> &firsts) {
    if (!(from.flags & POS_FLAG_ONLY_ENDS)) {
        return;
    }

    for (const auto &first : firsts) {
        if (first.pos != GlushkovBuildStateImpl::POS_EPSILON) {
            /* can make it through the parse tree */
            throw ParseError("Embedded end anchors not supported.");
        }
    }
}

// Wire up the lasts of one component to the firsts of another
void
GlushkovBuildStateImpl::connectRegions(const vector<PositionInfo> &lasts,
                                       const vector<PositionInfo> &firsts) {
    for (const auto &last : lasts) {
        checkEmbeddedEndAnchor(last, firsts);
        connectSuccessors(last, firsts);
    }
}

static
void filterEdges(const GlushkovBuildStateImpl &bs, const PositionInfo &from,
                 vector<PositionInfo> &tolist) {
    if (from.pos == bs.startDotstarState) {
        // If we're connecting from start-dotstar, remove all caret flavoured
        // positions.
        CheckPositionFlags check(POS_FLAG_NOFLOAT);
        tolist.erase(remove_if(tolist.begin(), tolist.end(), check),
                     tolist.end());
        if (from.flags & POS_FLAG_NOFLOAT) {
            tolist.clear();
        }
    } else if (from.pos == bs.startState) {
        // If we're connecting from start, we should remove any epsilons that
        // aren't caret flavoured.
        CheckUnflaggedEpsilon check;
        tolist.erase(remove_if(tolist.begin(), tolist.end(), check),
                     tolist.end());
        CheckPositionFlags check2(POS_FLAG_MUST_FLOAT | POS_FLAG_NOFLOAT);
        tolist.erase(remove_if(tolist.begin(), tolist.end(), check2),
                     tolist.end());
    }

    if (bs.builder.getAssertFlag(from.pos) & POS_FLAG_MULTILINE_START) {
        // If we have a (mildly boneheaded) pattern like /^$/m, we're right up
        // against the edge of what we can do without true assertion support.
        // Here we have an evil hack to prevent us plugging the \n generated by
        // the caret right into acceptEod (which is in the firsts of the
        // dollar).
        /* This is due to the 'interesting quirk' that multiline ^ does not
         * not match a newline at the end of buffer. */
        DEBUG_PRINTF("multiline start - no eod\n");
        tolist.erase(remove(tolist.begin(), tolist.end(), bs.acceptEodState),
                     tolist.end());
    }
}

static
Position makeNewlineAssertPos(GlushkovBuildState &bs) {
    NFABuilder &builder = bs.getBuilder();
    Position newline = builder.makePositions(1);
    builder.addCharReach(newline, CharReach('\n'));
    builder.setAssertFlag(newline, POS_FLAG_FIDDLE_ACCEPT);
    builder.setNodeReportID(newline, -1);
    return newline;
}

static
void generateAccepts(GlushkovBuildStateImpl &bs, const PositionInfo &from,
                     vector<PositionInfo> *tolist) {
    NFABuilder &builder = bs.getBuilder();
    u32 flags = from.flags;

    bool require_eod = flags & POS_FLAG_WIRE_EOD;
    bool require_nl_eod = flags & POS_FLAG_WIRE_NL_EOD
                       && !(flags & POS_FLAG_NO_NL_EOD);
    bool require_nl_accept = (flags & POS_FLAG_WIRE_NL_ACCEPT)
                           && !(flags & POS_FLAG_NO_NL_ACCEPT);

    bool require_accept = !(flags & POS_FLAG_ONLY_ENDS);

    if (require_eod) {
        tolist->push_back(bs.acceptEodState);
    }

    if (require_nl_accept) {
        if (bs.acceptNlState == GlushkovBuildState::POS_UNINITIALIZED) {
            Position newline = makeNewlineAssertPos(bs);
            bs.addSuccessor(newline, builder.getAccept());
            bs.acceptNlState = newline;
        }
        tolist->push_back(bs.acceptNlState);
    }

    if (require_nl_eod) {
        if (bs.acceptNlEodState == GlushkovBuildState::POS_UNINITIALIZED) {
            Position newline = makeNewlineAssertPos(bs);
            bs.addSuccessor(newline, builder.getAcceptEOD());
            bs.acceptNlEodState = newline;
        }
        tolist->push_back(bs.acceptNlEodState);
    }

    if (require_accept) {
        tolist->push_back(bs.acceptState);
    }
}

void GlushkovBuildStateImpl::connectAccepts(const vector<PositionInfo> &lasts) {
    for (const auto &last : lasts) {
        vector<PositionInfo> accepts;
        generateAccepts(*this, last, &accepts);
        connectSuccessors(last, accepts);
    }
}

#if defined(DEBUG) || defined(DUMP_SUPPORT)

static UNUSED
string dumpCaptures(const PositionInfo &p) {
    ostringstream oss;

    if (p.flags & POS_FLAG_NOFLOAT) {
        oss << "<nofloat>";
    }
    if (p.flags & POS_FLAG_MUST_FLOAT) {
        oss << "<must_float>";
    }
    if (p.flags & POS_FLAG_FIDDLE_ACCEPT) {
        oss << "<fiddle_accept>";
    }
    if (p.flags & POS_FLAG_ONLY_ENDS) {
        oss << "<only_ends>";
    }
    if (p.flags & POS_FLAG_NO_NL_EOD) {
        oss << "<no_nl_eod>";
    }
    if (p.flags & POS_FLAG_NO_NL_ACCEPT) {
        oss << "<no_nl_acc>";
    }

    return oss.str();
}

#endif // DEBUG || DUMP_SUPPORT

void GlushkovBuildStateImpl::connectSuccessors(const PositionInfo &from,
                                               vector<PositionInfo> tolist) {
    /* note: tolist maybe modified for our own internal use -> not a reference */
    assert(from.pos != POS_EPSILON);
    assert(from.pos != POS_UNINITIALIZED);
    assert(find(tolist.begin(), tolist.end(), POS_UNINITIALIZED)
           == tolist.end());

    DEBUG_PRINTF("FROM = %u%s TO = %s\n", from.pos, dumpCaptures(from).c_str(),
                 dumpPositions(tolist.begin(), tolist.end()).c_str());

    /* prevent creation of edges with invalid assertions */
    filterEdges(*this, from, tolist);

    if (from.flags & POS_FLAG_FIDDLE_ACCEPT) {
        auto accept = find(tolist.begin(), tolist.end(), acceptState);
        if (accept != tolist.end()) {
            DEBUG_PRINTF("accept through -1 offset-adjusting dot\n");
            Position fakedot = builder.makePositions(1);
            builder.addCharReach(fakedot, CharReach(0x00, 0xff));
            builder.setNodeReportID(fakedot, -1);
            addSuccessor(fakedot, acceptState);
            *accept = fakedot;
        } else {
            // We might lead to accept via an assertion vertex, so we add the
            // offset adj to this vertex itself. Used for cases like /^\B/m,
            // which should match only at 0 for '\n'.
            builder.setNodeReportID(from.pos, -1);
        }

        assert(find(tolist.begin(), tolist.end(), acceptState) == tolist.end());
    }

    auto &succ = successors[from.pos];

    DEBUG_PRINTF("connect %u -> %s\n", from.pos,
                 dumpPositions(tolist.begin(), tolist.end()).c_str());
    DEBUG_PRINTF("%u curr succ: %s\n", from.pos,
                 dumpPositions(begin(succ), end(succ)).c_str());

    for (const auto &to : tolist) {
        if (to.pos != POS_EPSILON) {
            succ.insert(to);
        }
    }

    DEBUG_PRINTF("%u succ: %s\n", from.pos,
                 dumpPositions(begin(succ), end(succ)).c_str());
}

void GlushkovBuildStateImpl::addSuccessor(Position from, Position to) {
    DEBUG_PRINTF("connect %u -> %u\n", from, to);
    assert(from != POS_EPSILON && from != POS_UNINITIALIZED);
    assert(to != POS_EPSILON && to != POS_UNINITIALIZED);

    auto &succ = successors[from];
    succ.insert(to);

    DEBUG_PRINTF("%u succ: %s\n", from,
                 dumpPositions(begin(succ), end(succ)).c_str());
}

void GlushkovBuildStateImpl::cloneFollowSet(Position first, Position last,
                                            unsigned offset) {
    assert(first <= last);

    // Clone vertex properties (reachability, etc)
    builder.cloneRegion(first, last, offset);

    /* Clone the successors of all the positions between first and last
     * inclusive, producing a new set of positions starting at (first +
     * offset). */
    for (Position i = first; i <= last; i++) {
        // This should be a new position.
        assert(successors[i + offset].empty());

        for (const PositionInfo &to : successors[i]) {
            if (to.pos >= first && to.pos <= last) {
                PositionInfo clone(to);
                clone.pos += offset;
                DEBUG_PRINTF("clone: %u -> %u\n", i + offset, clone.pos);
                successors[i + offset].insert(clone);
            } else {
                // There shouldn't be any stray edges leading out of this
                // region!
                assert(0);
            }
        }
    }
}

void GlushkovBuildStateImpl::buildEdge(Position from, const PositionInfo &to) {
    // Guard against embedded anchors
    if (to == startState) {
        /* can make it through the parse tree */
        throw ParseError("Embedded start anchors not supported.");
    }

    assert(to.pos != POS_UNINITIALIZED);
    assert(to.pos != POS_EPSILON);

    if (builder.hasEdge(from, to.pos)) {
        return;
    }

    builder.addEdge(from, to.pos);
}

void GlushkovBuildStateImpl::buildEdges() {
    // Create all the edges and track which vertices are asserts which need to
    // be removed later.
    for (const auto &m : successors) {
        const Position from = m.first;
        for (const auto &to : m.second) {
            buildEdge(from, to);
        }
    }
}

// Construct a usable GlushkovBuildState for the outside world.
unique_ptr<GlushkovBuildState> makeGlushkovBuildState(NFABuilder &b,
                                                      bool prefilter) {
    return ue2::make_unique<GlushkovBuildStateImpl>(b, prefilter);
}

// free functions for utility use

/** \brief Eliminate lower-priority duplicate PositionInfo entries.
 *
 * Scans through a list of positions and retains only the highest priority
 * version of a given (position, flags) entry. */
void cleanupPositions(vector<PositionInfo> &a) {
    ue2_unordered_set<pair<Position, int>> seen;

    vector<PositionInfo> out;
    out.reserve(a.size()); // output should be close to input in size.

    for (const auto &p : a) {
        if (seen.emplace(p.pos, p.flags).second) {
            out.push_back(p); // first encounter
        }
    }

    DEBUG_PRINTF("in %zu; out %zu\n", a.size(), out.size());
    a.swap(out);
}

static
vector<PositionInfo>::iterator
replaceElemWithSequence(vector<PositionInfo> &dest,
                        vector<PositionInfo>::iterator &victim,
                        const vector<PositionInfo> &replacement) {
    auto past = dest.erase(victim);
    size_t d = distance(dest.begin(), past) + replacement.size();
    dest.insert(past, replacement.begin(), replacement.end());
    /* recalc past as iterator may have been invalidated */
    return dest.begin() + d;
}

/** \brief Replace all epsilons with the given positions.
 *
 * Replace epsilons in a firsts list with another given firsts list. Note: the
 * firsts lists must come from disjoint sets of components. If no epsilons are
 * in the first firsts list the source is appended to the end.
 */
void replaceEpsilons(vector<PositionInfo> &target,
                     const vector<PositionInfo> &source) {
    auto found =
        find(target.begin(), target.end(), GlushkovBuildState::POS_EPSILON);

    if (found == target.end()) {
        // no epsilons to replace, push on to the end
        target.insert(target.end(), source.begin(), source.end());
        return;
    }

    while (found != target.end()) {
        checkEmbeddedEndAnchor(*found, source);

        // replace this epsilon with a copy of source with the same flags
        vector<PositionInfo> newsource(source);
        for (auto &pos : newsource) {
            pos.flags |= found->flags;
        }

        found = replaceElemWithSequence(target, found, newsource);
        // find the next epsilon
        found = find(found, target.end(), GlushkovBuildState::POS_EPSILON);
    }

    cleanupPositions(target);
}

#ifdef DUMP_SUPPORT

void dump(ostream &os, const PositionInfo &p) {
    if (p.pos == GlushkovBuildState::POS_EPSILON) {
        os << "epsilon";
    } else {
        os << p.pos;
    }

    os << dumpCaptures(p);
}

#endif // DUMP_SUPPORT

} // namespace ue2
