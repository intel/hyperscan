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

#ifndef PARTITIONED_SET_H
#define PARTITIONED_SET_H

#include "container.h"
#include "noncopyable.h"
#include "flat_containers.h"
#include "ue2common.h"

#include <algorithm>
#include <vector>

#include <boost/dynamic_bitset.hpp>

namespace ue2 {

static constexpr size_t INVALID_SUBSET = ~(size_t)0;

/**
 * partition_set represents a partitioning of a set of integers [0, n) into
 * disjoint non-empty subsets.
 *
 * The subsets themselves are also indexed by integers.
 *
 * The underlying integer type for the set members is parameterized.
 */

template<typename T>
class partitioned_set : noncopyable {
public:
    class subset {
    public:
        typedef typename std::vector<T>::const_iterator const_iterator;

        size_t size() const {
            assert(members.size());
            return members.size();
        }

        const_iterator begin() const {
            return members.begin();
        }

        const_iterator end() const {
            return members.end();
        }

    private:
        std::vector<T> members; /**< sorted members of the subset */

        friend class partitioned_set;
    };

    /** returns the number of subsets in the partition */
    size_t size() const { return subsets.size(); }

    /** returns the subset with the given index */
    const subset &operator[](size_t subset_index) const {
        assert(subset_index < size());
        return subsets[subset_index];
    }

    /**
     * Splits the subset with the given subset_index based on whether its
     * members are also members of the splitter set.
     *
     * The smaller of the intersection and difference is placed into a new
     * subset, the index of which is returned. The larger part remains with the
     * subset index.
     *
     * If the set was not split (due to there being no overlap with splitter or
     * being a complete subset), INVALID_SUBSET is returned.
     */
    size_t split(size_t subset_index, const flat_set<T> &splitter) {
        assert(!splitter.empty());
        if (splitter.empty()) {
            return INVALID_SUBSET;
        }

        subset &orig = subsets[subset_index];

        assert(orig.size());

        split_temp_diff.clear();
        split_temp_inter.clear();

        auto sp_it = splitter.begin();
        auto sp_e = splitter.end();

        /* subset members are always in sorted order. */
        assert(std::is_sorted(orig.members.begin(), orig.members.end()));

        if (orig.members.back() < *sp_it) {
            /* first splitter is greater than all our members */
            return INVALID_SUBSET;
        }

        if (orig.members.front() > *splitter.rbegin()) {
            /* last splitter is less than all our members */
            return INVALID_SUBSET;
        }

        for (auto it = orig.members.begin(); it != orig.members.end(); ++it) {
            const auto &member = *it;
            assert(member < member_to_subset.size());

            sp_it = std::lower_bound(sp_it, sp_e, member);
            if (sp_it == sp_e) {
                split_temp_diff.insert(split_temp_diff.end(), it,
                                       orig.members.end());
                break;
            }

            if (*sp_it > member) {
                split_temp_diff.push_back(member);
            } else {
                split_temp_inter.push_back(member);
            }
        }

        assert(split_temp_diff.size() + split_temp_inter.size() == orig.size());

        if (split_temp_inter.empty()) {
            assert(split_temp_diff == orig.members);
            return INVALID_SUBSET;
        }

        if (split_temp_diff.empty()) {
            assert(split_temp_inter == orig.members);
            return INVALID_SUBSET;
        }

        assert(MIN(split_temp_inter[0], split_temp_diff[0]) == orig.members[0]);

        /* work out which is the bigger half */
        std::vector<T> *big;
        std::vector<T> *small;
        if (split_temp_diff.size() > split_temp_inter.size()) {
            big = &split_temp_diff;
            small = &split_temp_inter;
        } else {
            big = &split_temp_inter;
            small = &split_temp_diff;
        }

        /* larger subset replaces the input subset */
        std::vector<T> temp_i;
        insert(&temp_i, temp_i.end(), *big);
        orig.members.swap(temp_i);

        /* smaller subset is placed in the new subset  */
        size_t new_index = subsets.size();
        subsets.push_back(subset());
        insert(&subsets.back().members, subsets.back().members.end(), *small);

        for (const auto &e : *small) {
            member_to_subset[e] = new_index;
        }

        return new_index;
    }

    /**
     * Returns all subsets which have a member in keys.
     */
    void find_overlapping(const flat_set<T> &keys,
                          std::vector<size_t> *containing) const {
        boost::dynamic_bitset<> seen(subsets.size()); // all zero by default.

        for (const auto &key : keys) {
            assert(key < member_to_subset.size());
            size_t sub = member_to_subset[key];
            assert(sub < subsets.size());
            seen.set(sub);
        }

        for (size_t i = seen.find_first(); i != seen.npos;
             i = seen.find_next(i)) {
            containing->push_back(i);
        }
    }

    /**
     * Creates a partitioned set containing elements [0, state_to_subset.size() )
     *
     * The initial subset that an element belongs to is given by the
     * corresponding entry in state_to_subset. The subsets should be identified
     * by a dense range of indices starting from 0.
     */
    explicit partitioned_set(const std::vector<size_t> &state_to_subset) {
        assert(!state_to_subset.empty());

        subsets.reserve(state_to_subset.size());
        member_to_subset.resize(state_to_subset.size());

        split_temp_inter.reserve(state_to_subset.size());
        split_temp_diff.reserve(state_to_subset.size());

        size_t subset_count = 0;
        for (const auto &sub : state_to_subset) {
            assert(sub != INVALID_SUBSET);
            ENSURE_AT_LEAST(&subset_count, sub + 1);
        }
        assert(subset_count <= state_to_subset.size());

        subsets.resize(subset_count);
        for (size_t i = 0; i < state_to_subset.size(); i++) {
             /* ensure that our underlying type is big enough to hold all our
              * set members */
            assert(i == (size_t)(T)i);

            size_t sub = state_to_subset[i];
            assert(sub < subsets.size());

            member_to_subset[i] = sub;
            subsets[sub].members.push_back(i);
        }

        /* none of the subsets should be empty */
        assert(std::all_of(subsets.begin(), subsets.end(),
                           [](const subset &sub){ return sub.size() > 0; }));
    }

private:
    std::vector<size_t> member_to_subset;
    std::vector<subset> subsets;

    std::vector<T> split_temp_inter; /**< used internally by split to hold the
                                      * intersection. */
    std::vector<T> split_temp_diff; /**< used internally by split to hold the
                                     * set difference. */
};

} // namespace

#endif
