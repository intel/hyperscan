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
 * \brief Data types used to represent depth quantities.
 */

#ifndef DEPTH_H
#define DEPTH_H

#include "ue2common.h"
#include "util/hash.h"
#include "util/operators.h"

#ifdef DUMP_SUPPORT
#include <string>
#endif

namespace ue2 {

/**
 * \brief Exception thrown if a depth operation overflows.
 */
struct DepthOverflowError {};

/**
 * \brief Type used to represent depth information; value is either a count,
 * or the special values "infinity" and "unreachable".
 */
class depth : totally_ordered<depth> {
public:
    /** \brief The default depth is special value "unreachable". */
    depth() = default;

    explicit depth(u32 v) : val(v) {
        if (v > max_value()) {
            DEBUG_PRINTF("depth %u too large to represent!\n", v);
            throw DepthOverflowError();
        }
    }

    static depth unreachable() {
        depth d;
        d.val = val_unreachable;
        return d;
    }

    static depth infinity() {
        depth d;
        d.val = val_infinity;
        return d;
    }

    /** \brief Returns the max finite value representable as a depth. */
    static constexpr u32 max_value() { return val_infinity - 1; }

    bool is_finite() const { return val < val_infinity; }
    bool is_infinite() const { return val == val_infinity; }
    bool is_unreachable() const { return val == val_unreachable; }
    bool is_reachable() const { return !is_unreachable(); }

    /** \brief Convert a finite depth to an integer. */
    operator u32() const {
        if (!is_finite()) {
            throw DepthOverflowError();
        }
        return val;
    }

    bool operator<(const depth &d) const { return val < d.val; }
    bool operator==(const depth &d) const { return val == d.val; }

    // The following comparison operators exist for use against integer types
    // that are bigger than what we can safely convert to depth (such as those
    // in extparam).

    bool operator<(u64a d) const {
        if (!is_finite()) {
            return false;
        }
        return val < d;
    }
    bool operator<=(u64a d) const {
        if (!is_finite()) {
            return false;
        }
        return val <= d;
    }
    bool operator==(u64a d) const {
        if (!is_finite()) {
            return false;
        }
        return val == d;
    }
    bool operator>(u64a d) const { return !(*this <= d); }
    bool operator>=(u64a d) const { return !(*this < d); }
    bool operator!=(u64a d) const { return !(*this == d); }

    depth operator+(const depth &d) const {
        if (is_unreachable() || d.is_unreachable()) {
            return unreachable();
        }
        if (is_infinite() || d.is_infinite()) {
            return infinity();
        }

        u64a rv = val + d.val;
        if (rv >= val_infinity) {
            DEBUG_PRINTF("depth %llu too large to represent!\n", rv);
            throw DepthOverflowError();
        }

        return depth((u32)rv);
    }

    depth &operator+=(const depth &d) {
        depth rv = *this + d;
        *this = rv;
        return *this;
    }

    depth operator-(const depth &d) const {
        if (!d.is_finite()) {
            throw DepthOverflowError();
        }

        if (is_unreachable()) {
            return unreachable();
        }
        if (is_infinite()) {
            return infinity();
        }

        if (val < d.val) {
            throw DepthOverflowError();
        }

        u32 rv = val - d.val;
        return depth(rv);
    }

    depth &operator-=(const depth &d) {
        depth rv = *this - d;
        *this = rv;
        return *this;
    }

    depth operator+(s32 d) const {
        if (is_unreachable()) {
            return unreachable();
        }
        if (is_infinite()) {
            return infinity();
        }

        s64a rv = val + d;
        if (rv < 0 || (u64a)rv >= val_infinity) {
            DEBUG_PRINTF("depth %lld too large to represent!\n", rv);
            throw DepthOverflowError();
        }

        return depth((u32)rv);
    }

    depth operator+=(s32 d) {
        depth rv = *this + d;
        *this = rv;
        return *this;
    }

    depth operator-(s32 d) const {
        if (is_unreachable()) {
            return unreachable();
        }
        if (is_infinite()) {
            return infinity();
        }

        s64a rv = val - d;
        if (rv < 0 || (u64a)rv >= val_infinity) {
            DEBUG_PRINTF("depth %lld too large to represent!\n", rv);
            throw DepthOverflowError();
        }

        return depth((u32)rv);
    }

    depth operator-=(s32 d) {
        depth rv = *this - d;
        *this = rv;
        return *this;
    }

#ifdef DUMP_SUPPORT
    /** \brief Render as a string, useful for debugging. */
    std::string str() const;
#endif

    size_t hash() const {
        return val;
    }

private:
    static constexpr u32 val_infinity = (1u << 31) - 1;
    static constexpr u32 val_unreachable = 1u << 31;

    u32 val = val_unreachable;
};

/**
 * \brief Encapsulates a min/max pair.
 */
struct DepthMinMax : totally_ordered<DepthMinMax> {
    depth min{depth::infinity()};
    depth max{0};

    DepthMinMax() = default;
    DepthMinMax(const depth &mn, const depth &mx) : min(mn), max(mx) {}

    bool operator<(const DepthMinMax &b) const {
        if (min != b.min) {
            return min < b.min;
        }
        return max < b.max;
    }

    bool operator==(const DepthMinMax &b) const {
        return min == b.min && max == b.max;
    }

#ifdef DUMP_SUPPORT
    /** \brief Render as a string, useful for debugging. */
    std::string str() const;
#endif

};

/**
 * \brief Merge two DepthMinMax values together to produce their union.
 */
DepthMinMax unionDepthMinMax(const DepthMinMax &a, const DepthMinMax &b);

} // namespace ue2

namespace std {

template<>
struct hash<ue2::depth> {
    size_t operator()(const ue2::depth &d) const {
        return d.hash();
    }
};

template<>
struct hash<ue2::DepthMinMax> {
    size_t operator()(const ue2::DepthMinMax &d) const {
        return hash_all(d.min, d.max);
    }
};

} // namespace

#endif // DEPTH_H
