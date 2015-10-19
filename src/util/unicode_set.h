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

#ifndef UNICODE_SET
#define UNICODE_SET

#include "unicode_def.h"

#include <boost/icl/interval_set.hpp>

namespace ue2 {

class CodePointSet {
public:
    typedef boost::icl::closed_interval<unichar> interval;
    typedef boost::icl::interval_set<unichar, std::less, interval> implT;
    typedef implT::const_iterator const_iterator;

    CodePointSet(void) {}

    explicit CodePointSet(const interval &st) : impl(st) {}

    bool none(void) const {
        return impl.empty();
    }

    void set(unichar c) {
        assert(c <= MAX_UNICODE);
        impl.insert(c);
    }

    void unset(unichar c) {
        assert(c <= MAX_UNICODE);
        impl.subtract(c);
    }

    void setRange(unichar from, unichar to) { /* inclusive */
        assert(from <= to);
        assert(to <= MAX_UNICODE);
        impl.insert(interval(from, to));
    }

    void unsetRange(unichar from, unichar to) { /* inclusive */
        assert(from <= to);
        assert(to <= MAX_UNICODE);
        impl.subtract(interval(from, to));
    }

    void flip(void) {
        impl = implT(interval(0, MAX_UNICODE)) - impl;
    }

    void operator|=(const CodePointSet &a) {
        impl += a.impl;
    }

    const_iterator begin(void) const {
        return impl.begin();
    }

    const_iterator end(void) const {
        return impl.end();
    }

    size_t count(void) const {
        return cardinality(impl);
    }

    CodePointSet operator~(void) const {
        CodePointSet rv = *this;
        rv.flip();
        return rv;
    }

    bool operator==(const CodePointSet &a) const {
        return is_element_equal(impl, a.impl);
    }

    bool operator!=(const CodePointSet &a) const {
        return !is_element_equal(impl, a.impl);
    }

    bool isSubset(const CodePointSet &a) const {
        // Check that adding an interval set has no effect
        return ((impl + a.impl) == impl);
    }

    void operator-=(const CodePointSet &a) {
        impl -= a.impl;
    }

    /* finds the nth set codepoint, returns INVALID_UNICODE on failure */
    unichar at(size_t pos) const {
        for (const_iterator i = begin(), e = end(); i != e; ++i) {
            size_t int_count = cardinality(*i);
            if (int_count <= pos) {
                /* not in this interval, check next */
                pos -= int_count;
                continue;
            } else {
                return lower(*i) + pos;
            }
        }

        return INVALID_UNICODE;
    }

    void swap(CodePointSet &other) { impl.swap(other.impl); }

private:
    implT impl;
};

} // namespace ue2

#endif
