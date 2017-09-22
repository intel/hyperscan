/*
 * Copyright (c) 2017, Intel Corporation
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

/**
 * \file
 * \brief Hashing utility functions.
 */

#ifndef UTIL_HASH_DYNAMIC_BITSET_H
#define UTIL_HASH_DYNAMIC_BITSET_H

#include "hash.h"

#include <boost/dynamic_bitset.hpp>

#include <iterator>

namespace ue2 {

/**
 * \brief An output iterator which calculates the combined hash of all elements
 * written to it.
 *
 * The location to output the hash is provided to the constructor and should
 * already be zero initialised.
 */
struct hash_output_it {
    using value_type = void;
    using difference_type = ptrdiff_t;
    using pointer = void *;
    using reference = void;
    using iterator_category = std::output_iterator_tag;

    hash_output_it(size_t *hash_out = nullptr) : out(hash_out) {}
    hash_output_it &operator++() {
        return *this;
    }
    hash_output_it &operator++(int) {
        return *this;
    }

    struct deref_proxy {
        deref_proxy(size_t *hash_out) : out(hash_out) {}

        template<typename T>
        void operator=(const T &val) const {
            hash_combine(*out, val);
        }

    private:
        size_t *out; /* output location of the owning iterator */
    };

    deref_proxy operator*() { return {out}; }

private:
    size_t *out; /* location to output the hashes to */
};

/* Function object for hashing a dynamic bitset */
struct hash_dynamic_bitset {
    size_t operator()(const boost::dynamic_bitset<> &bs) const {
        size_t rv = 0;
        to_block_range(bs, hash_output_it(&rv));
        return rv;
    }
};

} // namespace ue2

#endif
