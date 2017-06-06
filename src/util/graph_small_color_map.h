/*
 * Copyright (c) 2017-2018, Intel Corporation
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
 * \brief Small Color Map: implements a property map designed to represent
 * colors using minimal memory (two bits per index).
 *
 * This is based on the Boost BGL two_bit_color_map, but provides some extra
 * functionality (such as a fill operation).
 */

#ifndef GRAPH_SMALL_COLOR_MAP_H
#define GRAPH_SMALL_COLOR_MAP_H

#include "ue2common.h"

#include <cstring>
#include <memory>
#include <vector>

namespace ue2 {

enum class small_color : u8 {
    white = 0,
    gray = 1,
    black = 2
    // Note: we have room for one more colour.
};

} // namespace ue2

namespace boost {

/** \brief Specialisation of boost::color_traits for small_color. */
template<>
struct color_traits<ue2::small_color> {
    static ue2::small_color white() { return ue2::small_color::white; }
    static ue2::small_color gray() { return ue2::small_color::gray; }
    static ue2::small_color black() { return ue2::small_color::black; }
};

} // namespace boost

namespace ue2 {

static constexpr u8 fill_lut[] = {
    0,    // white
    0x55, // gray
    0xaa, // black
};

/**
 * \brief Small Color Map: implements a property map designed to represent
 * colors using minimal memory (two bits per index).
 *
 * If your graph type provides an index map in get(vertex_index, g), you can
 * use make_small_color_map() to construct this.
 */
template<typename IndexMap>
class small_color_map {
    size_t n;
    IndexMap index_map;

    // This class is passed by value into (potentially recursive) BGL
    // algorithms, so we use a shared_ptr to keep the copy lightweight and
    // ensure that data is correctly destroyed.
    std::shared_ptr<std::vector<u8>> data;

    static constexpr size_t bit_size = 2;
    static constexpr size_t entries_per_byte = (sizeof(u8) * 8) / bit_size;
    static constexpr u8 bit_mask = (1U << bit_size) - 1;

public:
    using key_type = typename boost::property_traits<IndexMap>::key_type;
    using value_type = small_color;
    using reference = small_color;
    using category = boost::read_write_property_map_tag;

    small_color_map(size_t n_in, const IndexMap &index_map_in)
        : n(n_in), index_map(index_map_in) {
        size_t num_bytes = (n + entries_per_byte - 1) / entries_per_byte;
        data = std::make_shared<std::vector<unsigned char>>(num_bytes);
        fill(small_color::white);
    }

    void fill(small_color color) {
        assert(static_cast<u8>(color) < sizeof(fill_lut));
        u8 val = fill_lut[static_cast<u8>(color)];
        std::memset(data->data(), val, data->size());
    }

    size_t count(small_color color) const {
        assert(static_cast<u8>(color) < sizeof(fill_lut));
        size_t num = 0;
        for (size_t i = 0; i < n; i++) {
            size_t byte = i / entries_per_byte;
            assert(byte < data->size());
            size_t bit = (i % entries_per_byte) * bit_size;
            u8 val = ((*data)[byte] >> bit) & bit_mask;
            if (static_cast<small_color>(val) == color) {
                num++;
            }
        }
        return num;
    }

    small_color get_impl(key_type key) const {
        auto i = get(index_map, key);
        assert(i < n);
        size_t byte = i / entries_per_byte;
        assert(byte < data->size());
        size_t bit = (i % entries_per_byte) * bit_size;
        u8 val = ((*data)[byte] >> bit) & bit_mask;
        return static_cast<small_color>(val);
    }

    void put_impl(key_type key, small_color color) {
        auto i = get(index_map, key);
        assert(i < n);
        size_t byte = i / entries_per_byte;
        assert(byte < data->size());
        size_t bit = (i % entries_per_byte) * bit_size;
        auto &block = (*data)[byte];
        u8 val = static_cast<u8>(color);
        block = (block & ~(bit_mask << bit)) | (val << bit);
    }
};

template<typename IndexMap>
small_color get(const small_color_map<IndexMap> &color_map,
                typename boost::property_traits<IndexMap>::key_type key) {
    return color_map.get_impl(key);
}

template<typename IndexMap>
void put(small_color_map<IndexMap> &color_map,
         typename boost::property_traits<IndexMap>::key_type key,
         small_color val) {
    color_map.put_impl(key, val);
}

template<typename Graph>
auto make_small_color_map(const Graph &g)
    -> small_color_map<decltype(get(vertex_index, g))> {
    return small_color_map<decltype(get(vertex_index, g))>(
        num_vertices(g), get(vertex_index, g));
}

} // namespace ue2

#endif // GRAPH_SMALL_COLOR_MAP_H
