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

#include "sidecar_compile.h"
#include "sidecar_internal.h"
#include "ue2common.h"
#include "nfa/shufticompile.h"
#include "util/alloc.h"
#include "util/charreach.h"
#include "util/simd_utils.h"
#include "util/verify_types.h"

#include <array>
#include <map>
#include <set>

#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;

namespace ue2 {

static
void prune(array<set<u32>, N_CHARS> &by_char, u32 p,
           map<CharReach, set<u32>> *impl_classes) {
    CharReach cr;
    assert(!by_char[p].empty());

    for (u32 i = 0; i < N_CHARS; i++) {
        if (by_char[i] == by_char[p]) {
            cr.set(i);
        }
    }

    assert(impl_classes->find(cr) == impl_classes->end());
    (*impl_classes)[cr] = by_char[p];

    for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
        by_char[i].clear();
    }

}

static really_inline
void set_bit(u8 *a, size_t i) {
    assert(i < 8);
    *a |= 1U << i;
}

static really_inline
void set_bit(u32 *a, size_t i) {
    assert(i < 32);
    *a |= 1U << i;
}

static really_inline
void set_bit(u64a *a, size_t i) {
    assert(i < 64);
    *a |= 1ULL << i;
}

static really_inline
void set_bit(m128 *a, size_t i) {
    setbit128(a, i);
}

static really_inline
void set_bit(m256 *a, size_t i) {
    setbit256(a, i);
}

template<typename s>
static really_inline
void flip(s *v) {
    *v = ~*v;
}

static really_inline
void flip(m128 *v) {
    *v = not128(*v);
}

static really_inline
void flip(m256 *v) {
    *v = not256(*v);
}

template<typename s>
static really_inline
void or_into_mask(s *a, const s b) {
    *a |= b;
}

static really_inline
void or_into_mask(m128 *a, const m128 b) {
    *a = or128(*a, b);
}

static really_inline
void or_into_mask(m256 *a, const m256 b) {
    *a = or256(*a, b);
}

template<u8 s_type> struct sidecar_traits { };
#define MAKE_TRAITS(type_id, base_type_in, mask_bits) \
    template<> struct sidecar_traits<type_id> {       \
        typedef base_type_in base_type;               \
        static const u32 bits = mask_bits;            \
        typedef sidecar_##mask_bits impl_type;        \
        typedef sidecar_enabled_##mask_bits enabled_type;  \
    };

MAKE_TRAITS(SIDECAR_8,   u8,   8)
MAKE_TRAITS(SIDECAR_32,  u32,  32)
MAKE_TRAITS(SIDECAR_64,  u64a, 64)
MAKE_TRAITS(SIDECAR_128, m128, 128)
MAKE_TRAITS(SIDECAR_256, m256, 256)

template<> struct sidecar_traits<SIDECAR_N> {
    typedef sidecar_N impl_type;
};

template<> struct sidecar_traits<SIDECAR_S> {
    typedef u8 base_type;
    typedef sidecar_S impl_type;
};

/* builds the main char reach table */
template <u8 s_type>
static
void populateTable(const map<CharReach, set<u32>> &impl_classes,
                   typename sidecar_traits<s_type>::impl_type *ns) {
    assert(impl_classes.size()
           <= sizeof(typename sidecar_traits<s_type>::base_type) * 8);

    u32 b = 0;
    for (const CharReach &cr : impl_classes | map_keys) {
        for (size_t i = cr.find_first(); i != cr.npos; i = cr.find_next(i)) {
            set_bit(&ns->reach[i], b);
        }
        b++;
    }

    for (u32 i = 0; i < N_CHARS; i++) {
        flip(&ns->reach[i]);
    }
}

/* builds the table controlling which bits in the mask to turn on for each
 * external id */
template <u8 s_type>
static
void populateIdMasks(const map<CharReach, set<u32>> &impl_classes,
                     typename sidecar_traits<s_type>::impl_type *ns) {
    typedef typename sidecar_traits<s_type>::base_type base;
    base *table = (base *)((char *)ns + sizeof(*ns));
    u32 b = 0;
    for (const set<u32> &id_list : impl_classes | map_values) {
        for (const u32 id : id_list) {
            set_bit(&table[id], b);
        }
        if (id_list.size() == 1) {
            set_bit(&ns->unshared_mask, b);
        }
        b++;
    }
}

/* builds the lists of ids to report for each set bit */
template <u8 s_type>
static
void populateMaskInfo(const map<CharReach, set<u32>> &impl_classes,
                      u32 num_ext_classes,
                      typename sidecar_traits<s_type>::impl_type *ns,
                      sidecar_id_offset *mask_info) {
    typedef typename sidecar_traits<s_type>::base_type base;

    u32 *curr_ptr = (u32 *)((char *)ns + sizeof(*ns)
                            + sizeof(base) * num_ext_classes);
    curr_ptr = ROUNDUP_PTR(curr_ptr, sizeof(u32));

    u32 b = 0;
    for (const set<u32> &id_list : impl_classes | map_values) {
        mask_info[b].first_offset = verify_u32((char *)curr_ptr - (char *)ns);
        mask_info[b].count = verify_u32(id_list.size());
        for (const u32 id : id_list) {
            *curr_ptr = id;
            curr_ptr++;
        }
        b++;
    }
}

static
size_t calcIdListSize(const map<CharReach, set<u32>> &impl_classes) {
    size_t id_count = 0;
    for (const auto &id_list : impl_classes | map_values) {
        id_count += id_list.size();
    }

    return id_count * sizeof(u32);
}

template<u8 s_type>
static
aligned_unique_ptr<sidecar> construct(const vector<CharReach> &ext_classes,
                   const map<CharReach, set<u32> > &impl_classes_in,
                   bool allow_collapse) {
    if (impl_classes_in.size() > sidecar_traits<s_type>::bits) {
        return nullptr;
    }

    map<CharReach, set<u32>> impl_classes_loc;
    const map<CharReach, set<u32>> *impl_classes;

    if (ext_classes.size() <= sidecar_traits<s_type>::bits) {
        /* we can directly map internal bits to external ids; no need for
         * indirection */
        for (u32 i = 0; i < ext_classes.size(); i++) {
            impl_classes_loc[ext_classes[i]].insert(i);
        }

        impl_classes = &impl_classes_loc;
    } else {
        /* TODO: spread classes out if possible */
        if (!allow_collapse) {
            return nullptr;
        }
        impl_classes = &impl_classes_in;
    }

    typedef typename sidecar_traits<s_type>::base_type base;
    typedef typename sidecar_traits<s_type>::impl_type impl;

    u32 id_count = verify_u32(ext_classes.size());
    size_t total_id_list_size = calcIdListSize(*impl_classes);
    size_t size = sizeof(impl) + id_count * sizeof(base); /* ids -> masks */
    size = ROUNDUP_N(size, sizeof(u32));
    size += total_id_list_size;
    DEBUG_PRINTF("allocated %zu\n", size);

    auto s = aligned_zmalloc_unique<sidecar>(size);
    assert(s); // otherwise we would have thrown std::bad_alloc
    impl *ns = (impl *)(s.get());

    ns->header.type = s_type;
    ns->header.size = size;
    ns->header.id_count = id_count;
    ns->header.mask_bit_count = verify_u32(impl_classes->size());

    populateTable<s_type>(*impl_classes, ns);
    populateIdMasks<s_type>(*impl_classes, ns);
    populateMaskInfo<s_type>(*impl_classes, id_count, ns, ns->id_list);

    return s;
}

static
bool isNoodable(const CharReach &cr) {
    return cr.count() == 1 || (cr.count() == 2 && cr.isBit5Insensitive());
}

template <>
aligned_unique_ptr<sidecar>
construct<SIDECAR_N>(const vector<CharReach> &ext_classes,
                     const map<CharReach, set<u32>> &impl_classes,
                     bool) {
    if (impl_classes.size() != 1 || !isNoodable(impl_classes.begin()->first)) {
        return nullptr;
    }

    const CharReach &cr = impl_classes.begin()->first;
    const set<u32> &reports = impl_classes.begin()->second;

    u32 id_count = verify_u32(ext_classes.size());
    size_t size = sizeof(sidecar_N) + sizeof(u32) * reports.size();
    DEBUG_PRINTF("allocated %zu\n", size);

    auto s = aligned_zmalloc_unique<sidecar>(size);
    assert(s); // otherwise we would have thrown std::bad_alloc
    sidecar_N *ns = (sidecar_N *)(s.get());

    ns->header.type = SIDECAR_N;
    ns->header.size = size;
    ns->header.id_count = id_count;
    ns->header.mask_bit_count = verify_u32(impl_classes.size());

    ns->c = cr.find_first();
    ns->nocase = cr.isBit5Insensitive();

    ns->report_count = verify_u32(reports.size());
    u32 *p = ns->reports;
    for (u32 report : reports) {
        *p = report;
    }

    return s;
}

static
void flipShuftiMask(m128 *a) {
    *a = not128(*a);
}

template <>
aligned_unique_ptr<sidecar>
construct<SIDECAR_S>(const vector<CharReach> &ext_classes,
                     const map<CharReach, set<u32>> &impl_classes,
                     bool) {
    u32 id_count = verify_u32(ext_classes.size());
    size_t total_id_list_size = calcIdListSize(impl_classes);
    size_t size = sizeof(sidecar_S)
        + id_count * sizeof(u8); /* ids -> masks */
    size = ROUNDUP_N(size, sizeof(u32));
    size += total_id_list_size;
    DEBUG_PRINTF("allocated %zu\n", size);

    auto s = aligned_zmalloc_unique<sidecar>(size);
    assert(s); // otherwise we would have thrown std::bad_alloc
    sidecar_S *ns = (sidecar_S *)(s.get());

    ns->header.type = SIDECAR_S;
    ns->header.size = size;
    ns->header.id_count = id_count;

    vector<const CharReach *> shuf_bit_to_impl;

    /* populate the shufti masks */
    u32 used_bits = 0;
    for (const CharReach &cr : impl_classes | map_keys) {
        m128 lo, hi;
        int bits = shuftiBuildMasks(cr, &lo, &hi);

        if (bits < 0 || used_bits + bits > 8) {
            return nullptr;
        }

        mergeShuftiMask(&ns->lo, lo, used_bits);
        mergeShuftiMask(&ns->hi, hi, used_bits);
        for (u32 i = used_bits; i < used_bits + bits; i++) {
            shuf_bit_to_impl.push_back(&cr);
        }
        used_bits += bits;
    }

    flipShuftiMask(&ns->lo); /* we are shift-or around here */
    flipShuftiMask(&ns->hi);
    ns->header.mask_bit_count = used_bits;

    /* populate the enable masks */
    u8 *table = (u8 *)((char *)ns + sizeof(*ns));
    u32 b = 0;
    for (const CharReach *cr : shuf_bit_to_impl) {
        const set<u32> &rep_set = impl_classes.find(*cr)->second;
        for (u32 report : rep_set) {
            set_bit(&table[report], b);
        }
        if (rep_set.size() == 1) {
            set_bit(&ns->unshared_mask, b);
        }
        b++;
    }

    /* populate the report id masks */
    sidecar_id_offset temp_id_list[8];
    populateMaskInfo<SIDECAR_S>(impl_classes, id_count, ns, temp_id_list);

    u32 i = 0, j = 0;
    auto iit = impl_classes.begin();
    while (i < shuf_bit_to_impl.size()) {
        assert(iit != impl_classes.end());
        if (shuf_bit_to_impl[i] == &iit->first) {
            ns->id_list[i] = temp_id_list[j];
            i++;
        } else {
            j++;
            ++iit;
        }
    }

    return s;
}

static
aligned_unique_ptr<sidecar>
constructWithHint(int hint, const vector<CharReach> &classes,
                  const map<CharReach, set<u32>> &impl_classes) {
    switch (hint) {
    case SIDECAR_8:
        return construct<SIDECAR_8>(classes, impl_classes, true);
    case SIDECAR_32:
        return construct<SIDECAR_32>(classes, impl_classes, true);
    case SIDECAR_64:
        return construct<SIDECAR_64>(classes, impl_classes, true);
    case SIDECAR_128:
        return construct<SIDECAR_128>(classes, impl_classes, true);
    case SIDECAR_256:
        return construct<SIDECAR_256>(classes, impl_classes, true);
    case SIDECAR_N:
        return construct<SIDECAR_N>(classes, impl_classes, true);
    case SIDECAR_S:
        return construct<SIDECAR_S>(classes, impl_classes, true);
    default:
        DEBUG_PRINTF("derp\n");
        assert(0);
        return nullptr;
    }
}

aligned_unique_ptr<sidecar> sidecarCompile(const vector<CharReach> &classes,
                                           int hint) {
    array<set<u32>, N_CHARS> by_char;

    for (u32 i = 0; i < classes.size(); i++) {
        const CharReach &cr = classes[i];
        for (size_t j = cr.find_first(); j != cr.npos; j = cr.find_next(j)) {
            by_char[j].insert(i);
        }
    }

    map<CharReach, set<u32>> impl_classes;

    bool changed;
    do {
        changed = false;
        u32 smallest = N_CHARS;
        for (u32 i = 0; i < N_CHARS; i++) {
            if (by_char[i].empty()) {
                continue;
            }

            if (by_char[i].size() == 1) {
                prune(by_char, i, &impl_classes);
                changed = true;
            } else if (smallest == N_CHARS ||
                       by_char[i].size() < by_char[smallest].size()) {
                smallest = i;
            }
        }

        if (!changed && smallest != N_CHARS) {
            prune(by_char, smallest, &impl_classes);
            changed = true;
        }
    } while (changed);

    DEBUG_PRINTF("matching %zu classes; %zu impl classes\n", classes.size(),
                 impl_classes.size());
    assert(impl_classes.size() <= N_CHARS);

    if (hint != SIDECAR_NO_HINT) {
        return constructWithHint(hint, classes, impl_classes);
    }

    aligned_unique_ptr<sidecar> (*facts[])(const vector<CharReach> &,
                        const map<CharReach, set<u32> > &, bool) = {
        construct<SIDECAR_N>,
        // construct<SIDECAR_S>, TODO: first offset stuff for S
        construct<SIDECAR_8>,
        construct<SIDECAR_32>,
        construct<SIDECAR_64>,
        construct<SIDECAR_128>,
        construct<SIDECAR_256>
    };

    for (u32 i = 0; i < ARRAY_LENGTH(facts); i++) {
        auto sc = facts[i](classes, impl_classes, false);
        if (sc) {
            return sc;
        }
    }

    for (u32 i = 0; i < ARRAY_LENGTH(facts); i++) {
        auto sc = facts[i](classes, impl_classes, true);
        if (sc) {
            return sc;
        }
    }

    return nullptr;
}

u32 sidecarSize(const sidecar *ns) {
    return ns->size;
}

u32 sidecarEnabledSize(const sidecar *n) {
    switch (n->type) {
    case SIDECAR_8:
        return sizeof(struct sidecar_enabled_8);
    case SIDECAR_32:
        return sizeof(struct sidecar_enabled_32);
    case SIDECAR_64:
        return sizeof(struct sidecar_enabled_64);
    case SIDECAR_128:
        return sizeof(struct sidecar_enabled_128);
    case SIDECAR_256:
        return sizeof(struct sidecar_enabled_256);
    case SIDECAR_N:
        return sizeof(struct sidecar_enabled_N);
    case SIDECAR_S:
        return sizeof(struct sidecar_enabled_S);
    default:
        assert(0);
    }
    return 0;
}

template<u8 s_type>
static
void sidecarEnabledAdd_int(const sidecar *nn, struct sidecar_enabled *enabled,
                           u32 id) {
    typedef typename sidecar_traits<s_type>::enabled_type e_type;
    typedef typename sidecar_traits<s_type>::impl_type n_type;
    e_type *e = (e_type *)enabled;
    const n_type *n = (const n_type *)nn;

    DEBUG_PRINTF("enabling %u\n", id);
    typedef typename sidecar_traits<s_type>::base_type base;
    const base *masks = (const base *)sidecar_ids_to_mask_const(n);
    or_into_mask(&e->bits, masks[id]);
}

template<>
void sidecarEnabledAdd_int<SIDECAR_S>(const sidecar *nn,
                                      sidecar_enabled *enabled, u32 id) {
    const sidecar_S *n = (const sidecar_S *)nn;
    sidecar_enabled_S *e = (sidecar_enabled_S *)enabled;
    const u8 *masks = (const u8 *)sidecar_ids_to_mask_const(n);
    e->bits |= masks[id];
}

template<>
void sidecarEnabledAdd_int<SIDECAR_N>(UNUSED const sidecar *n,
                                      struct sidecar_enabled *enabled,
                                      UNUSED u32 id) {
    sidecar_enabled_N *e = (sidecar_enabled_N *)enabled;
    /* assuming we are not being called by a complete idiot, there is only one
     * thing we could be asked to do here */
    e->bits = 1;
}

void sidecarEnabledAdd(const sidecar *n, struct sidecar_enabled *enabled,
                       u32 id) {
    DEBUG_PRINTF("enabling %hhu:%u\n", n->type, id);
    switch (n->type) {
    case SIDECAR_8:
        sidecarEnabledAdd_int<SIDECAR_8>(n, enabled, id);
        break;
    case SIDECAR_32:
        sidecarEnabledAdd_int<SIDECAR_32>(n, enabled, id);
        break;
    case SIDECAR_64:
        sidecarEnabledAdd_int<SIDECAR_64>(n, enabled, id);
        break;
    case SIDECAR_128:
        sidecarEnabledAdd_int<SIDECAR_128>(n, enabled, id);
        break;
    case SIDECAR_256:
        sidecarEnabledAdd_int<SIDECAR_256>(n, enabled, id);
        break;
    case SIDECAR_N:
        sidecarEnabledAdd_int<SIDECAR_N>(n, enabled, id);
        break;
    case SIDECAR_S:
        sidecarEnabledAdd_int<SIDECAR_S>(n, enabled, id);
        break;
    default:
        assert(0);
    }
}

} // namespace ue2
