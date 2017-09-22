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

/**
 * \file
 * \brief Noodle literal matcher: build code.
 */

#include "noodle_build.h"

#include "hwlm_literal.h"
#include "noodle_internal.h"
#include "util/bitutils.h"
#include "util/compare.h"
#include "util/verify_types.h"
#include "ue2common.h"

#include <cstring> // for memcpy
#include <vector>

using std::vector;

namespace ue2 {

static
u64a make_u64a_mask(const vector<u8> &v) {
    assert(v.size() <= sizeof(u64a));
    if (v.size() > sizeof(u64a)) {
        throw std::exception();
    }

    u64a mask = 0;
    size_t len = v.size();
    unsigned char *m = (unsigned char *)&mask;
    DEBUG_PRINTF("making mask len %zu\n", len);
    memcpy(m, &v[0], len);
    return mask;
}

static
size_t findNoodFragOffset(const hwlmLiteral &lit) {
    const auto &s = lit.s;
    const size_t len = lit.s.length();

    size_t offset = 0;
    for (size_t i = 0; i + 1 < len; i++) {
        int diff = 0;
        const char c = s[i];
        const char d = s[i + 1];
        if (lit.nocase && ourisalpha(c)) {
            diff = (mytoupper(c) != mytoupper(d));
        } else {
            diff = (c != d);
        }
        offset = i;
        if (diff) {
            break;
        }
    }
    return offset;
}

bytecode_ptr<noodTable> noodBuildTable(const hwlmLiteral &lit) {
    const auto &s = lit.s;

    size_t mask_len = std::max(s.length(), lit.msk.size());
    DEBUG_PRINTF("mask is %zu bytes\n", lit.msk.size());
    assert(mask_len <= 8);
    assert(lit.msk.size() == lit.cmp.size());

    vector<u8> n_msk(mask_len);
    vector<u8> n_cmp(mask_len);

    for (unsigned i = mask_len - lit.msk.size(), j = 0; i < mask_len;
         i++, j++) {
        DEBUG_PRINTF("m[%u] %hhx c[%u] %hhx\n", i, lit.msk[j], i, lit.cmp[j]);
        n_msk[i] = lit.msk[j];
        n_cmp[i] = lit.cmp[j];
    }

    size_t s_off = mask_len - s.length();
    for (unsigned i = s_off; i < mask_len; i++) {
        u8 c = s[i - s_off];
        u8 si_msk = lit.nocase && ourisalpha(c) ? (u8)CASE_CLEAR : (u8)0xff;
        n_msk[i] |= si_msk;
        n_cmp[i] |= c & si_msk;
        assert((n_cmp[i] & si_msk) == c);
        DEBUG_PRINTF("m[%u] %hhx c[%u] %hhx '%c'\n", i, n_msk[i], i, n_cmp[i],
                     ourisprint(c) ? (char)c : '.');
    }

    auto n = make_zeroed_bytecode_ptr<noodTable>(sizeof(noodTable));
    assert(n);
    DEBUG_PRINTF("size of nood %zu\n", sizeof(noodTable));

    size_t key_offset = findNoodFragOffset(lit);

    n->id = lit.id;
    n->single = s.length() == 1 ? 1 : 0;
    n->key_offset = verify_u8(s.length() - key_offset);
    n->nocase = lit.nocase ? 1 : 0;
    n->key0 = s[key_offset];
    if (n->single) {
        n->key1 = 0;
    } else {
        n->key1 = s[key_offset + 1];
    }
    n->msk = make_u64a_mask(n_msk);
    n->cmp = make_u64a_mask(n_cmp);
    n->msk_len = mask_len;

    return n;
}

size_t noodSize(const noodTable *) {
    return sizeof(noodTable);
}

} // namespace ue2

#ifdef DUMP_SUPPORT
#include <cctype>

namespace ue2 {

void noodPrintStats(const noodTable *n, FILE *f) {
    fprintf(f, "Noodle table\n");
    fprintf(f, "Key Offset: %u\n", n->key_offset);
    fprintf(f, "Msk: %llx Cmp: %llx MskLen %u\n",
            n->msk >> 8 * (8 - n->msk_len), n->cmp >> 8 * (8 - n->msk_len),
            n->msk_len);
    fprintf(f, "String: ");
    for (u32 i = 0; i < n->msk_len; i++) {
        const u8 *m = (const u8 *)&n->cmp;
        if (isgraph(m[i]) && m[i] != '\\') {
            fprintf(f, "%c", m[i]);
        } else {
            fprintf(f, "\\x%02hhx", m[i]);
        }
    }
    fprintf(f, "\n");
}

} // namespace ue2

#endif
