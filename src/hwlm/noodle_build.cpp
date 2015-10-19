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

/** \file
 * \brief Noodle literal matcher: build code.
 */
#include <cstring> // for memcpy

#include "noodle_build.h"
#include "noodle_internal.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/compare.h"
#include "util/verify_types.h"

namespace ue2 {

static
size_t findNoodFragOffset(const u8 *lit, size_t len, bool nocase) {
    size_t offset = 0;
    for (size_t i = 0; i + 1 < len; i++) {
        int diff = 0;
        const char c = lit[i];
        const char d = lit[i + 1];
        if (nocase && ourisalpha(c)) {
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

/** \brief Construct a Noodle matcher for the given literal. */
aligned_unique_ptr<noodTable> noodBuildTable(const u8 *lit, size_t len,
                                             bool nocase, u32 id) {
    size_t noodle_len = sizeof(noodTable) + len;
    aligned_unique_ptr<noodTable> n =
        aligned_zmalloc_unique<noodTable>(noodle_len);
    assert(n);

    size_t key_offset = findNoodFragOffset(lit, len, nocase);

    n->id = id;
    n->len = verify_u32(len);
    n->key_offset = verify_u32(key_offset);
    n->nocase = nocase ? 1 : 0;
    memcpy(n->str, lit, len);

    return n;
}

size_t noodSize(const noodTable *n) {
    assert(n); // shouldn't call with null
    return sizeof(*n) + n->len;
}

} // namespace ue2

#ifdef DUMP_SUPPORT
#include <cctype>

namespace ue2 {

void noodPrintStats(const noodTable *n, FILE *f) {
    fprintf(f, "Noodle table\n");
    fprintf(f, "Len: %u Key Offset: %u\n", n->len, n->key_offset);
    fprintf(f, "String: ");
    for (u32 i = 0; i < n->len; i++) {
        if (isgraph(n->str[i]) && n->str[i] != '\\') {
            fprintf(f, "%c", n->str[i]);
        } else {
            fprintf(f, "\\x%02hhx", n->str[i]);
        }
    }
    fprintf(f, "\n");
}

} // namespace ue2

#endif
