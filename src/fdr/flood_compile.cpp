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

#include "fdr_internal.h"
#include "fdr_confirm.h"
#include "fdr_compile_internal.h"
#include "fdr_engine_description.h"
#include "grey.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace std;

namespace ue2 {

namespace {
struct FloodComparator {
    bool operator()(const FDRFlood &f1, const FDRFlood &f2) const {
        return std::memcmp(&f1, &f2, sizeof(f1)) < 0;
    }
};
}

static
bool isDifferent(u8 oldC, u8 c, bool caseless) {
    if (caseless) {
        return mytolower(oldC) != mytolower(c);
    } else {
        return oldC != c;
    }
}

static
void updateFloodSuffix(vector<FDRFlood> &tmpFlood, u8 c, u32 suffix) {
    FDRFlood &fl = tmpFlood[c];
    fl.suffix = MAX(fl.suffix, suffix + 1);
    DEBUG_PRINTF("Updated Flood Suffix for char 0x%02x to %u\n", c, fl.suffix);
}

static
void addFlood(vector<FDRFlood> &tmpFlood, u8 c, const hwlmLiteral &lit,
              u32 suffix) {
    FDRFlood &fl = tmpFlood[c];
    fl.suffix = MAX(fl.suffix, suffix + 1);
    if (fl.idCount < FDR_FLOOD_MAX_IDS) {
        fl.ids[fl.idCount] = lit.id;
        fl.allGroups |= lit.groups;
        fl.groups[fl.idCount] = lit.groups;
        // when idCount gets to max_ids this flood no longer happens
        // only incremented one more time to avoid arithmetic overflow
        DEBUG_PRINTF("Added Flood for char '%c' suffix=%u len[%hu]=%u\n",
                     c, fl.suffix, fl.idCount, suffix);
        fl.idCount++;
   }
}

bytecode_ptr<u8> setupFDRFloodControl(const vector<hwlmLiteral> &lits,
                                      const EngineDescription &eng,
                                      const Grey &grey) {
    vector<FDRFlood> tmpFlood(N_CHARS);
    u32 default_suffix = eng.getDefaultFloodSuffixLength();

    // zero everything to avoid spurious distinctions in the compares
    memset(&tmpFlood[0], 0, N_CHARS * sizeof(FDRFlood));

    for (u32 c = 0; c < N_CHARS; c++) {
        tmpFlood[c].suffix = default_suffix;
    }

    for (const auto &lit : lits) {
        DEBUG_PRINTF("lit: '%s'%s\n", escapeString(lit.s).c_str(),
                     lit.nocase ? " (nocase)" : "");
        u32 litSize = verify_u32(lit.s.size());
        u32 maskSize = (u32)lit.msk.size();
        u8 c = lit.s[litSize - 1];
        bool nocase = ourisalpha(c) ? lit.nocase : false;

        if (nocase && maskSize && (lit.msk[maskSize - 1] & CASE_BIT)) {
            c = (lit.cmp[maskSize - 1] & CASE_BIT) ? mytolower(c) : mytoupper(c);
            nocase = false;
        }

        u32 iEnd = MAX(litSize, maskSize);
        u32 upSuffix = iEnd; // upSuffix is used as an upper case suffix length
                             // for case-less, or as a suffix length for case-sensitive;
        u32 loSuffix = iEnd; // loSuffix used only for case-less as a lower case suffix
                             // length;

        for (u32 i = 0; i < iEnd; i++) {
            if (i < litSize) {
                if (isDifferent(c, lit.s[litSize - i - 1], lit.nocase)) {
                    DEBUG_PRINTF("non-flood char in literal[%u]: "
                                 "0x%02x != 0x%02x\n",
                                 i, c, lit.s[litSize - i - 1]);
                    upSuffix = MIN(upSuffix, i);
                    loSuffix = MIN(loSuffix, i); // makes sense only for case-less
                    break;
                }
            }
            if (i < maskSize) {
                u8 m = lit.msk[maskSize - i - 1];
                u8 cm = lit.cmp[maskSize - i - 1] & m;
                if(nocase) {
                    if ((mytoupper(c) & m) != cm) {
                        DEBUG_PRINTF("non-flood char in mask[%u] %c != %c\n",
                                                            i, mytoupper(c), cm);
                        upSuffix = MIN(upSuffix, i);
                    }
                    if ((mytolower(c) & m) != cm) {
                        DEBUG_PRINTF("non-flood char in mask[%u] %c != %c\n",
                                                            i, mytolower(c), cm);
                        loSuffix = MIN(loSuffix, i);
                    }
                    if (loSuffix != iEnd && upSuffix != iEnd) {
                        break;
                    }
                } else if ((c & m) != cm) {
                    DEBUG_PRINTF("non-flood char in mask[%u] %c != %c\n", i, c, cm);
                    upSuffix = MIN(upSuffix, i);
                    break;
                }
            }
        }
        if(upSuffix != iEnd) {
            updateFloodSuffix(tmpFlood, nocase ? mytoupper(c) : c, upSuffix);
        } else {
            addFlood(tmpFlood, nocase ? mytoupper(c) : c, lit, upSuffix);
        }
        if (nocase) {
            if(loSuffix != iEnd) {
                updateFloodSuffix(tmpFlood, mytolower(c), loSuffix);
            } else {
                addFlood(tmpFlood, mytolower(c), lit, loSuffix);
            }
        }
    }

#ifdef DEBUG
    for (u32 i = 0; i < N_CHARS; i++) {
        FDRFlood &fl = tmpFlood[i];
        if (!fl.idCount) {
            continue;
        }

        printf("i is %02x fl->idCount is %hd fl->suffix is %d fl->allGroups is "
               "%016llx\n", i, fl.idCount, fl.suffix, fl.allGroups);
        for (u32 j = 0; j < fl.idCount; j++) {
            printf("j is %d fl.groups[j] %016llx\n", j, fl.groups[j]);
        }
    }
#endif

    // If flood detection has been switched off in the grey box, we comply by
    // setting idCount too high for all floods.
    if (!grey.fdrAllowFlood) {
        for (auto &fl : tmpFlood) {
            fl.idCount = FDR_FLOOD_MAX_IDS;
        }
    }

    map<FDRFlood, CharReach, FloodComparator> flood2chars;
    for (u32 i = 0; i < N_CHARS; i++) {
        FDRFlood fl = tmpFlood[i];
        flood2chars[fl].set(i);
    }

    u32 nDistinctFloods = flood2chars.size();
    size_t floodHeaderSize = sizeof(u32) * N_CHARS;
    size_t floodStructSize = sizeof(FDRFlood) * nDistinctFloods;
    size_t totalSize = ROUNDUP_16(floodHeaderSize + floodStructSize);

    auto buf = make_zeroed_bytecode_ptr<u8>(totalSize, 16);
    assert(buf); // otherwise would have thrown std::bad_alloc

    u32 *floodHeader = (u32 *)buf.get();
    FDRFlood *layoutFlood = (FDRFlood *)(buf.get() + floodHeaderSize);

    u32 currentFloodIndex = 0;
    for (const auto &m : flood2chars) {
        const FDRFlood &fl = m.first;
        const CharReach &cr = m.second;
        layoutFlood[currentFloodIndex] = fl;
        for (size_t c = cr.find_first(); c != cr.npos; c = cr.find_next(c)) {
            floodHeader[c] = currentFloodIndex;
        }
        currentFloodIndex++;
    }

    DEBUG_PRINTF("made a flood structure with %zu + %zu = %zu\n",
                 floodHeaderSize, floodStructSize, totalSize);

    return buf;
}

} // namespace ue2
