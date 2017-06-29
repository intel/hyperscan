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

#include "config.h"

#include "slot_manager_dump.h"
#include "slot_manager_internal.h"
#include "slot_manager.h"

#include "grey.h"
#include "nfagraph/ng_dump.h"
#include "nfagraph/ng_is_equal.h"
#include "util/container.h"
#include "util/dump_util.h"
#include "ue2common.h"

#include <cstdio>
#include <map>
#include <string>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

void dumpSomSlotManager(const SomSlotManager &ssm, const Grey &grey) {
    if (!grey.dumpFlags) {
        return;
    }

    map<u32, const SlotCacheEntry *> by_slot;
    map<u32, const InitialResetInfo *> by_slot_ir;

    for (const auto &e : ssm.cache->store) {
        by_slot[e.slot] = &e;
    }

    for (const auto &e : ssm.cache->initial_resets) {
        by_slot_ir[e.slot] = &e;
    }

    StdioFile f(grey.dumpPath + "/ssm.txt", "w");

    fprintf(f, "slot width %u bytes\n\n", ssm.precision);

    if (by_slot.empty()) {
        fprintf(f, "<no som slots>\n");
    }

    for (u32 i = 0; i < ssm.numSomSlots(); i++) {
        fprintf(f, "%u", i);
        if (contains(by_slot_ir, i)) {
            const InitialResetInfo &ir = *by_slot_ir[i];
            fprintf(f, "\t shared reset (users = %zu)\n", ir.entries.size());
        } else if (contains(by_slot, i)) {
            const SlotCacheEntry &ce = *by_slot.at(i);
            if (ce.parent_slot != SomSlotManager::NO_PARENT) {
                fprintf(f, "\tparent:%u", ce.parent_slot);
            }
            if (ce.is_reset) {
                fprintf(f, "\treset");
            }
            fprintf(f, "\n");
        } else {
            fprintf(f, "\t<private>\n");
        }
    }

    for (const auto &h : ssm.cache->initial_prefixes) {
        dumpHolder(*h, hash_holder(*h), "ssm_prefix", grey);
    }
}

} // namespace ue2
