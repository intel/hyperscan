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
 * \brief SOM Slot Manager.
 */

#ifndef SLOT_MANAGER_H
#define SLOT_MANAGER_H

#include "ue2common.h"
#include "nfagraph/ng_holder.h"
#include "util/bytecode_ptr.h"
#include "util/noncopyable.h"

#include <deque>
#include <memory>
#include <unordered_map>

struct NFA;

namespace ue2 {

class CharReach;
class NGHolder;
struct Grey;
struct SlotCache;

/** \brief SOM slot manager. Used to hand out SOM slots and track their
 * relationships during SOM construction. Also stores reverse NFAs used for
 * SOM. */
class SomSlotManager : noncopyable {
public:
    explicit SomSlotManager(u8 precision);
    ~SomSlotManager();

    /** \brief Sentinel value used to specify that a slot has no parent. */
    static constexpr u32 NO_PARENT = ~0;

    u32 getSomSlot(const NGHolder &prefix, const CharReach &escapes,
                   bool is_reset, u32 parent_slot);

    /** prefix must be acting as a resetting sentinel and should be a dag (if
     * not how are we establish som?) */
    u32 getInitialResetSomSlot(const NGHolder &prefix, const NGHolder &g,
                           const std::unordered_map<NFAVertex, u32> &region_map,
                           u32 last_sent_region,
                           bool *prefix_already_implemented);

    u32 getPrivateSomSlot(void);

    void rollbackSomTo(u32 num);

    u32 numSomSlots() const;

    const std::deque<bytecode_ptr<NFA>> &getRevNfas() const {
        return rev_nfas;
    }

    u32 addRevNfa(bytecode_ptr<NFA> nfa, u32 maxWidth);

    u32 somHistoryRequired() const { return historyRequired; }

    u32 somPrecision() const { return precision; }

    void somPrecision(u32 p) {
        precision = p;
    }

private:
    u32 nextSomSlot;
    std::unique_ptr<SlotCache> cache;

    /** \brief Reverse NFAs used for SOM support. */
    std::deque<bytecode_ptr<NFA>> rev_nfas;

    /** \brief In streaming mode, the amount of history we've committed to
     * using for SOM rev NFAs. */
    u32 historyRequired;

    /** \brief Number of bytes of SOM precision requested by the user, zero if
     * not in SOM mode. */
    u32 precision;

#ifdef DUMP_SUPPORT
    friend void dumpSomSlotManager(const SomSlotManager &ssm, const Grey &grey);
#endif
};

} // namespace ue2

#endif
