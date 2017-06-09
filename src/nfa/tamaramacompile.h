/*
 * Copyright (c) 2016-2017, Intel Corporation
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
 * \brief Tamarama: container engine for exclusive engines, compiler code.
 */

#ifndef NFA_TAMARAMACOMPILE_H
#define NFA_TAMARAMACOMPILE_H

#include "ue2common.h"
#include "util/bytecode_ptr.h"

#include <map>
#include <set>
#include <vector>

struct NFA;

namespace ue2 {

/**
 * \brief A TamaProto that contains top remapping and reports info.
 */
struct TamaProto {
    void add(const NFA *n, const u32 id, const u32 top,
             const std::map<std::pair<const NFA *, u32>, u32> &out_top_remap);
    /** Top remapping between <vertex id, top value> and
     ** remapped top value. */
    std::map<std::pair<u32, u32>, u32> top_remap;

    /** All the reports in subengines */
    std::set<ReportID> reports;
};

/**
 * \brief Construction info for a Tamarama engine:
 * contains at least two subengines.
 *
 * A TamaInfo is converted into a single NFA, with each top triggering a
 * subengine. A TamaInfo can contain at most TamaInfo::max_occupancy
 * subengines.
 */
struct TamaInfo {
    static constexpr size_t max_occupancy = 65536; // arbitrary limit

    /** \brief Add a new subengine. */
    void add(NFA *sub, const std::set<u32> &top);

    /** \brief All the subengines */
    std::vector<NFA *> subengines;

    /** \brief Tops of subengines */
    std::vector<std::set<u32>> tops;
};

std::set<ReportID> all_reports(const TamaProto &proto);

/**
 * Take in a collection of exclusive subengines and produces a tamarama, also
 * returns via out_top_remap, a mapping indicating how tops in the subengines in
 * relate to the tamarama's tops.
 */
bytecode_ptr<NFA>
buildTamarama(const TamaInfo &tamaInfo, const u32 queue,
              std::map<std::pair<const NFA *, u32>, u32> &out_top_remap);

} // namespace ue2

#endif // NFA_TAMARAMACOMPILE_H
