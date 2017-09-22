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

/** \file
 * \brief Hamster Wheel Literal Matcher: build API.
 */

#ifndef HWLM_BUILD_H
#define HWLM_BUILD_H

#include "hwlm.h"
#include "hwlm_literal.h"
#include "ue2common.h"
#include "util/bytecode_ptr.h"

#include <map>
#include <memory>
#include <vector>

struct HWLM;

namespace ue2 {

class FDREngineDescription;
class TeddyEngineDescription;
struct CompileContext;
struct Grey;

/** \brief Class representing a literal matcher prototype. */
struct HWLMProto {
    /**
     * \brief Engine type to distinguish noodle from FDR and Teddy.
     */
    u8 engType;

    /**
     * \brief FDR engine description.
     */
    std::unique_ptr<FDREngineDescription> fdrEng;

    /**
     * \brief Teddy engine description.
     */
    std::unique_ptr<TeddyEngineDescription> teddyEng;

     /**
      * \brief HWLM literals passed from Rose.
      */
    std::vector<hwlmLiteral> lits;

    /**
     * \brief Bucket assignment info in FDR and Teddy
     */
    std::map<u32, std::vector<u32>> bucketToLits;

    /**
     * \brief Flag to optimise matcher for small size from Rose.
     */
    bool make_small = false;

    HWLMProto(u8 engType_in, std::vector<hwlmLiteral> lits_in);

    HWLMProto(u8 engType_in, std::unique_ptr<FDREngineDescription> eng_in,
              std::vector<hwlmLiteral> lits_in,
              std::map<u32, std::vector<u32>> bucketToLits_in,
              bool make_small_in);

    HWLMProto(u8 engType_in, std::unique_ptr<TeddyEngineDescription> eng_in,
              std::vector<hwlmLiteral> lits_in,
              std::map<u32, std::vector<u32>> bucketToLits_in,
              bool make_small_in);

    ~HWLMProto();
};

/** \brief Build an \ref HWLM literal matcher runtime structure for a group of
 * literals.
 *
 * \param proto Literal matcher prototype.
 * \param cc Compile context.
 * \param expected_groups FIXME: document me!
 *
 * Build failures are generally a result of memory allocation failure. These
 * may result in a nullptr return value, or a std::bad_alloc exception being
 * thrown.
 */
bytecode_ptr<HWLM> hwlmBuild(const HWLMProto &proto, const CompileContext &cc,
                             hwlm_group_t expected_groups = HWLM_ALL_GROUPS);

std::unique_ptr<HWLMProto>
hwlmBuildProto(std::vector<hwlmLiteral> &lits, bool make_small,
               const CompileContext &cc);

/**
 * Returns an estimate of the number of repeated characters on the end of a
 * literal that will make a literal set of size \a numLiterals suffer
 * performance degradation.
 */
size_t hwlmFloodProneSuffixLen(size_t numLiterals, const CompileContext &cc);

/** \brief Return the size in bytes of an HWLM structure. */
size_t hwlmSize(const HWLM *h);

} // namespace

#endif // HWLM_BUILD_H
