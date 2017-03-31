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

#ifndef SMWR_BUILD_H
#define SMWR_BUILD_H

/**
 * \file
 * \brief Small-write engine build interface.
 *
 * Everything you ever needed to feed literals in and get a SmallWriteEngine
 * out. This header should be everything needed by the rest of UE2.
 */

#include "ue2common.h"
#include "util/bytecode_ptr.h"
#include "util/noncopyable.h"

#include <memory>
#include <set>

struct SmallWriteEngine;

namespace ue2 {

struct CompileContext;
struct ue2_literal;
class ExpressionInfo;
class NGHolder;
class ReportManager;

/**
 * Abstract interface intended for callers from elsewhere in the tree, real
 * underlying implementation is SmallWriteBuildImpl in smwr_build_impl.h.
 */
class SmallWriteBuild : noncopyable {
public:
    virtual ~SmallWriteBuild();

    virtual bytecode_ptr<SmallWriteEngine> build(u32 roseQuality) = 0;

    virtual void add(const NGHolder &g, const ExpressionInfo &expr) = 0;
    virtual void add(const ue2_literal &literal, ReportID r) = 0;

    virtual std::set<ReportID> all_reports() const = 0;
};

/** \brief Construct a usable SmallWrite builder. */
std::unique_ptr<SmallWriteBuild>
makeSmallWriteBuilder(size_t num_patterns, const ReportManager &rm,
                      const CompileContext &cc);

} // namespace ue2

#endif // SMWR_BUILD_H
