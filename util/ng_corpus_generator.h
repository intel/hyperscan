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
 * \brief Corpus Generation tool.
 */

#ifndef NG_CORPUS_GENERATOR_H_
#define NG_CORPUS_GENERATOR_H_

#include "ng_corpus_properties.h"

#include <memory>
#include <string>
#include <vector>

namespace ue2 {

class ExpressionInfo;
class NGHolder;

} // namespace ue2

struct CorpusGenerationFailure {
    explicit CorpusGenerationFailure(const std::string s) :
        message(std::move(s)) {}
    std::string message;
};

/** \brief Abstract interface to corpus generator tool. */
class CorpusGenerator {
public:
    virtual ~CorpusGenerator();

    /** \brief Build some corpora.
     *
     * Generate a set of corpora, placed in the \a data vector, for the current
     * NFAGraph according to the parameters provided by the CorpusProperties
     * object. Returns the number of corpora generated.
     */
    virtual void generateCorpus(std::vector<std::string> &data) = 0;
};

/** \brief Build a concrete impl conforming to the \ref CorpusGenerator
 * interface. */
std::unique_ptr<CorpusGenerator>
makeCorpusGenerator(const ue2::NGHolder &g, const ue2::ExpressionInfo &expr,
                    CorpusProperties &props);

#endif
