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

#ifndef CORPORA_H
#define CORPORA_H

#include <set>
#include <string>
#include <vector>

#include <boost/core/noncopyable.hpp>

struct Corpus {
    Corpus() : hasMatches(false) {}
    explicit Corpus(const std::string &s) : data(s), hasMatches(false) {}

    std::string data; // Corpus itself
    bool hasMatches; // Have the matches been pre-calculated?
    std::set<unsigned int> matches; // end-offsets of matches
};

struct CorpusFailure {
    explicit CorpusFailure(const std::string &s) : message(s) {}
    std::string message;
};

// Abstract class for a corpora source: new ways to load or generate corpora
// can be written by subclassing this class and providing its generate
// method.
class CorporaSource : boost::noncopyable {
public:
    // destructor
    virtual ~CorporaSource();

    // Make a copy of this corpora source.
    virtual CorporaSource *clone() const = 0;

    // Generate corpora for the given signature ID, adding them to the
    // vector of strings provided.
    virtual void generate(unsigned id, std::vector<Corpus> &data) = 0;
};

#endif // CORPORA_H
