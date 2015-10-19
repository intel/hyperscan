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
 * \brief State for corpus generator.
  */

#include "config.h"

#include "ng_corpus_properties.h"
#include "ue2common.h"

#include <boost/random/uniform_int_distribution.hpp>

// default constructor
CorpusProperties::CorpusProperties()
    : matchness(100), unmatchness(0), randomness(0), prefixRange(0, 0),
      suffixRange(0, 0), cycleMin(1), cycleMax(1),
      corpusLimit(DEFAULT_CORPUS_GENERATOR_LIMIT), editDistance(0),
      alphabetSize(~0) {
    // empty
}

bool CorpusProperties::setPercentages(unsigned int match, unsigned int unmatch,
                                      unsigned int random) {
    if (match + unmatch + random != 100) {
        // Do not update probabilities
        return false;
    }
    matchness = match;
    unmatchness = unmatch;
    randomness = random;
    return true;
}

void CorpusProperties::seed(unsigned val) {
    rngSeed = val;
    randomGen.seed(val);
}

unsigned CorpusProperties::getSeed() const {
    return rngSeed;
}

unsigned CorpusProperties::rand(unsigned n, unsigned m) {
    boost::random::uniform_int_distribution<> dist(n, m);
    return dist(randomGen);
}

// not const because it stores state for the random number generator
CorpusProperties::RollResult CorpusProperties::throwDice() {
    if (matchness == 100) {
        return ROLLED_MATCH;
    }
    if (unmatchness == 100) {
        return ROLLED_UNMATCH;
    }
    if (randomness == 100) {
        return ROLLED_RANDOM;
    }

    // This assumes a uniform distribution.  Perhaps factor some 'depth' param
    // and whether this 'depth' should increase or decrease the likelihood of
    // unmatch or random rolls.
    unsigned int outcome = rand(0, 99);
    if (outcome < matchness) {
        return ROLLED_MATCH;
    }
    if (outcome < matchness + unmatchness) {
        return ROLLED_UNMATCH;
    }

    return ROLLED_RANDOM;
}
