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

#ifndef NG_CORPUS_PROPERTIES_H
#define NG_CORPUS_PROPERTIES_H

#include <utility> // for std::pair
#include <boost/random/mersenne_twister.hpp>

#include "ue2common.h"

#define DEFAULT_CORPUS_GENERATOR_LIMIT 500000

struct min_max {
    min_max(u32 min_in, u32 max_in) : min(min_in), max(max_in) {
        assert(min <= max);
    }
    u32 min;
    u32 max;
};

class CorpusProperties {
public:
    /**
     * Default constructor with default properties:
     * - generate match char with 100% probability
     * - generate unmatch char with 0% probability
     * - generate random char with 0% probability
     * - follow cycles once
     * - do not expand character classes (including case classes)
     * - generate data for all possible paths through graph
     * - pick random characters from the full ASCII alphabet
     */
    CorpusProperties();

    /**
     * Set probabilities (as percentages).  Returns true if sum == 100,
     * else returns false and no changes are made to current probabilities.
     */
    bool setPercentages(unsigned int match, unsigned int unmatch,
            unsigned int random);

    unsigned percentMatch() const { return matchness; }
    unsigned percentUnmatch() const { return unmatchness; }
    unsigned percentRandom() const { return randomness; }

    // The number of times a cycle is followed
    void setCycleLimit(unsigned int min, unsigned int max) {
        cycleMin = min;
        cycleMax = max;
    }
    std::pair<unsigned int, unsigned int> getCycleLimit() const {
        return std::make_pair(cycleMin, cycleMax);
    }

    // Roll for initiative
    enum RollResult {
        ROLLED_MATCH,
        ROLLED_UNMATCH,
        ROLLED_RANDOM,
    };
    RollResult throwDice();

    /** \brief Set the PRNG seed. */
    void seed(unsigned val);
    unsigned int getSeed() const;

    /** \brief Retrieve a value from the PRNG in the closed range [n, m]. */
    unsigned rand(unsigned n, unsigned m);

private:
    // Percentages
    unsigned int matchness;
    unsigned int unmatchness;
    unsigned int randomness;

public:
    // Extra data
    min_max prefixRange;
    min_max suffixRange;

private:
    // Behaviours
    unsigned int cycleMin;
    unsigned int cycleMax;

public:
    // FIXME: Limit the number of corpus files generated to the first 'limit'
    // number of paths - note that this means the corpus will not be a complete
    // representation of the pattern.
    unsigned int corpusLimit;

    unsigned int editDistance;
    unsigned int alphabetSize;

private:
    // PRNG.
    boost::random::mt19937 randomGen;
    unsigned int rngSeed;
};

#endif
