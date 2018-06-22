/*
 * Copyright (c) 2017-2018, Intel Corporation
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
 * \brief ExpressionInfo class for storing the properties of an expression.
 */

#ifndef COMPILER_EXPRESSION_INFO_H
#define COMPILER_EXPRESSION_INFO_H

#include "ue2common.h"
#include "som/som.h"

namespace ue2 {

/** \brief Properties of an expression. */
class ExpressionInfo {
public:
    ExpressionInfo(unsigned int index_in, bool allow_vacuous_in,
                   bool highlander_in, bool utf8_in, bool prefilter_in,
                   som_type som_in, ReportID report_in, u64a min_offset_in,
                   u64a max_offset_in, u64a min_length_in, u32 edit_distance_in,
                   u32 hamm_distance_in, bool quiet_in)
        : index(index_in), report(report_in), allow_vacuous(allow_vacuous_in),
          highlander(highlander_in), utf8(utf8_in), prefilter(prefilter_in),
          som(som_in), min_offset(min_offset_in), max_offset(max_offset_in),
          min_length(min_length_in), edit_distance(edit_distance_in),
          hamm_distance(hamm_distance_in), quiet(quiet_in) {}

    /**
     * \brief Index of the expression represented by this graph.
     *
     * Used:
     * - down the track in error handling;
     * - for identifying parts of an expression in highlander mode.
     */
    unsigned int index;

    /** \brief Report ID specified by the user. */
    ReportID report;

    /** \brief Vacuous pattern is allowed. (HS_FLAG_ALLOWEMPTY) */
    bool allow_vacuous;

    /** \brief "Highlander" (single match) pattern. (HS_FLAG_SINGLEMATCH) */
    bool highlander;

    /** \brief UTF-8 pattern. (HS_FLAG_UTF8) */
    bool utf8;

    /** \brief Prefiltering pattern. (HS_FLAG_PREFILTER) */
    bool prefilter;

    /** \brief Start-of-match type requested, or SOM_NONE. */
    som_type som;

    /** \brief Minimum match offset extended parameter. 0 if not used. */
    u64a min_offset;

    /**
     * \brief Maximum match offset extended parameter.
     * MAX_OFFSET if not used.
     */
    u64a max_offset;

    /** \brief Minimum match length extended parameter. 0 if not used. */
    u64a min_length;

    /**
     * \brief Approximate matching edit distance extended parameter.
     * 0 if not used.
     */
    u32 edit_distance;
    u32 hamm_distance;

    /** \brief Quiet on match. */
    bool quiet;
};

}

#endif // COMPILER_EXPRESSION_INFO_H
