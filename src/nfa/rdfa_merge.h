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
 * \brief Merge code for McClellan DFA.
 */

#ifndef RDFA_MERGE_H
#define RDFA_MERGE_H

#include <memory>
#include <vector>

namespace ue2 {

class ReportManager;
struct raw_dfa;
struct Grey;

/** \brief Attempts to merge two raw_dfas into one. */
std::unique_ptr<raw_dfa> mergeTwoDfas(const raw_dfa *d1, const raw_dfa *d2,
                                      size_t max_states, const ReportManager *rm,
                                      const Grey &grey);

/** \brief Attempts to merge all the given raw_dfas into one. */
std::unique_ptr<raw_dfa> mergeAllDfas(const std::vector<const raw_dfa *> &dfas,
                                      size_t max_states,
                                      const ReportManager *rm,
                                      const Grey &grey);

/** \brief Merges the given list of raw_dfas as much as possible in-place. */
void mergeDfas(std::vector<std::unique_ptr<raw_dfa>> &dfas, size_t max_states,
               const ReportManager *rm, const Grey &grey);

} // namespace ue2

#endif // RDFA_MERGE_H
