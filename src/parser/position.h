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
 * \brief Per-position flags used during Glushkov construction, PositionInfo class.
 */

#ifndef PARSER_POSITION_H
#define PARSER_POSITION_H

#include "ue2common.h"

#include <set>

namespace ue2 {

#define POS_FLAG_NOFLOAT                    (1 << 0) //!< don't wire to start-dotstar
#define POS_FLAG_MUST_FLOAT                 (1 << 1) //!< don't wire solely to start
#define POS_FLAG_FIDDLE_ACCEPT              (1 << 2) //!< add a dot with an offset adjustment when wiring to accept
#define POS_FLAG_ASSERT_WORD_TO_NONWORD     (1 << 3) //!< epsilon for word to nonword transition
#define POS_FLAG_ASSERT_NONWORD_TO_WORD     (1 << 4) //!< epsilon for nonword to word transition
#define POS_FLAG_ASSERT_WORD_TO_WORD        (1 << 5) //!< epsilon for word to word transition
#define POS_FLAG_ASSERT_NONWORD_TO_NONWORD  (1 << 6) //!< epsilon for nonword to nonword transition

/** vertex created by cloning startDs, not considered part of the match.
 * mirrors POS_FLAG_FIDDLE_ACCEPT */
#define POS_FLAG_VIRTUAL_START              (1 << 7)

/** multi-line ^ does not match \\n at end of buffer. As a result, we must never
 * wire the \\n from ^ to eod */
#define POS_FLAG_MULTILINE_START            (1 << 8)

#define POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP     (1 << 9)
#define POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP     (1 << 10)
#define POS_FLAG_ASSERT_WORD_TO_WORD_UCP        (1 << 11)
#define POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP  (1 << 12)

#define POS_FLAG_ASSERT_NONWORD_TO_ANY  (POS_FLAG_ASSERT_NONWORD_TO_NONWORD \
                                   | POS_FLAG_ASSERT_NONWORD_TO_WORD)
#define POS_FLAG_ASSERT_WORD_TO_ANY (POS_FLAG_ASSERT_WORD_TO_NONWORD \
                                   | POS_FLAG_ASSERT_WORD_TO_WORD)

#define POS_FLAG_ASSERT_ANY_TO_NONWORD  (POS_FLAG_ASSERT_NONWORD_TO_NONWORD \
                                         | POS_FLAG_ASSERT_WORD_TO_NONWORD)
#define POS_FLAG_ASSERT_ANY_TO_WORD (POS_FLAG_ASSERT_NONWORD_TO_WORD \
                                   | POS_FLAG_ASSERT_WORD_TO_WORD)

#define POS_FLAG_ASSERT_NONWORD_TO_ANY_UCP                              \
    (POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP                             \
     | POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP)
#define POS_FLAG_ASSERT_WORD_TO_ANY_UCP (POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP \
                                   | POS_FLAG_ASSERT_WORD_TO_WORD_UCP)

#define POS_FLAG_ASSERT_ANY_TO_NONWORD_UCP                              \
    (POS_FLAG_ASSERT_NONWORD_TO_NONWORD_UCP                             \
     | POS_FLAG_ASSERT_WORD_TO_NONWORD_UCP)
#define POS_FLAG_ASSERT_ANY_TO_WORD_UCP (POS_FLAG_ASSERT_WORD_TO_WORD_UCP \
                                   | POS_FLAG_ASSERT_NONWORD_TO_WORD_UCP)

#define UCP_ASSERT_FLAGS (POS_FLAG_ASSERT_WORD_TO_ANY_UCP \
                        | POS_FLAG_ASSERT_NONWORD_TO_ANY_UCP)

#define NON_UCP_ASSERT_FLAGS (POS_FLAG_ASSERT_WORD_TO_ANY \
                            | POS_FLAG_ASSERT_NONWORD_TO_ANY)

/** do not wire to accept or other pos; may still wire to eod, etc if
 * instructed */
#define POS_FLAG_ONLY_ENDS (1 << 23)

#define POS_FLAG_WIRE_EOD       (1 << 24) /**< wire to accept eod */
#define POS_FLAG_WIRE_NL_EOD    (1 << 25) /**< wire to nl before accept eod */
#define POS_FLAG_WIRE_NL_ACCEPT (1 << 26) /**< wire to nl before accept */
#define POS_FLAG_NO_NL_EOD      (1 << 27) /**< disallow nl before accept eod */
#define POS_FLAG_NO_NL_ACCEPT   (1 << 28) /**< disallow nl before accept */

/** \brief Parse and Glushkov construction use only. State number within the
 * NFA as it is being constructed. */
typedef u32 Position;

} // namespace ue2

#endif // PARSER_POSITION_H
