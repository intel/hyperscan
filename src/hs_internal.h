/*
 * Copyright (c) 2019, Intel Corporation
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
 * \brief Internal-use only definitions. Available to internal tools.
 */

#ifndef HS_INTERNAL_H
#define HS_INTERNAL_H

#include "ue2common.h"
#include "hs.h"

#ifdef __cplusplus

namespace ue2 {

struct Grey;

/** \brief Internal use only: takes a Grey argument so that we can use it in
 * tools. */
hs_error_t hs_compile_multi_int(const char *const *expressions,
                                const unsigned *flags, const unsigned *ids,
                                const hs_expr_ext *const *ext,
                                unsigned elements, unsigned mode,
                                const hs_platform_info_t *platform,
                                hs_database_t **db,
                                hs_compile_error_t **comp_error, const Grey &g);

/** \brief Internal use only: takes a Grey argument so that we can use it in
 * tools. */
hs_error_t hs_compile_lit_multi_int(const char *const *expressions,
                                    const unsigned *flags, const unsigned *ids,
                                    const hs_expr_ext *const *ext,
                                    const size_t *lens, unsigned elements,
                                    unsigned mode,
                                    const hs_platform_info_t *platform,
                                    hs_database_t **db,
                                    hs_compile_error_t **comp_error,
                                    const Grey &g);
} // namespace ue2

extern "C"
{
#endif

#define HS_MATCH_FLAG_ADJUSTED  1U

/** \brief Bitmask of all valid Hyperscan flags. */
#define HS_FLAG_ALL ( HS_FLAG_CASELESS \
                    | HS_FLAG_DOTALL \
                    | HS_FLAG_MULTILINE \
                    | HS_FLAG_UTF8 \
                    | HS_FLAG_UCP \
                    | HS_FLAG_PREFILTER \
                    | HS_FLAG_SINGLEMATCH \
                    | HS_FLAG_ALLOWEMPTY \
                    | HS_FLAG_SOM_LEFTMOST)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
