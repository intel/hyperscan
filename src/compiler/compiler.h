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
 * \brief Compiler front-end interface
 */

#ifndef COMPILER_H
#define COMPILER_H

#include "ue2common.h"
#include "database.h"
#include "parser/Component.h"
#include "som/som.h"

#include <memory>
#include <boost/core/noncopyable.hpp>

struct hs_database;
struct hs_expr_ext;

namespace ue2 {

struct CompileContext;
struct Grey;
struct target_t;
class NG;
class ReportManager;
class NGWrapper;

/** Class gathering together the pieces of a parsed expression.
 * Note: Owns the provided component.
 */
class ParsedExpression : boost::noncopyable {
public:
    ParsedExpression(unsigned index, const char *expression, unsigned flags,
                     ReportID actionId, const hs_expr_ext *ext = nullptr);

    bool utf8; //!< UTF-8 mode flag specified

    /** \brief root node of parsed component tree. */
    std::unique_ptr<ue2::Component> component;

    const bool allow_vacuous;   //!< HS_FLAG_ALLOWEMPTY specified
    const bool highlander;      //!< HS_FLAG_SINGLEMATCH specified
    const bool prefilter;       //!< HS_FLAG_PREFILTER specified
    som_type som;               //!< chosen SOM mode, or SOM_NONE

    /** \brief index in expressions array passed to \ref hs_compile_multi */
    const unsigned index;

    const ReportID id; //!< user-specified pattern ID
    u64a min_offset;   //!< 0 if not used
    u64a max_offset;   //!< MAX_OFFSET if not used
    u64a min_length;   //!< 0 if not used
};

/**
 * Add an expression to the compiler.
 *
 * @param ng
 *      The global NG object.
 * @param index
 *      The index of the expression (used for errors)
 * @param expression
 *      NULL-terminated PCRE expression
 * @param flags
 *      The full set of Hyperscan flags associated with this rule.
 * @param ext
 *      Struct containing extra parameters for this expression, or NULL if
 *      none.
 * @param actionId
 *      The identifier to associate with the expression; returned by engine on
 *      match.
 */
void addExpression(NG &ng, unsigned index, const char *expression,
                   unsigned flags, const hs_expr_ext *ext, ReportID actionId);

/**
 * Build a Hyperscan database out of the expressions we've been given. A
 * fatal error will result in an exception being thrown.
 *
 * @param ng
 *      The global NG object.
 * @param[out] length
 *      The number of bytes occupied by the compiled structure.
 * @return
 *      The compiled structure. Should be deallocated with the
 *      hs_database_free() function.
 */
struct hs_database *build(NG &ng, unsigned int *length);

/**
 * Constructs an NFA graph from the given expression tree.
 *
 * @param rm
 *      Global ReportManager for this compile.
 * @param cc
 *      Global compile context for this compile.
 * @param expr
 *      ParsedExpression object.
 * @return
 *      nullptr on error.
 */
std::unique_ptr<NGWrapper> buildWrapper(ReportManager &rm,
                                        const CompileContext &cc,
                                        const ParsedExpression &expr);

/**
 * Build a platform_t out of a target_t.
 */
platform_t target_to_platform(const target_t &target_info);

#if defined(DUMP_SUPPORT) || defined(DEBUG)
void dumpExpression(const ParsedExpression &expr, const char *stage,
                    const Grey &grey);
#else
static really_inline
void dumpExpression(UNUSED const ParsedExpression &expr,
                    UNUSED const char *stage, UNUSED const Grey &grey) {
}

#endif

} // namespace

#endif // COMPILER_H
