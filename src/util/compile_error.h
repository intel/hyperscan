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

#ifndef UTIL_COMPILE_ERROR_H
#define UTIL_COMPILE_ERROR_H

#include <cassert>
#include <stdexcept>
#include <string>

#include "ue2common.h"

namespace ue2 {

/** \brief Error thrown by the compiler, can reference a specific expression
 * index. */
class CompileError {
public:
    // Note: 'why' should describe why the error occurred and end with a
    // full stop, but no line break.
    explicit CompileError(const std::string &why);
    CompileError(u32 index, const std::string &why);

    virtual ~CompileError();

    void setExpressionIndex(u32 index);

    std::string reason; //!< Reason for the error
    bool hasIndex; //!< Does it reference a specific expression?
    u32 index; //!< The index of the expression referred to
};

/** \brief Error thrown by the compiler when an arbitrary resource limit (as
 * specified in the grey box) is exceeded. */
class ResourceLimitError : public CompileError {
public:
    ResourceLimitError();
    ~ResourceLimitError() override;
};

} // namespace ue2

#endif
