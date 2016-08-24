/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief Compile-time error utils.
 */
#include "allocator.h"
#include "error.h"
#include "ue2common.h"
#include "hs_compile.h"
#include "util/compile_error.h"

#include <cstring>
#include <string>

using std::string;

static const char failureNoMemory[] = "Unable to allocate memory.";
static const char failureInternal[] = "Internal error.";
static const char failureBadAlloc[] = "Allocator returned misaligned memory.";

extern const hs_compile_error_t hs_enomem = {
    const_cast<char *>(failureNoMemory), 0
};
extern const hs_compile_error_t hs_einternal = {
    const_cast<char *>(failureInternal), 0
};
extern const hs_compile_error_t hs_badalloc = {
    const_cast<char *>(failureBadAlloc), 0
};

namespace ue2 {

hs_compile_error_t *generateCompileError(const string &err, int expression) {
    hs_compile_error_t *ret =
        (struct hs_compile_error *)hs_misc_alloc(sizeof(hs_compile_error_t));
    if (ret) {
        hs_error_t e = hs_check_alloc(ret);
        if (e != HS_SUCCESS) {
            hs_misc_free(ret);
            return const_cast<hs_compile_error_t *>(&hs_badalloc);
        }
        char *msg = (char *)hs_misc_alloc(err.size() + 1);
        if (msg) {
            e = hs_check_alloc(msg);
            if (e != HS_SUCCESS) {
                hs_misc_free(msg);
                return const_cast<hs_compile_error_t *>(&hs_badalloc);
            }
            memcpy(msg, err.c_str(), err.size() + 1);
            ret->message = msg;
        } else {
            hs_misc_free(ret);
            ret = nullptr;
        }
    }

    if (!ret || !ret->message) {
        return const_cast<hs_compile_error_t *>(&hs_enomem);
    }

    ret->expression = expression;

    return ret;
}

hs_compile_error_t *generateCompileError(const CompileError &e) {
    return generateCompileError(e.reason, e.hasIndex ? (int)e.index : -1);
}

void freeCompileError(hs_compile_error_t *error) {
    if (!error) {
        return;
    }
    if (error == &hs_enomem || error == &hs_einternal ||
        error == &hs_badalloc) {
        // These are not allocated.
        return;
    }

    hs_misc_free(error->message);
    hs_misc_free(error);
}

} // namespace ue2
