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

#include "ConstComponentVisitor.h"

#include "AsciiComponentClass.h"
#include "ComponentAlternation.h"
#include "ComponentAssertion.h"
#include "ComponentAtomicGroup.h"
#include "ComponentBackReference.h"
#include "ComponentBoundary.h"
#include "ComponentByte.h"
#include "ComponentCondReference.h"
#include "ComponentClass.h"
#include "ComponentEmpty.h"
#include "ComponentEUS.h"
#include "ComponentRepeat.h"
#include "ComponentSequence.h"
#include "ComponentWordBoundary.h"
#include "Utf8ComponentClass.h"

namespace ue2 {

ConstComponentVisitor::~ConstComponentVisitor() {
    // empty
}

// Default implementations.

DefaultConstComponentVisitor::DefaultConstComponentVisitor() {}
DefaultConstComponentVisitor::~DefaultConstComponentVisitor() {}

#define DEFAULT_FUNCS(comp)                                                    \
    void DefaultConstComponentVisitor::pre(const comp &) {}                    \
    void DefaultConstComponentVisitor::during(const comp &) {}                 \
    void DefaultConstComponentVisitor::post(const comp &) {}

DEFAULT_FUNCS(AsciiComponentClass)
DEFAULT_FUNCS(ComponentAlternation)
DEFAULT_FUNCS(ComponentAssertion)
DEFAULT_FUNCS(ComponentAtomicGroup)
DEFAULT_FUNCS(ComponentBackReference)
DEFAULT_FUNCS(ComponentBoundary)
DEFAULT_FUNCS(ComponentByte)
DEFAULT_FUNCS(ComponentCondReference)
DEFAULT_FUNCS(ComponentEmpty)
DEFAULT_FUNCS(ComponentEUS)
DEFAULT_FUNCS(ComponentRepeat)
DEFAULT_FUNCS(ComponentSequence)
DEFAULT_FUNCS(ComponentWordBoundary)
DEFAULT_FUNCS(UTF8ComponentClass)

} // namespace ue2
