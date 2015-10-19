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

#include "internal_report.h"
#include "report.h"
#include "report_manager.h"

namespace ue2 {

void writeInternalReport(const Report &report, const ReportManager &rm,
                         internal_report *ir) {
    assert(ir);
    assert(ISALIGNED(ir));

    ir->type = report.type;
    ir->hasBounds = report.hasBounds() ? 1 : 0;
    ir->quashSom = report.quashSom ? 1 : 0;
    ir->minOffset = report.minOffset;
    ir->maxOffset = report.maxOffset;
    ir->minLength = report.minLength;
    ir->ekey = report.ekey;
    ir->offsetAdjust = report.offsetAdjust;
    ir->onmatch = report.onmatch;

    switch (report.type) {
    case INTERNAL_ROSE_CHAIN:
        ir->aux.topSquashDistance = report.topSquashDistance;
        break;
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
        ir->aux.revNfaIndex = report.revNfaIndex;
        break;
    default:
        ir->aux.somDistance = report.somDistance;
        break;
    }

    // Dedupe keys are managed by ReportManager.
    ir->dkey = rm.getDkey(report);
}

} // namespace ue2
