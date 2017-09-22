/*
 * Copyright (c) 2017, Intel Corporation
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

#ifndef ROSE_BUILD_RESOURCES_H
#define ROSE_BUILD_RESOURCES_H

namespace ue2 {

/**
 * \brief Structure tracking which resources are used by this Rose instance at
 * runtime.
 *
 * We use this to control how much initialisation we need to do at the
 * beginning of a stream/block at runtime.
 */
struct RoseResources {
    bool has_outfixes = false;
    bool has_suffixes = false;
    bool has_leftfixes = false;
    bool has_literals = false;
    bool has_states = false;
    bool checks_groups = false;
    bool has_lit_delay = false;
    bool has_lit_check = false; // long literal support
    bool has_anchored = false;
    bool has_anchored_multiple = false; /* multiple anchored dfas */
    bool has_anchored_large = false; /* mcclellan 16 anchored dfa */
    bool has_floating = false;
    bool has_eod = false;
};

}

#endif
