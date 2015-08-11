/*
 * Copyright (c) 2016, Intel Corporation
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
 *\brief Tamarama: container engine for exclusive engines,
 *                 data structures.
 */

/* Tamarama bytecode layout:
 * * |-----|
 * * |     | struct NFA
 * * |-----|
 * * |     | struct Tamarama
 * * |     |
 * * |-----|
 * * |     | top remapping table:
 * * |     | stores top base for each subengine.
 * * |     | old_top = remapped_top - top_base;
 * * |     | The size of table is equal to the number of subengines.
 * * ...
 * * |     |
 * * |-----|
 * * |     | offsets from the start of struct Tamarama to subengines --\
 * * ...                                                               |
 * * |     |                                          -----------\     |
 * * |-----|                                                     |     |
 * * ||--| | subengine 1 (struct NFA + rest of subengine)     <--/     |
 * * ||  | |                                                           |
 * * ||--| |                                                           |
 * * ||  | |                                                           |
 * * ||  | |                                                           |
 * * ||--| |                                                           |
 * * |     |                                                           |
 * * ||--| | subengine 2 (struct NFA + rest of subengine)      <-------/
 * * ||  | |
 * * ||--| |
 * * ||  | |
 * * ||  | |
 * * ||--| |
 * * |     |
 * * ...
 * * |     |
 * * |-----| total size of tamarama
 * *
 * * Tamarama stream state:
 * *
 * * |---|
 * * |   | active subengine id
 * * |---|
 * * |   | common pool of stream state for each engine
 * * |   |
 * * |   |
 * * ...
 * * |   |
 * * |   |
 * * |---|
 * *
 * * Tamarama scratch space:
 * *
 * * |---|
 * * |   | common pool of scratch for each engine
 * * |   |
 * * |   |
 * * ...
 * * |   |
 * * |   |
 * * |---|
 * */

#ifndef NFA_TAMARAMA_INTERNAL_H
#define NFA_TAMARAMA_INTERNAL_H

#include "ue2common.h"

struct ALIGN_AVX_DIRECTIVE Tamarama {
    u32 numSubEngines;
    u8 activeIdxSize;
};

#endif // NFA_TAMARAMA_INTERNAL_H
