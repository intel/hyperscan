/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#ifndef COMMON_H
#define COMMON_H

#include <cstddef>
#include <string>
#include <vector>

enum ColliderMode {
    MODE_BLOCK,
    MODE_STREAMING,
    MODE_VECTORED,
    MODE_HYBRID
};

extern unsigned numThreads;
extern enum ColliderMode colliderMode;
extern unsigned int somFlags;
extern bool loadDatabases;
extern bool saveDatabases;
extern bool saveCorpora;
extern std::string saveCorporaFile;
extern std::string serializePath;
extern bool echo_matches;
extern int g_quiet;
extern bool g_verbose;
extern std::string g_exprPath;
extern std::vector<std::string> g_signatureFiles;
extern bool g_allSignatures;
extern bool g_ue2CompileAll;
extern unsigned g_streamBlocks;
extern unsigned long long g_streamOffset;
extern std::string g_corpora_prefix;
extern std::string g_corpora_suffix;
extern unsigned multicompile_bands;
extern std::string g_corporaFile;
extern std::vector<unsigned> g_signatures;
extern unsigned long int g_matchLimit;
extern unsigned long int g_matchLimitRecursion;
extern unsigned min_ue2_align;
extern unsigned max_ue2_align;
extern size_t g_memoryLimit;
extern bool force_utf8;
extern int force_prefilter;
extern int no_groups;
extern unsigned somPrecisionMode;
extern unsigned limit_matches;
extern unsigned randomSeed;
extern bool use_random_alignment;
extern bool use_PCRE;
extern bool use_NFA;
extern bool use_UE2;
extern bool use_copy_scratch;
extern bool use_copy_stream;
extern bool use_mangle_scratch;
extern bool use_compress_expand;
extern bool use_compress_reset_expand;
extern int abort_on_failure;
extern int no_signal_handler;
extern bool force_edit_distance;
extern unsigned edit_distance;

// Constants
static const unsigned long int DEFAULT_PCRE_MATCH_LIMIT = 10*1000*1000;
static const unsigned long int DEFAULT_PCRE_MATCH_RECURSION_LIMIT = 10000;
#define MAX_MAX_UE2_ALIGN 64
#endif
