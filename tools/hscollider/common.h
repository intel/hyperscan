/*
 * Copyright (C) 2025 Intel Corporation
 *
 * This software and the related documents are Intel copyrighted materials,
 * and your use of them is governed by the express license under which they were
 * provided to you ("License"). Unless the License provides otherwise,
 * you may not use, modify, copy, publish, distribute, disclose or transmit this
 * software or the related documents without Intel's prior written permission.
 *
 * This software and the related documents are provided as is, with no express or
 * implied warranties, other than those that are expressly stated in the License.
 */

#ifndef COMMON_H
#define COMMON_H

#include <cstddef>
#include <string>
#include <vector>
#include <cstdint>

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
extern uint64_t randomSeed;
extern bool use_random_alignment;
extern bool use_PCRE;
extern bool use_NFA;
extern bool use_UE2;
extern bool use_copy_scratch;
extern bool use_copy_stream;
extern bool use_mangle_scratch;
extern bool use_compress_expand;
extern bool use_compress_reset_expand;
extern bool use_literal_api;
extern bool use_rliteral_api;
extern int abort_on_failure;
extern int no_signal_handler;
extern bool force_edit_distance;
extern unsigned edit_distance;
extern bool use_universal_database;

// Constants
static const unsigned long int DEFAULT_PCRE_MATCH_LIMIT = 10*1000*1000;
static const unsigned long int DEFAULT_PCRE_MATCH_RECURSION_LIMIT = 10000;
#define MAX_MAX_UE2_ALIGN 64
#endif
