/*
 * Copyright (c) 2015-2020, Intel Corporation
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
    \brief Declarations for the main NFA engine types and structures.
*/
#ifndef NFA_INTERNAL_H
#define NFA_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "ue2common.h"

// Constants

#define MO_INVALID_IDX  0xffffffff /**< index meaning value is invalid */

// Flags (used in NFA::flags)

#define NFA_ACCEPTS_EOD 1U     /**< can produce matches on EOD. */
#define NFA_ZOMBIE      2U     /**< supports zombies */

// Common data structures for NFAs

enum NFAEngineType {
    LIMEX_NFA_32,
    LIMEX_NFA_64,
    LIMEX_NFA_128,
    LIMEX_NFA_256,
    LIMEX_NFA_384,
    LIMEX_NFA_512,
    MCCLELLAN_NFA_8,    /**< magic pseudo nfa */
    MCCLELLAN_NFA_16,   /**< magic pseudo nfa */
    GOUGH_NFA_8,        /**< magic pseudo nfa */
    GOUGH_NFA_16,       /**< magic pseudo nfa */
    MPV_NFA,            /**< magic pseudo nfa */
    LBR_NFA_DOT,        /**< magic pseudo nfa */
    LBR_NFA_VERM,       /**< magic pseudo nfa */
    LBR_NFA_NVERM,      /**< magic pseudo nfa */
    LBR_NFA_SHUF,       /**< magic pseudo nfa */
    LBR_NFA_TRUF,       /**< magic pseudo nfa */
    CASTLE_NFA,         /**< magic pseudo nfa */
    SHENG_NFA,          /**< magic pseudo nfa */
    TAMARAMA_NFA,       /**< magic nfa container */
    MCSHENG_NFA_8,      /**< magic pseudo nfa */
    MCSHENG_NFA_16,     /**< magic pseudo nfa */
    SHENG_NFA_32,       /**< magic pseudo nfa */
    SHENG_NFA_64,       /**< magic pseudo nfa */
    MCSHENG_64_NFA_8,   /**< magic pseudo nfa */
    MCSHENG_64_NFA_16,  /**< magic pseudo nfa */
    /** \brief bogus NFA - not used */
    INVALID_NFA
};

/** \brief header for the NFA implementation. */
struct ALIGN_CL_DIRECTIVE NFA {
    u32 flags;

    /** \brief The size in bytes of the NFA engine. The engine is
     * serialized to the extent that copying length bytes back into a
     * 16-byte aligned memory location yields a structure that has the same
     * behaviour as the original engine. */
    u32 length;

    /** \brief Active implementation used by this NFAEngineType */
    u8 type;

    u8 rAccelType;
    u8 rAccelOffset;
    u8 maxBiAnchoredWidth; /**< if non zero, max width of the block */

    union {
        u8 c;
        u16 dc;
        u8 array[2];
    } rAccelData;

    u32 queueIndex; /**< index of the associated queue in scratch */

    /** \brief The number of valid positions/states for this NFA. Debug only */
    u32 nPositions;

    /** \brief Size of the state required in scratch space.
     *
     * This state has less strict size requirements (as it doesn't go in stream
     * state) and does not persist between stream writes.
     */
    u32 scratchStateSize;

    /** \brief Size of the state required in stream state.
     *
     * This encompasses all state stored by the engine that must persist between
     * stream writes. */
    u32 streamStateSize;

    u32 maxWidth; /**< longest possible match in this NFA, 0 if unbounded */
    u32 minWidth; /**< minimum bytes required to match this NFA */
    u32 maxOffset; /**< non zero: maximum offset this pattern can match at */

    /* Note: implementation (e.g. a LimEx) directly follows struct in memory */
} ;

// Accessor macro for the implementation NFA: we do things this way to avoid
// type-punning warnings.
#define getImplNfa(nfa) \
    ((const void *)((const char *)(nfa) + sizeof(struct NFA)))

// Non-const version of the above, used at compile time.
#define getMutableImplNfa(nfa)     ((char *)(nfa) + sizeof(struct NFA))

static really_inline u32 nfaAcceptsEod(const struct NFA *nfa) {
    return nfa->flags & NFA_ACCEPTS_EOD;
}

static really_inline u32 nfaSupportsZombie(const struct NFA *nfa) {
    return nfa->flags & NFA_ZOMBIE;
}

/** \brief True if the given type (from NFA::type) is a McClellan DFA. */
static really_inline int isMcClellanType(u8 t) {
    return t == MCCLELLAN_NFA_8 || t == MCCLELLAN_NFA_16;
}

/** \brief True if the given type (from NFA::type) is a Sheng-McClellan hybrid
 * DFA. */
static really_inline int isShengMcClellanType(u8 t) {
    return t == MCSHENG_NFA_8 || t == MCSHENG_NFA_16 ||
           t == MCSHENG_64_NFA_8 || t == MCSHENG_64_NFA_16;
}

/** \brief True if the given type (from NFA::type) is a Gough DFA. */
static really_inline int isGoughType(u8 t) {
    return t == GOUGH_NFA_8 || t == GOUGH_NFA_16;
}

/** \brief True if the given type (from NFA::type) is a Sheng DFA. */
static really_inline int isSheng16Type(u8 t) {
    return t == SHENG_NFA;
}

/** \brief True if the given type (from NFA::type) is a Sheng32 DFA. */
static really_inline int isSheng32Type(u8 t) {
    return t == SHENG_NFA_32;
}

/** \brief True if the given type (from NFA::type) is a Sheng64 DFA. */
static really_inline int isSheng64Type(u8 t) {
    return t == SHENG_NFA_64;
}

/** \brief True if the given type (from NFA::type) is a Sheng16/32/64 DFA. */
static really_inline int isShengType(u8 t) {
    return t == SHENG_NFA || t == SHENG_NFA_32 || t == SHENG_NFA_64;
}

/**
 * \brief True if the given type (from NFA::type) is a McClellan, Gough or
 * Sheng DFA.
 */
static really_inline int isDfaType(u8 t) {
    return isMcClellanType(t) || isGoughType(t) || isShengType(t)
        || isShengMcClellanType(t);
}

static really_inline int isBigDfaType(u8 t) {
    return t == MCCLELLAN_NFA_16 || t == MCSHENG_NFA_16 || t == GOUGH_NFA_16;
}

static really_inline int isSmallDfaType(u8 t) {
    return isDfaType(t) && !isBigDfaType(t);
}

/** \brief True if the given type (from NFA::type) is an NFA. */
static really_inline int isNfaType(u8 t) {
    switch (t) {
    case LIMEX_NFA_32:
    case LIMEX_NFA_64:
    case LIMEX_NFA_128:
    case LIMEX_NFA_256:
    case LIMEX_NFA_384:
    case LIMEX_NFA_512:
        return 1;
    default:
        break;
    }
    return 0;
}

/** \brief True if the given type (from NFA::type) is an LBR. */
static really_inline
int isLbrType(u8 t) {
    return t == LBR_NFA_DOT || t == LBR_NFA_VERM || t == LBR_NFA_NVERM ||
           t == LBR_NFA_SHUF || t == LBR_NFA_TRUF;
}

/** \brief True if the given type (from NFA::type) is a container engine. */
static really_inline
int isContainerType(u8 t) {
    return t == TAMARAMA_NFA;
}

static really_inline
int isMultiTopType(u8 t) {
    return !isDfaType(t) && !isLbrType(t);
}

/** Macros used in place of unimplemented NFA API functions for a given
 * engine. */
#if !defined(_WIN32)

/* Use for functions that return an integer. */
#define NFA_API_NO_IMPL(...)                                                   \
    ({                                                                         \
        assert(!"not implemented for this engine!");                           \
        0; /* return value, for places that need it */                         \
    })

/* Use for _zombie_status functions. */
#define NFA_API_ZOMBIE_NO_IMPL(...)                                            \
    ({                                                                         \
        assert(!"not implemented for this engine!");                           \
        NFA_ZOMBIE_NO;                                                         \
    })

#else

/* Simpler implementation for compilers that don't like the GCC extension used
 * above. */
#define NFA_API_NO_IMPL(...)        0
#define NFA_API_ZOMBIE_NO_IMPL(...) NFA_ZOMBIE_NO

#endif

#ifdef __cplusplus
}
#endif

#endif
